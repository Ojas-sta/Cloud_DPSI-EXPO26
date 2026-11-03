"""
Project Cloud: Central Supervisory Finite State Machine Node
Orchestrates autonomous lifecycle, persona transitions, navigation pacing,
VIP narration, and fail-safe safety preemption.
Reference: PROJECT.md Section 7.5 & Section 8
"""

import time
from typing import Optional, List, Dict, Any

from cloud_core.bus import CloudNode
from cloud_core.types import (
    Twist,
    LEDCommand,
    SpillAlert,
    TriggerTourRequest,
    TriggerTourResponse,
    RobotStateReport
)
from cloud_navigation.topological_graph import TopologicalGraph
from cloud_cognition.persona_manager import PersonaManager
from cloud_cognition.rag_engine import GroundedRAGEngine
from cloud_cognition.speech_pipeline import SpeechPipelineNode

class CloudSupervisoryFSM(CloudNode):
    """
    Central brain supervisory finite state machine.
    """
    # States
    STATE_IDLE_STANDBY = "STATE_IDLE_STANDBY"
    STATE_ARGUS_PATROL = "STATE_ARGUS_PATROL"
    STATE_HAZARD_ALERT = "STATE_HAZARD_ALERT"
    STATE_DOCENT_GREET = "STATE_DOCENT_GREET"
    STATE_DOCENT_CRUISE = "STATE_DOCENT_CRUISE"
    STATE_DOCENT_PIVOT = "STATE_DOCENT_PIVOT"
    STATE_DOCENT_NARRATE = "STATE_DOCENT_NARRATE"
    STATE_STAGE_FINALE = "STATE_STAGE_FINALE"
    STATE_PRONE_REST = "STATE_PRONE_REST"

    def __init__(self, node_name: str = "cloud_supervisory_fsm"):
        super().__init__(node_name)
        self.state: str = self.STATE_IDLE_STANDBY
        self.previous_state: str = self.STATE_IDLE_STANDBY

        # Subsystems
        self.graph = TopologicalGraph()
        self.persona = PersonaManager(initial_persona=PersonaManager.PERSONA_DOCENT)
        self.rag = GroundedRAGEngine()
        self.speech = SpeechPipelineNode("fsm_speech_sub")

        self.current_node_id: str = "NODE_00"
        self.target_node_id: Optional[str] = None
        self.last_hazard_alert: Optional[SpillAlert] = None
        self.tour_active: bool = False

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, "/cloud/cmd_vel")
        self.pub_led = self.create_publisher(LEDCommand, "/cloud/led_cmd")
        self.pub_fsm_state = self.create_publisher(str, "/cloud/fsm_state")

        # Subscriptions
        self.sub_button = self.create_subscription(
            bool, "/cloud/console_button", self._on_console_button
        )
        self.sub_spill = self.create_subscription(
            SpillAlert, "/cloud/spill_hazard", self._on_spill_alert
        )
        self.sub_arrived = self.create_subscription(
            bool, "/cloud/navigation_arrived", self._on_navigation_arrived
        )
        self.sub_pivot_done = self.create_subscription(
            bool, "/cloud/pivot_completed", self._on_pivot_completed
        )

        # Service Server
        self.srv_tour = self.create_service(
            TriggerTourRequest, "/cloud/trigger_tour", self._handle_trigger_tour
        )

        # Initial LED state
        self._set_led_for_state()
        self.logger.info("Cloud Supervisory FSM initialized in STATE_IDLE_STANDBY.")

    def transition_to(self, new_state: str):
        old_state = self.state
        self.previous_state = old_state
        self.state = new_state
        self.logger.info(f"FSM TRANSITION: {old_state} -> {new_state}")
        self.pub_fsm_state.publish(new_state)
        self._set_led_for_state()
        self._execute_state_entry()

    def _set_led_for_state(self):
        if self.state == self.STATE_HAZARD_ALERT:
            cmd = self.persona.get_hazard_led_command()
        elif self.state in [self.STATE_DOCENT_GREET, self.STATE_DOCENT_NARRATE, self.STATE_STAGE_FINALE]:
            cmd = self.persona.get_led_command()
        elif self.state == self.STATE_DOCENT_CRUISE:
            cmd = LEDCommand(mode=LEDCommand.MODE_CHASE, red=80, green=140, blue=255, frequency_hz=1.0)
        elif self.state == self.STATE_PRONE_REST:
            cmd = LEDCommand(mode=LEDCommand.MODE_OFF)
        else:
            cmd = LEDCommand(mode=LEDCommand.MODE_BREATHE, red=0, green=160, blue=220, frequency_hz=0.5)
        self.pub_led.publish(cmd)

    def _execute_state_entry(self):
        if self.state == self.STATE_HAZARD_ALERT:
            # Emergency freeze
            self.pub_cmd_vel.publish(Twist())
            self.speech.speak("Caution! Chemical or liquid spill hazard intercepted. Autonomous transit halted.")

        elif self.state == self.STATE_DOCENT_GREET:
            welcome_text = self.rag.get_narration("reception_welcome")
            self.speech.speak(welcome_text)

        elif self.state == self.STATE_DOCENT_CRUISE:
            target_node = self.graph.get_node(self.target_node_id)
            if target_node:
                self.logger.info(f"Cruising towards {target_node.label} (Marker ID {target_node.marker_id}).")

        elif self.state == self.STATE_DOCENT_NARRATE:
            node = self.graph.get_node(self.current_node_id)
            if node and node.narration_key:
                script = self.rag.get_narration(node.narration_key)
                self.speech.speak(script)

        elif self.state == self.STATE_STAGE_FINALE:
            script = self.rag.get_narration("stage_finale")
            self.speech.speak(script)

        elif self.state == self.STATE_PRONE_REST:
            self.pub_cmd_vel.publish(Twist())
            self.speech.speak("Tour concluded. Project Cloud docking into autonomous prone recharge.")

    def _on_spill_alert(self, alert: SpillAlert):
        """Immediate priority safety preemption."""
        self.last_hazard_alert = alert
        if self.state != self.STATE_HAZARD_ALERT:
            self.logger.warn(f"PREEMPTION TRIGGERED by SpillAlert! Halting all operations.")
            self.transition_to(self.STATE_HAZARD_ALERT)

    def clear_hazard(self):
        """Operator or steward clears the containment zone."""
        if self.state == self.STATE_HAZARD_ALERT:
            self.logger.info("Hazard cleared by steward. Resuming operational state.")
            self.transition_to(self.previous_state if self.previous_state != self.STATE_HAZARD_ALERT else self.STATE_IDLE_STANDBY)

    def _on_console_button(self, pressed: bool):
        """Tactile push-to-proceed button handler."""
        if not pressed:
            return
        self.logger.info(f"Push-to-proceed received in state {self.state}")

        if self.state in [self.STATE_IDLE_STANDBY, self.STATE_DOCENT_GREET]:
            # Begin tour to Booth 1
            self.tour_active = True
            self.target_node_id = "NODE_10"
            self.transition_to(self.STATE_DOCENT_CRUISE)

        elif self.state == self.STATE_DOCENT_NARRATE:
            # Advance to next booth or stage finale
            next_node = self.graph.get_next_tour_node(self.current_node_id)
            if next_node:
                self.target_node_id = next_node
                if next_node == "NODE_30":
                    self.transition_to(self.STATE_DOCENT_CRUISE)
                elif next_node == "NODE_99":
                    self.transition_to(self.STATE_PRONE_REST)
                else:
                    self.transition_to(self.STATE_DOCENT_CRUISE)
            else:
                self.transition_to(self.STATE_PRONE_REST)

        elif self.state == self.STATE_STAGE_FINALE:
            # Tour complete; transition to prone rest in docking bay
            self.target_node_id = "NODE_99"
            self.transition_to(self.STATE_PRONE_REST)

    def _on_navigation_arrived(self, arrived: bool):
        """Arrival at booth landmark standoff distance."""
        if arrived and self.state == self.STATE_DOCENT_CRUISE:
            self.current_node_id = self.target_node_id or self.current_node_id
            self.logger.info(f"Arrived at landmark {self.current_node_id}. Triggering Presenter Pivot.")
            self.transition_to(self.STATE_DOCENT_PIVOT)

    def _on_pivot_completed(self, done: bool):
        """Presenter pivot complete; robot now faces evaluators."""
        if done and self.state == self.STATE_DOCENT_PIVOT:
            if self.current_node_id == "NODE_30":
                self.transition_to(self.STATE_STAGE_FINALE)
            else:
                self.transition_to(self.STATE_DOCENT_NARRATE)

    def _handle_trigger_tour(self, request: TriggerTourRequest) -> TriggerTourResponse:
        self.logger.info(f"Triggering tour profile: {request.tour_profile}")
        self.current_node_id = request.starting_node
        self.transition_to(self.STATE_DOCENT_GREET)
        return TriggerTourResponse(
            accepted=True,
            message="VIP Tour initiated.",
            initial_state=self.STATE_DOCENT_GREET
        )
