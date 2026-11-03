"""
Project Cloud: Dual Cognitive Persona Manager
Orchestrates persona switching, system prompts, LED color schemes,
and vocal styles between THE DOCENT and ARGUS.
Reference: PROJECT.md Section 7.4 & 8.2
"""

from typing import Dict, Any
from cloud_core.types import LEDCommand

class PersonaManager:
    """
    Manages operational personas:
    - THE_DOCENT: VIP guide, diplomatic, articulate, inspiring.
    - ARGUS: Makerspace utility steward, terse, safety-first, logistics-focused.
    """
    PERSONA_DOCENT = "THE_DOCENT"
    PERSONA_ARGUS = "ARGUS"

    def __init__(self, initial_persona: str = PERSONA_DOCENT):
        self.active_persona = initial_persona

    def set_persona(self, persona_name: str):
        if persona_name not in [self.PERSONA_DOCENT, self.PERSONA_ARGUS]:
            raise ValueError(f"Unknown persona: {persona_name}")
        self.active_persona = persona_name

    def get_system_prompt(self) -> str:
        if self.active_persona == self.PERSONA_DOCENT:
            return (
                "You are Cloud, an autonomous quadruped robotics docent at the DPSI EXPO 2026. "
                "Your role is to guide evaluators and VIP visitors through student engineering capstones. "
                "You speak with articulate warmth, academic authority, and enthusiastic clarity. "
                "You adhere strictly to verified project specifications and never invent technical facts."
            )
        else:
            return (
                "You are ARGUS, the autonomous ground custodian and inventory logistics steward of the makerspace. "
                "Your role is to safeguard the laboratory, track 50 component bins, report physical hazards, "
                "and maintain strict operational safety. You speak with concise, utilitarian brevity."
            )

    def get_led_command(self) -> LEDCommand:
        """Returns standard ambient NeoPixel LED state for current persona."""
        if self.active_persona == self.PERSONA_DOCENT:
            # Regal Violet Breathe (0.5 Hz)
            return LEDCommand(
                mode=LEDCommand.MODE_BREATHE,
                red=110,
                green=30,
                blue=230,
                frequency_hz=0.5
            )
        else:
            # Tactical Cyan/Electric Blue Solid
            return LEDCommand(
                mode=LEDCommand.MODE_SOLID,
                red=0,
                green=180,
                blue=255,
                frequency_hz=1.0
            )

    def get_hazard_led_command(self) -> LEDCommand:
        """Flashing Amber/Red Strobe for critical alerts."""
        return LEDCommand(
            mode=LEDCommand.MODE_STROBE,
            red=255,
            green=40,
            blue=0,
            frequency_hz=3.0
        )
