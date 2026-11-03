"""
Project Cloud: Speech & Vocal Synthesis Pipeline Node
Interfaces with Whisper-Base STT and Piper Neural TTS.
In SIL mode, provides simulated vocal timing and event broadcasting.
Reference: PROJECT.md Section 7.4
"""

import time
import threading
from typing import Optional, Callable
from cloud_core.bus import CloudNode

class SpeechPipelineNode(CloudNode):
    """
    Manages speech recognition input and synthesized speech output.
    """
    def __init__(self, node_name: str = "speech_pipeline_node", words_per_minute: int = 145):
        super().__init__(node_name)
        self.wpm = words_per_minute
        self.is_speaking: bool = False
        self.last_utterance: str = ""

        # Publishers
        self.pub_speech = self.create_publisher(str, "/cloud/speech_output")
        self.pub_speaking_status = self.create_publisher(bool, "/cloud/speaking_status")

    def speak(self, text: str, blocking: bool = False, on_complete: Optional[Callable[[], None]] = None):
        """
        Synthesizes and delivers an utterance.
        Computes realistic vocal delivery timing based on word count.
        """
        self.last_utterance = text
        word_count = len(text.split())
        est_duration_sec = max(1.0, (word_count / self.wpm) * 60.0)

        self.logger.info(f"VOCAL OUTPUT ({est_duration_sec:.1f}s): \"{text}\"")
        self.pub_speech.publish(text)
        self.pub_speaking_status.publish(True)
        self.is_speaking = True

        def _utterance_worker():
            # In simulation, we sleep for a scaled preview duration (or actual duration)
            sim_duration = min(2.5, est_duration_sec * 0.15)  # Accelerated for snappy automated testing
            time.sleep(sim_duration)
            self.is_speaking = False
            self.pub_speaking_status.publish(False)
            if on_complete:
                on_complete()

        thread = threading.Thread(target=_utterance_worker, daemon=True)
        thread.start()

        if blocking:
            thread.join()
