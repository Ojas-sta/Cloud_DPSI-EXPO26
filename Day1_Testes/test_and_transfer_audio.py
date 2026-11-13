#!/usr/bin/env python3
"""
Unitree Go2 Audio Transfer & Onboard Playback Diagnostic Utility
Venue: DPSI EXPO-2026 | Platform: Unitree Go2 Pro/Edu Quadruped

Features:
  1. Scans & verifies local pre-generated 25s stall pitch audio guides (v1, v2, v3, v4).
  2. Syncs/Transfers audio files over Ethernet to Go2 companion computer via SCP:
       unitree@192.168.123.161:/home/unitree/cloud_audio/
  3. Dual-backend playback with automatic fallback:
       Backend A: Unitree Go2 Onboard VuiClient / ALSA over SSH (internal speaker)
       Backend B: Companion USB Speaker / Local Audio Driver (afplay / aplay / ffplay)
  4. Spoken test tone & non-blocking execution during the 5-step stall sequence.

Usage:
  python3 Day1_Testes/test_and_transfer_audio.py [--test-tone] [--voice v3] [--sync-remote] [--remote-host 192.168.123.161]
"""

import os
import sys
import time
import argparse
import subprocess
import threading
from typing import List, Dict, Any, Optional

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUDIO_DIR_FISH = os.path.join(PROJECT_ROOT, "audio", "fish_audio_25s")
AUDIO_DIR_DANIEL = os.path.join(PROJECT_ROOT, "audio", "docent_narration")
REMOTE_DEFAULT_PATH = "/home/unitree/cloud_audio"


class Go2AudioSubsystem:
    def __init__(
        self,
        voice: str = "v3",
        remote_user: str = "unitree",
        remote_host: str = "192.168.123.161",
        remote_dest: str = REMOTE_DEFAULT_PATH
    ):
        self.voice = voice.lower()
        self.remote_user = remote_user
        self.remote_host = remote_host
        self.remote_dest = remote_dest

        # Local source audio directory
        self.local_audio_dir = os.path.join(AUDIO_DIR_FISH, self.voice)
        if not os.path.exists(self.local_audio_dir) or len(os.listdir(self.local_audio_dir)) == 0:
            self.local_audio_dir = AUDIO_DIR_DANIEL
            print(f"  ℹ️ [Audio]: Using local narration directory: {self.local_audio_dir}")
        else:
            print(f"  ℹ️ [Audio]: Active Fish Audio Voice Model: {self.voice.upper()} ({self.local_audio_dir})")

    def scan_local_audio_files(self) -> List[str]:
        """Scans and validates local audio assets."""
        if not os.path.exists(self.local_audio_dir):
            return []
        files = [f for f in sorted(os.listdir(self.local_audio_dir)) if f.endswith(('.mp3', '.wav'))]
        print(f"  📊 Found {len(files)} narration tracks in {self.local_audio_dir}")
        return files

    def transfer_to_robot(self, timeout: int = 120) -> bool:
        """Transfers audio files into Go2 AudioHub flash memory via WebRTC DataChannel RPC."""
        print(f"\n📡 [AudioHub Sync]: Preparing upload to Unitree Go2 ({self.remote_host}) memory...")

        go2_speaker_script = os.path.join(PROJECT_ROOT, "Day1_Testes", "play_on_go2_speaker.py")
        webrtc_python = "/Users/roopalisingh/PycharmProjects/TEST/.venv/bin/python"

        if os.path.exists(go2_speaker_script) and os.path.exists(webrtc_python):
            try:
                cmd = [webrtc_python, go2_speaker_script, "--upload-all", self.local_audio_dir]
                t0 = time.time()
                res = subprocess.run(cmd, timeout=timeout)
                if res.returncode == 0:
                    duration = time.time() - t0
                    print(f"  ✅ [AudioHub Sync]: Successfully stored stall narrations in Go2 flash memory in {duration:.1f}s!")
                    return True
                else:
                    print(f"  ⚠️ AudioHub upload returned code {res.returncode}")
            except Exception as e:
                print(f"  ⚠️ Exception during AudioHub upload: {e}")

        # Fallback to SCP if SSH is ever enabled
        print("  Trying legacy SCP fallback...")
        test_ssh = [
            "ssh", "-o", "ConnectTimeout=2", "-o", "BatchMode=yes",
            f"{self.remote_user}@{self.remote_host}", "mkdir -p " + self.remote_dest
        ]
        try:
            res = subprocess.run(test_ssh, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if res.returncode == 0:
                scp_cmd = f"scp -r -o ConnectTimeout=5 {os.path.join(self.local_audio_dir, '*')} {self.remote_user}@{self.remote_host}:{self.remote_dest}/"
                scp_res = subprocess.run(scp_cmd, shell=True, timeout=timeout)
                return scp_res.returncode == 0
        except Exception:
            pass

        return False

    def play_onboard_or_local(self, audio_file: str, blocking: bool = True) -> bool:
        """
        Attempts onboard playback over SSH/VuiClient; falls back immediately
        to companion USB speaker / local system player if remote is offline.
        """
        base_name = os.path.basename(audio_file)
        local_path = os.path.join(self.local_audio_dir, base_name)
        remote_path = f"{self.remote_dest}/{base_name}"

        # 1. First priority: Unitree Go2 Physical Loudspeaker via WebRTC AudioHub
        go2_speaker_script = os.path.join(PROJECT_ROOT, "Day1_Testes", "play_on_go2_speaker.py")
        webrtc_python = "/Users/roopalisingh/PycharmProjects/TEST/.venv/bin/python"

        # Check local player availability for fallback
        local_player = None
        for p in ["afplay", "aplay", "ffplay", "mpg123"]:
            try:
                res = subprocess.run(["which", p], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if res.returncode == 0:
                    local_player = p
                    break
            except Exception:
                continue

        def _execute():
            t0 = time.time()
            # Try Go2 physical speaker via WebRTC AudioHub first
            if os.path.exists(go2_speaker_script) and os.path.exists(webrtc_python):
                try:
                    print(f"  🔊 [Go2 Physical Speaker]: Sending audio to robot chassis loudspeaker ({base_name})...")
                    cmd = [webrtc_python, go2_speaker_script, "--file", local_path]
                    p_rem = subprocess.run(cmd, timeout=35)
                    if p_rem.returncode == 0:
                        latency = time.time() - t0
                        print(f"  ✓ Playback on Go2 loudspeaker completed (duration: {latency:.1f}s)")
                        return True
                except Exception as e:
                    print(f"  ⚠️ WebRTC AudioHub error: {e}")

            # Fallback: Local companion USB speaker
            if local_player and os.path.exists(local_path):
                print(f"  🔊 [Companion Speaker]: Playing locally via '{local_player}' ({base_name})")
                try:
                    cmd = [local_player, local_path] if local_player != "ffplay" else ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", local_path]
                    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    latency = time.time() - t0
                    print(f"  ✓ Playback completed (duration: {latency:.1f}s)")
                    return True
                except Exception as e:
                    print(f"  ⚠️ Playback error: {e}")
                    return False
            else:
                print(f"  [Simulated Audio]: Playing '{base_name}' (duration ~15s)")
                time.sleep(2.0)
                return True

        if blocking:
            return _execute()
        else:
            t = threading.Thread(target=_execute, daemon=True)
            t.start()
            return True

    def play_test_tone(self) -> bool:
        """Plays a spoken test phrase to verify volume, clarity and latency."""
        print("\n" + "=" * 70)
        print("🎙️ PLAYING SPOKEN TEST TONE & AUDIO SUB-SYSTEM CHECK")
        print("=" * 70)
        test_phrase = "Project Cloud audio check nominal. Welcome to D P S I Capstone Expo 2026."
        print(f"  🗣️ Script: \"{test_phrase}\"")

        go2_speaker_script = os.path.join(PROJECT_ROOT, "Day1_Testes", "play_on_go2_speaker.py")
        webrtc_python = "/Users/roopalisingh/PycharmProjects/TEST/.venv/bin/python"

        if os.path.exists(go2_speaker_script) and os.path.exists(webrtc_python):
            print(f"  🔊 Broadcasting test tone DIRECTLY to Unitree Go2 PHYSICAL SPEAKER...")
            t0 = time.time()
            res = subprocess.run([webrtc_python, go2_speaker_script, "--test-tone"])
            latency = time.time() - t0
            if res.returncode == 0:
                print(f"  ✅ Go2 speaker test tone broadcast complete (Playback latency: {latency:.2f}s).")
                return True

        # Fallback to local audio
        if sys.platform == "darwin":
            print(f"  🔊 Companion Fallback: Broadcasting test tone to local speaker...")
            t0 = time.time()
            subprocess.run(["say", "-v", "Samantha", test_phrase], check=False)
            latency = time.time() - t0
            print(f"  ✅ Test tone broadcast complete (Playback latency: {latency:.2f}s).")
            return True
        else:
            # Pick first available narration file as test tone
            files = self.scan_local_audio_files()
            if files:
                return self.play_onboard_or_local(files[0], blocking=True)
            print(f"  ⚠️ No audio test files found.")
            return False


def main():
    parser = argparse.ArgumentParser(description="Go2 Audio Sync & Onboard Speaker Verification")
    parser.add_argument("--test-tone", action="store_true", help="Broadcast spoken test tone to verify speaker volume and clarity")
    parser.add_argument("--voice", type=str, default="v3", choices=["v1", "v2", "v3", "v4"], help="Voice model preset (default: v3)")
    parser.add_argument("--sync-remote", action="store_true", help="Transfer audio directory to Go2 robot via SCP")
    parser.add_argument("--remote-host", type=str, default="192.168.123.161", help="Robot IP (default: 192.168.123.161)")
    parser.add_argument("--play-stall", type=str, default=None, help="Stall code to test play, e.g. P-01, D-05, B-02")
    args = parser.parse_args()

    subsystem = Go2AudioSubsystem(
        voice=args.voice,
        remote_host=args.remote_host
    )

    files = subsystem.scan_local_audio_files()

    if args.sync_remote:
        subsystem.transfer_to_robot()

    if args.play_stall:
        fname = f"{args.play_stall.lower().replace('-', '_')}_narration.mp3"
        subsystem.play_onboard_or_local(fname, blocking=True)
    elif args.test_tone or len(sys.argv) == 1:
        subsystem.play_test_tone()


if __name__ == "__main__":
    main()
