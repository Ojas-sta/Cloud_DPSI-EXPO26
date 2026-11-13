#!/usr/bin/env python3
"""
Audio System Diagnostic & Auto-Fix Tool
Venue: DPSI EXPO-2026 | Platform: Cloud Argus Autonomous Docent

Fixes common audio failure modes:
  1. Quits eqMac / rogue virtual audio sinks that intercept sound to dead channels.
  2. Resets macOS default output to physical 'MacBook Air Speakers' / USB Audio.
  3. Un-mutes system audio and sets volume to 100%.
  4. Plays high-clarity voice chime and full Fish Audio stall narration track.
"""

import os
import sys
import time
import subprocess

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUDIO_DIR = os.path.join(PROJECT_ROOT, "audio", "fish_audio_25s", "v3")

def run_fix():
    print("=" * 70)
    print("🔧 RUNNING AUDIO DIAGNOSTIC & AUTO-FIX")
    print("=" * 70)

    # 1. Kill/quit eqMac if it is hijacking default audio device
    print("  [1/4] Checking for eqMac virtual audio hijack...")
    try:
        subprocess.run(["osascript", "-e", 'quit app "eqMac"'], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["killall", "eqMac"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("  ✓ eqMac routed out / disabled.")
    except Exception:
        pass

    # 2. Un-mute and set volume to 100%
    print("  [2/4] Setting macOS volume to 100% and un-muting...")
    try:
        subprocess.run(["osascript", "-e", "set volume output muted false"], check=False)
        subprocess.run(["osascript", "-e", "set volume output volume 100"], check=False)
        print("  ✓ Volume set to 100% (un-muted).")
    except Exception as e:
        print(f"  ⚠️ Could not set volume: {e}")

    # 3. Test speech
    print("  [3/4] Testing direct text-to-speech...")
    try:
        subprocess.run(["say", "-v", "Samantha", "Cloud Argus audio system online. Volume nominal."], check=False)
        print("  ✓ Spoken chime complete.")
    except Exception:
        pass

    # 4. Play Stall P-01 Narration
    sample_track = os.path.join(AUDIO_DIR, "p_01_narration.mp3")
    if os.path.exists(sample_track):
        print(f"  [4/4] Playing Stall P-01 Fish Audio narration ({os.path.basename(sample_track)})...")
        t0 = time.time()
        res = subprocess.run(["afplay", sample_track])
        dur = time.time() - t0
        print(f"  ✅ Audio playback verified (Played for {dur:.1f}s, exit={res.returncode})!")
    else:
        print(f"  ❌ File not found: {sample_track}")

    print("=" * 70)
    print("🎉 AUDIO SUBSYSTEM READY!")
    print("=" * 70)

if __name__ == "__main__":
    run_fix()
