#!/usr/bin/env python3
"""
Docent Audio Player & Live Narrator CLI
Plays pre-generated 25s Fish Audio guides for any stall across voices v1, v2, v3, v4.
Supports macOS (afplay), Linux/ROS (ffplay/mpg123/aplay), and terminal transcript HUD.
"""

import os
import sys
import json
import time
import argparse
import subprocess

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUDIO_DIR = os.path.join(BASE_DIR, "audio", "fish_audio_25s")
MANIFEST_PATH = os.path.join(AUDIO_DIR, "manifest_25s_all_models.json")
LAYOUT_PATH = os.path.join(BASE_DIR, "config", "expo_hall_layout.yaml")

def load_manifest():
    if os.path.exists(MANIFEST_PATH):
        with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    return []

def play_audio_file(filepath):
    """Play audio file using available audio player without blocking stdout."""
    players = [
        ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", filepath],
        ["afplay", filepath],
        ["mpg123", "-q", filepath],
        ["mpv", "--no-video", filepath]
    ]
    for cmd in players:
        try:
            # Check if player binary exists
            subprocess.run(["which", cmd[0]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            proc = subprocess.Popen(cmd)
            return proc
        except Exception:
            continue
    print(f"⚠️ No suitable audio player found (install ffplay, afplay, or mpg123)")
    return None

def main():
    parser = argparse.ArgumentParser(description="Cloud Argus Autonomous Docent Audio Player")
    parser.add_argument("--stall", "-s", type=str, required=True, help="Stall code, e.g. P-01, D-05, B-02")
    parser.add_argument("--voice", "-v", type=str, default="v3", choices=["v1", "v2", "v3", "v4"], help="Voice model (default: v3)")
    parser.add_argument("--no-play", action="store_true", help="Print info without playing audio")
    args = parser.parse_args()

    stall_code = args.stall.upper().strip()
    voice = args.voice.lower().strip()

    filename = f"{stall_code.lower().replace('-', '_')}_narration.mp3"
    audio_path = os.path.join(AUDIO_DIR, voice, filename)

    if not os.path.exists(audio_path):
        print(f"❌ Error: Audio file not found at: {audio_path}")
        sys.exit(1)

    # Load metadata if manifest exists
    manifest = load_manifest()
    entry = next((item for item in manifest if item.get("code") == stall_code and item.get("voice") == voice), None)

    title = entry.get("title", stall_code) if entry else stall_code
    lead = entry.get("lead", "Unknown") if entry else "N/A"
    script = entry.get("script", "") if entry else ""
    duration = entry.get("duration_sec", 0.0) if entry else 0.0

    print("=" * 70)
    print(f"🎙️  CLOUD ARGUS DOCENT NARRATION HUD")
    print(f"🏛️  Stall: {stall_code} | Project: {title}")
    print(f"👥  Lead/Evaluator: {lead}")
    print(f"🔊  Voice Model: {voice.upper()} | Target Duration: {duration:.1f}s (<= 25s)")
    print(f"📁  File: {audio_path}")
    print("-" * 70)
    print(f"📜  Transcript:\n\"{script}\"")
    print("=" * 70)

    if not args.no_play:
        print(f"▶️  Playing narration...")
        proc = play_audio_file(audio_path)
        if proc:
            proc.wait()
            print("✅ Narration complete.")

if __name__ == "__main__":
    main()
