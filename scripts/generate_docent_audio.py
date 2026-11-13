#!/usr/bin/env python3
"""
Generates high-fidelity professional Docent tour audio files for all exhibition projects.
Synthesizes using macOS TTS (Daniel voice) and encodes to standard MP3.
"""

import os
import json
import subprocess

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(PROJECT_ROOT, "data", "extracted_expo_projects.json")
AUDIO_DIR = os.path.join(PROJECT_ROOT, "audio", "docent_narration")
SCRIPTS_DIR = os.path.join(PROJECT_ROOT, "audio", "scripts")

os.makedirs(AUDIO_DIR, exist_ok=True)
os.makedirs(SCRIPTS_DIR, exist_ok=True)

with open(JSON_PATH, "r", encoding="utf-8") as f:
    projects = json.load(f)

curated_scripts = {}

for code, p in sorted(projects.items()):
    title = p['title'].replace('&', 'and')
    lead = p['lead'].replace('&', 'and')
    synopsis = p['synopsis'].strip()
    
    # Clean synopsis
    synopsis_clean = synopsis.replace('\n', ' ').replace('"', '').strip()
    
    # Docent narration template
    if code == "DISPLAY":
        narration = (
            f"Welcome to the Special Exhibition Showcase: {title}. "
            f"Supervised by {lead}, this live robotics and engineering demonstration highlights "
            f"advanced humanoid simulations, mechatronics, and autonomous systems engineered by DPSI students."
        )
    else:
        narration = (
            f"Welcome to Stall {code}: {title}. "
            f"Guided by {lead}, {synopsis_clean}"
        )
    
    curated_scripts[code] = {
        "code": code,
        "title": title,
        "lead": lead,
        "script": narration,
        "students": p["students"],
        "audio_file": f"{code.lower().replace('-', '_')}_narration.mp3"
    }

# Save complete scripts manifest
manifest_path = os.path.join(PROJECT_ROOT, "config", "docent_audio_manifest.json")
with open(manifest_path, "w", encoding="utf-8") as f:
    json.dump(curated_scripts, f, indent=2, ensure_ascii=False)

print(f"Generated narration manifest for {len(curated_scripts)} projects.")

# Synthesize audio files
for code, item in curated_scripts.items():
    filename_base = item["audio_file"].replace(".mp3", "")
    aiff_path = f"/tmp/{filename_base}.aiff"
    mp3_path = os.path.join(AUDIO_DIR, item["audio_file"])
    
    text = item["script"]
    
    # say command
    subprocess.run(["say", "-v", "Daniel", "-r", "175", "-o", aiff_path, text], check=True)
    # ffmpeg convert
    subprocess.run(["/opt/homebrew/bin/ffmpeg", "-y", "-i", aiff_path, "-codec:a", "libmp3lame", "-qscale:a", "2", mp3_path], 
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    
    if os.path.exists(aiff_path):
        os.remove(aiff_path)
    
    print(f"  ✓ [{code}] {item['title']} -> {os.path.basename(mp3_path)}")

print("\n🎉 All audio files successfully synthesized!")
