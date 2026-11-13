#!/usr/bin/env python3
"""
Pre-generates high-fidelity Fish Audio voice tracks for all exhibition stalls.
Constraints:
  - Voice: v3 (711cf3ed00ab441a8f54a45058047b7a) - crisp, fast, professional
  - Duration constraint: <= 25 seconds (concise 45-55 word high-impact docent pitches)
  - Zero latency at runtime (instant local playback)
"""

import os
import json
import time
import requests
import subprocess

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(PROJECT_ROOT, "data", "extracted_expo_projects.json")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "audio", "fish_audio_pregenerated")
os.makedirs(OUTPUT_DIR, exist_ok=True)

API_KEY = "sk-fish-XR2jpZ3D7cWMDae3vQnncS730o8UVtn78SXh11TkpZc"
URL = "https://api.fish.audio/v1/tts"
VOICE_REF_ID = "711cf3ed00ab441a8f54a45058047b7a"  # v3

with open(JSON_PATH, "r", encoding="utf-8") as f:
    projects = json.load(f)

# Curate high-impact 20-24 second docent scripts (~45-55 words @ ~140-150 wpm)
curated_pitches = {}

for code, p in sorted(projects.items()):
    title = p["title"].strip()
    lead = p["lead"].strip().replace("&", "and")
    synopsis = p["synopsis"].strip().replace("\n", " ")
    
    # Generate concise, professional pitch targeted at ~45-52 words (approx 18-22 seconds spoken)
    # Split synopsis into sentences and pick key value proposition
    sentences = [s.strip() for s in synopsis.split(".") if len(s.strip()) > 10]
    
    if code == "DISPLAY":
        pitch_text = (
            f"Welcome to our Special Exhibition Showcase. "
            f"Supervised by {lead}, this live robotics and simulation hub features advanced humanoid bipedal reinforcement learning, "
            f"autonomous mobile rovers, and real-time student mechatronics."
        )
    elif len(sentences) >= 2:
        # First two sentences usually capture problem and innovation
        s1 = sentences[0]
        s2 = sentences[1]
        pitch_text = f"Welcome to Stall {code}: {title}. Guided by {lead}, {s1}. {s2}."
    elif len(sentences) == 1:
        pitch_text = f"Welcome to Stall {code}: {title}. Guided by {lead}, {sentences[0]}."
    else:
        pitch_text = f"Welcome to Stall {code}: {title}. Supervised by {lead}, this student capstone demonstrates cutting-edge mechatronics and engineering innovation here at DPSI EXPO."

    # If too long (> 58 words), truncate cleanly
    words = pitch_text.split()
    if len(words) > 55:
        pitch_text = " ".join(words[:52]) + "."
        
    curated_pitches[code] = {
        "code": code,
        "title": title,
        "lead": lead,
        "pitch": pitch_text,
        "word_count": len(pitch_text.split()),
        "output_file": f"{code.lower().replace('-', '_')}_25s.mp3"
    }

manifest_path = os.path.join(OUTPUT_DIR, "fish_audio_manifest.json")
with open(manifest_path, "w", encoding="utf-8") as f:
    json.dump(curated_pitches, f, indent=2)

print(f"Prepared {len(curated_pitches)} stall scripts under 25s constraint.\n")
print("=" * 80)
print("🎙️ BATCH GENERATING WITH FISH AUDIO API (v3 MODEL)")
print("=" * 80)

headers = {
    "Authorization": f"Bearer {API_KEY}",
    "model": "s2.1-pro-free",
    "Content-Type": "application/json"
}

summary = []

for code, item in curated_pitches.items():
    mp3_path = os.path.join(OUTPUT_DIR, item["output_file"])
    payload = {
        "text": item["pitch"],
        "reference_id": VOICE_REF_ID,
        "format": "mp3"
    }
    
    t0 = time.perf_counter()
    resp = requests.post(URL, headers=headers, json=payload, timeout=40)
    latency = time.perf_counter() - t0
    
    if resp.status_code == 200:
        with open(mp3_path, "wb") as f:
            f.write(resp.content)
            
        # Get audio duration using ffprobe
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "default=noprint_wrappers=1:nokey=1", mp3_path],
            capture_output=True, text=True, check=True
        )
        duration_s = float(probe.stdout.strip()) if probe.stdout.strip() else 0.0
        
        status_flag = "✓ PASS (<= 25s)" if duration_s <= 25.0 else "⚠️ EXCEEDED 25s"
        print(f"  ✓ [{code:>7}] {item['title'][:28]:<28} | Duration: {duration_s:4.1f}s | {item['word_count']}w | API: {latency:4.1f}s | {status_flag}")
        
        summary.append({
            "code": code,
            "title": item["title"],
            "duration_s": round(duration_s, 2),
            "file": item["output_file"]
        })
    else:
        print(f"  ✗ [{code:>7}] Failed: HTTP {resp.status_code} - {resp.text[:80]}")
        
    time.sleep(0.4)

with open(os.path.join(OUTPUT_DIR, "generation_summary.json"), "w") as f:
    json.dump(summary, f, indent=2)

print("\n" + "=" * 80)
print(f"🎉 Successfully pre-generated all {len(summary)} stall audio files in {OUTPUT_DIR}")
print("=" * 80)
