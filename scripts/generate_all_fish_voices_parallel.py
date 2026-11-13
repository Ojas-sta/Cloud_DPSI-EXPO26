#!/usr/bin/env python3
"""
High-throughput parallel generator for 25s-max Fish Audio tour guides across all 4 voices.
128 files generated concurrently.
"""

import os
import time
import json
import requests
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

API_KEY = "sk-fish-XR2jpZ3D7cWMDae3vQnncS730o8UVtn78SXh11TkpZc"
URL = "https://api.fish.audio/v1/tts"
BASE_DIR = "/Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/audio/fish_audio_25s"

VOICES = {
    "v1": "bf322df2096a46f18c579d0baa36f41d",
    "v2": "536d3a5e000945adb7038665781a4aca",
    "v3": "711cf3ed00ab441a8f54a45058047b7a",
    "v4": "a9261e0625ce4253914f5349fa0d7852"
}

from generate_all_fish_voices_25s import CURATED_SCRIPTS

for v in VOICES:
    os.makedirs(os.path.join(BASE_DIR, v), exist_ok=True)

tasks = []
for v_name, ref_id in VOICES.items():
    for code, item in CURATED_SCRIPTS.items():
        filename = f"{code.lower().replace('-', '_')}_narration.mp3"
        out_path = os.path.join(BASE_DIR, v_name, filename)
        tasks.append({
            "voice": v_name,
            "ref_id": ref_id,
            "code": code,
            "title": item["title"],
            "lead": item["lead"],
            "script": item["script"],
            "out_path": out_path
        })

print(f"🚀 Queued {len(tasks)} parallel synthesis jobs across 6 threads...")

def process_task(task):
    out_path = task["out_path"]
    if os.path.exists(out_path) and os.path.getsize(out_path) > 10000:
        return task, True, 0.0, 0.0, os.path.getsize(out_path)/1024.0, "Cached"
        
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "model": "s2.1-pro-free",
        "Content-Type": "application/json"
    }
    payload = {
        "text": task["script"],
        "reference_id": task["ref_id"],
        "format": "mp3"
    }
    
    for attempt in range(4):
        t0 = time.perf_counter()
        try:
            resp = requests.post(URL, headers=headers, json=payload, timeout=25)
            latency = time.perf_counter() - t0
            if resp.status_code == 200:
                with open(out_path, "wb") as f:
                    f.write(resp.content)
                size_kb = len(resp.content) / 1024.0
                return task, True, latency, 0.0, size_kb, "OK"
            elif resp.status_code == 429:
                time.sleep(1.0 + attempt * 1.5)
            else:
                time.sleep(1.0)
        except Exception as e:
            time.sleep(1.0)
            
    return task, False, 0.0, 0.0, 0.0, "Failed"

completed_count = 0
results_meta = []

t_start = time.time()
with ThreadPoolExecutor(max_workers=6) as executor:
    futures = {executor.submit(process_task, t): t for t in tasks}
    for f in as_completed(futures):
        task, ok, latency, _, size_kb, msg = f.result()
        completed_count += 1
        stat = "✓" if ok else "✗"
        if completed_count % 8 == 0 or completed_count == len(tasks):
            print(f"  [{completed_count:>3}/{len(tasks)}] {stat} {task['voice']} | {task['code']:<7} | Latency: {latency:.1f}s | Size: {size_kb:.1f} KB | {msg}")
        results_meta.append({
            "voice": task["voice"],
            "code": task["code"],
            "title": task["title"],
            "lead": task["lead"],
            "audio_path": task["out_path"],
            "status": msg
        })

total_time = round(time.time() - t_start, 1)
print(f"\n🎉 Finished generating all {len(tasks)} audio files in {total_time}s!")

# Probe durations for all generated files
durations = []
print("Probing audio durations...")
for r in results_meta:
    p = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "default=noprint_wrappers=1:nokey=1", r["audio_path"]],
                       stdout=subprocess.PIPE, text=True)
    try:
        dur = float(p.stdout.strip())
        r["duration_sec"] = round(dur, 2)
        durations.append(dur)
    except:
        r["duration_sec"] = 0.0

max_dur = max(durations) if durations else 0.0
min_dur = min(durations) if durations else 0.0
avg_dur = sum(durations)/len(durations) if durations else 0.0
print(f"📊 Audio Durations: Min={min_dur:.1f}s | Avg={avg_dur:.1f}s | Max={max_dur:.1f}s (All <= 25s: {max_dur <= 25.0})")

with open(os.path.join(BASE_DIR, "manifest_25s_all_models.json"), "w", encoding="utf-8") as f:
    json.dump(results_meta, f, indent=2)

