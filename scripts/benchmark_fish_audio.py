#!/usr/bin/env python3
import time
import os
import requests
import json

API_KEY = "sk-fish-XR2jpZ3D7cWMDae3vQnncS730o8UVtn78SXh11TkpZc"
URL = "https://api.fish.audio/v1/tts"
OUTPUT_DIR = "/Users/roopalisingh/.gemini/antigravity/scratch/cloud-argus-docent/audio/fish_benchmark"
os.makedirs(OUTPUT_DIR, exist_ok=True)

VOICES = {
    "v1": "bf322df2096a46f18c579d0baa36f41d",
    "v2": "536d3a5e000945adb7038665781a4aca",
    "v3": "711cf3ed00ab441a8f54a45058047b7a",
    "v4": "a9261e0625ce4253914f5349fa0d7852"
}

TEST_TEXTS = {
    "short_cue": "Welcome to Stall P-01. I am Cloud, your quadruped tour docent.",
    "medium_pitch": "Welcome to Stall P-01: Stepwise AI. Stepwise AI is a biomechanical gait analysis platform deploying six-axis IMUs to analyze ankle pronation with clinical precision.",
    "full_synopsis": "Welcome to Stall P-01: Stepwise AI. Guided by Divya, Stepwise AI is an innovative rehabilitation technology utilizing wearable inertial measurement sensors combined with computer vision algorithms to track joint kinematics, detect gait deviations in real time, and quantify recovery metrics for clinicians."
}

results = []

print("=" * 80)
print(f"🚀 BENCHMARKING FISH AUDIO API (4 VOICES × 3 PROMPT LENGTHS)")
print("=" * 80)

for v_name, ref_id in VOICES.items():
    for t_name, text in TEST_TEXTS.items():
        char_count = len(text)
        word_count = len(text.split())
        output_file = os.path.join(OUTPUT_DIR, f"{v_name}_{t_name}.mp3")
        
        headers = {
            "Authorization": f"Bearer {API_KEY}",
            "model": "s2.1-pro-free",
            "Content-Type": "application/json"
        }
        payload = {
            "text": text,
            "reference_id": ref_id,
            "format": "mp3"
        }
        
        start_time = time.perf_counter()
        try:
            resp = requests.post(URL, headers=headers, json=payload, timeout=30)
            latency = time.perf_counter() - start_time
            status = resp.status_code
            if status == 200:
                with open(output_file, "wb") as f:
                    f.write(resp.content)
                file_size_kb = len(resp.content) / 1024.0
                err_msg = ""
            else:
                file_size_kb = 0.0
                err_msg = resp.text[:120]
        except Exception as e:
            latency = time.perf_counter() - start_time
            status = 0
            file_size_kb = 0.0
            err_msg = str(e)
            
        res_entry = {
            "voice": v_name,
            "ref_id": ref_id,
            "test_type": t_name,
            "char_count": char_count,
            "word_count": word_count,
            "latency_sec": round(latency, 2),
            "status": status,
            "file_size_kb": round(file_size_kb, 1),
            "audio_path": output_file if status == 200 else None,
            "error": err_msg
        }
        results.append(res_entry)
        
        status_mark = "✓" if status == 200 else "✗"
        print(f"  {status_mark} [{v_name}] {t_name:<15} | {word_count:>2} words ({char_count:>3} chars) | Latency: {latency:.2f}s | Size: {file_size_kb:.1f} KB {err_msg}")
        # small pause to be polite
        time.sleep(0.3)

with open(os.path.join(OUTPUT_DIR, "benchmark_results.json"), "w") as f:
    json.dump(results, f, indent=2)

print("\n" + "=" * 80)
print(f"🎉 Benchmark complete. Saved audio & metrics to {OUTPUT_DIR}")
print("=" * 80)
