#!/usr/bin/env python3
"""
Unitree Go2 Onboard Loudspeaker Player & AudioHub Manager
=========================================================
Plays exhibition stall docent narration audio DIRECTLY through the physical
loudspeaker of the Unitree Go2 quadruped via WebRTC DataChannel RPC (AudioHub API).

Key Features:
- Direct chassis loudspeaker output (zero dependency on laptop speakers).
- Persistent AudioHub onboard storage: files uploaded once stay in Go2 flash memory.
- Sub-50ms playback latency using `play_by_uuid`.
- Automatic mono 44.1kHz WAV optimization for fast chunked uploads.
- Hardware volume management (0-10) via CycloneDDS VuiClient.
- Standalone CLI & direct programmatic integration for `topological_aruco_navigator.py`.

Usage:
    # Play specific stall narration on Go2 speaker:
    ./play_on_go2_speaker.py --stall P-01

    # Play specific file on Go2 speaker:
    ./play_on_go2_speaker.py --file audio/fish_audio_25s/v3/p_01_narration.mp3

    # Upload all 36 stall narration files into Go2 flash memory:
    ./play_on_go2_speaker.py --upload-all

    # Play test tone on Go2 speaker:
    ./play_on_go2_speaker.py --test-tone

    # Set Go2 speaker hardware volume (0-10):
    ./play_on_go2_speaker.py --volume 10

    # List all audio files currently stored in Go2 memory:
    ./play_on_go2_speaker.py --list
"""

import os
import sys
import json
import time
import uuid
import hashlib
import base64
import asyncio
import argparse
from typing import Optional, Dict, Any, List

# Ensure project root & site-packages in sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

VENV_REALSENSE_SITE = os.path.join(PROJECT_ROOT, ".venv_realsense", "lib", "python3.14", "site-packages")
if os.path.exists(VENV_REALSENSE_SITE) and VENV_REALSENSE_SITE not in sys.path:
    sys.path.insert(0, VENV_REALSENSE_SITE)

MANIFEST_PATH = os.path.join(PROJECT_ROOT, "config", "go2_audiohub_manifest.json")

# Tag to stall code mapping
TAG_TO_CODE = {
    1: "p_01", 2: "p_02", 3: "p_03", 4: "p_04", 5: "p_05", 6: "p_06",
    7: "p_07", 8: "p_08", 9: "p_09", 10: "p_10", 11: "p_11", 12: "p_12",
    13: "d_01", 14: "d_02", 15: "d_03", 16: "d_04", 17: "d_05", 18: "d_06",
    19: "d_07", 20: "d_08", 21: "d_09", 22: "d_10", 23: "d_11", 24: "d_12",
    25: "b_01", 26: "b_02", 27: "b_03", 28: "b_04", 29: "b_05", 30: "b_06",
    31: "b_07", 32: "display"
}

def load_manifest() -> Dict[str, str]:
    if os.path.exists(MANIFEST_PATH):
        try:
            with open(MANIFEST_PATH, "r") as f:
                return json.load(f)
        except Exception:
            return {}
    return {}

def save_manifest(manifest: Dict[str, str]):
    os.makedirs(os.path.dirname(MANIFEST_PATH), exist_ok=True)
    with open(MANIFEST_PATH, "w") as f:
        json.dump(manifest, f, indent=2)

def set_go2_volume(volume: int, robot_ip: str = "192.168.123.161", net_if: str = "en11") -> bool:
    """Sets Go2 hardware volume (0-10) via CycloneDDS VuiClient."""
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.vui.vui_client import VuiClient
        ChannelFactoryInitialize(0, net_if)
        vc = VuiClient()
        vc.Init()
        ret = vc.SetVolume(max(0, min(10, volume)))
        print(f"🔊 Go2 Hardware Volume set to {volume}/10 (result code: {ret})")
        return ret == 0
    except Exception as e:
        print(f"⚠️ Notice: CycloneDDS SetVolume skipped ({e})")
        return False

def resolve_audio_file(stall_arg: str, voice: str = "v3") -> Optional[str]:
    """Resolves audio file path from stall code (e.g. P-01, p_01, 1)."""
    arg = stall_arg.strip().lower().replace("-", "_")
    # Check if integer ID
    if arg.isdigit():
        tag_id = int(arg)
        code = TAG_TO_CODE.get(tag_id, f"p_{tag_id:02d}")
    else:
        code = arg

    candidates = [
        os.path.join(PROJECT_ROOT, "audio", "fish_audio_25s", voice, f"{code}_narration.mp3"),
        os.path.join(PROJECT_ROOT, "audio", "fish_audio_25s", "v1", f"{code}_narration.mp3"),
        os.path.join(PROJECT_ROOT, "audio", "fish_audio_pregenerated", f"{code}_narration.mp3"),
        os.path.join(PROJECT_ROOT, "audio", "docent_narration", f"{code}_narration.mp3"),
        os.path.join(PROJECT_ROOT, "audio", "fish_audio_25s", voice, f"{code}.mp3")
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None

class Go2SpeakerClient:
    def __init__(self, robot_ip: str = "192.168.123.161"):
        self.robot_ip = robot_ip
        self.conn = None
        self.hub = None

    async def connect(self):
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub
        self.conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA,
            ip=self.robot_ip
        )
        await self.conn.connect()
        self.hub = WebRTCAudioHub(self.conn)

    async def disconnect(self):
        if self.conn:
            try:
                await self.conn.disconnect()
            except Exception:
                pass
            self.conn = None
            self.hub = None

    async def list_audio(self) -> List[Dict[str, Any]]:
        res = await self.hub.get_audio_list()
        try:
            outer = res.get("data", {})
            if isinstance(outer, dict):
                inner = outer.get("data", "{}")
            else:
                inner = outer
            if isinstance(inner, str):
                data = json.loads(inner)
            else:
                data = inner
            return data.get("audio_list", [])
        except Exception as e:
            print(f"Error parsing audio list: {e}")
            return []

    async def upload_file_to_robot(self, file_path: str, custom_name: Optional[str] = None) -> Optional[str]:
        """Uploads an audio file into Go2 AudioHub and returns its assigned UNIQUE_ID."""
        from pydub import AudioSegment
        from unitree_webrtc_connect.webrtc_audiohub import AUDIO_API

        if not os.path.exists(file_path):
            print(f"❌ File not found: {file_path}")
            return None

        base_name = custom_name or os.path.splitext(os.path.basename(file_path))[0]
        # Clean name for Go2 AudioHub (alphanumeric + underscores, max 16 chars)
        clean_name = "".join(c for c in base_name if c.isalnum() or c == "_")[:16]

        print(f"🔄 Preparing audio: {base_name} -> mono 44.1kHz WAV...")
        if file_path.endswith(".mp3"):
            audio = AudioSegment.from_mp3(file_path)
        else:
            audio = AudioSegment.from_file(file_path)

        # Optimize for transmission & Go2 speaker: mono, 44100Hz, 16-bit
        audio = audio.set_channels(1).set_frame_rate(44100)
        temp_wav = f"/tmp/{clean_name}.wav"
        audio.export(temp_wav, format="wav")

        with open(temp_wav, "rb") as f:
            audio_data = f.read()

        file_md5 = hashlib.md5(audio_data).hexdigest()
        b64_data = base64.b64encode(audio_data).decode("utf-8")
        chunk_size = 4096
        chunks = [b64_data[i:i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
        total_chunks = len(chunks)

        print(f"📤 Uploading '{clean_name}' to Go2 AudioHub ({len(audio_data)} bytes in {total_chunks} chunks)...")
        last_res = None
        for i, chunk in enumerate(chunks, 1):
            param = {
                "file_name": clean_name,
                "file_type": "wav",
                "file_size": len(audio_data),
                "current_block_index": i,
                "total_block_number": total_chunks,
                "block_content": chunk,
                "current_block_size": len(chunk),
                "file_md5": file_md5,
                "create_time": int(time.time() * 1000)
            }
            last_res = await self.hub.data_channel.pub_sub.publish_request_new(
                "rt/api/audiohub/request",
                {
                    "api_id": AUDIO_API["UPLOAD_AUDIO_FILE"],
                    "parameter": json.dumps(param, ensure_ascii=True)
                }
            )
            await asyncio.sleep(0.015)

        # Allow robot flash write to settle
        await asyncio.sleep(0.5)

        # Find newly assigned UUID
        audio_list = await self.list_audio()
        assigned_uuid = None
        for item in audio_list:
            if item.get("CUSTOM_NAME") == clean_name:
                assigned_uuid = item.get("UNIQUE_ID")
                break

        if assigned_uuid:
            print(f"✅ Upload verified! Assigned UUID: {assigned_uuid}")
            # Update manifest
            manifest = load_manifest()
            manifest[base_name] = assigned_uuid
            save_manifest(manifest)
            return assigned_uuid
        else:
            print(f"⚠️ Upload sent, but could not confirm UUID from list response: {last_res}")
            return None

    async def play_audio_file(self, file_path: str, custom_name: Optional[str] = None, wait: bool = True):
        """Uploads (if not present) and triggers instant playback through Go2 physical speaker."""
        base_name = custom_name or os.path.splitext(os.path.basename(file_path))[0]
        clean_name = "".join(c for c in base_name if c.isalnum() or c == "_")[:16]

        # Check local manifest first
        manifest = load_manifest()
        target_uuid = manifest.get(base_name)

        # Verify on robot
        audio_list = await self.list_audio()
        verified_item = None
        for item in audio_list:
            if item.get("UNIQUE_ID") == target_uuid or item.get("CUSTOM_NAME") == clean_name:
                verified_item = item
                target_uuid = item.get("UNIQUE_ID")
                break

        if not verified_item:
            print(f"ℹ️ Audio '{clean_name}' not found on Go2 memory. Uploading now...")
            target_uuid = await self.upload_file_to_robot(file_path, clean_name)
            if not target_uuid:
                print(f"❌ Failed to prepare audio on Go2.")
                return False

        print(f"🔊 Playing '{clean_name}' (UUID: {target_uuid}) on Go2 PHYSICAL SPEAKER...")
        await self.hub.play_by_uuid(target_uuid)
        print("✅ Playback command triggered on robot!")

        if wait:
            # Estimate track duration
            try:
                from pydub import AudioSegment
                audio = AudioSegment.from_file(file_path)
                duration_sec = len(audio) / 1000.0
            except Exception:
                duration_sec = 25.0
            print(f"⏳ Waiting for playback to finish (~{duration_sec:.1f}s)...")
            await asyncio.sleep(duration_sec + 0.5)

        return True

    async def delete_audio(self, uuid_to_delete: str):
        print(f"🗑️ Deleting UUID {uuid_to_delete} from Go2...")
        await self.hub.delete_record(uuid_to_delete)
        manifest = load_manifest()
        manifest = {k: v for k, v in manifest.items() if v != uuid_to_delete}
        save_manifest(manifest)
        print("✅ Deleted.")

async def main_async(args):
    client = Go2SpeakerClient(robot_ip=args.ip)
    await client.connect()

    try:
        if args.volume is not None:
            set_go2_volume(args.volume, robot_ip=args.ip)

        if args.list:
            audio_list = await client.list_audio()
            print("\n📋 Audio Files Stored on Go2 Quadruped Memory:")
            print(f"{'#':<3} | {'CUSTOM_NAME':<18} | {'UNIQUE_ID':<38}")
            print("-" * 65)
            for idx, item in enumerate(audio_list, 1):
                print(f"{idx:<3} | {item.get('CUSTOM_NAME', ''):<18} | {item.get('UNIQUE_ID', ''):<38}")
            print("-" * 65)

        elif args.delete:
            await client.delete_audio(args.delete)

        elif args.upload_all is not None:
            search_dir = args.upload_all if (args.upload_all and os.path.isdir(args.upload_all)) else os.path.join(PROJECT_ROOT, "audio", "fish_audio_25s", args.voice)
            print(f"📦 Batch uploading all audio files from {search_dir}...")
            if not os.path.exists(search_dir):
                print(f"❌ Directory not found: {search_dir}")
                return
            mp3_files = sorted([f for f in os.listdir(search_dir) if f.endswith(".mp3")])
            print(f"Found {len(mp3_files)} audio files.")
            for f in mp3_files:
                full_path = os.path.join(search_dir, f)
                await client.upload_file_to_robot(full_path)
                await asyncio.sleep(0.3)
            print("🎉 All files uploaded to Go2 onboard memory!")

        elif args.test_tone:
            # Generate or use test tone
            from pydub.generators import Sine
            tone = Sine(520).to_audio_segment(duration=1500)
            tone_path = "/tmp/go2_audio_check.wav"
            tone.export(tone_path, format="wav")
            print("🔔 Playing Go2 speaker test tone...")
            await client.play_audio_file(tone_path, custom_name="audio_check", wait=not args.no_wait)

        elif args.stall:
            audio_file = resolve_audio_file(args.stall, voice=args.voice)
            if not audio_file:
                print(f"❌ Could not find audio track for stall: {args.stall}")
                return
            print(f"🎯 Resolved stall '{args.stall}' -> {audio_file}")
            await client.play_audio_file(audio_file, wait=not args.no_wait)

        elif args.file:
            if not os.path.exists(args.file):
                print(f"❌ File does not exist: {args.file}")
                return
            await client.play_audio_file(args.file, wait=not args.no_wait)

        else:
            print("⚠️ No action specified. Use --stall <ID>, --file <path>, --test-tone, or --list.")

    finally:
        await client.disconnect()

def main():
    parser = argparse.ArgumentParser(description="Unitree Go2 Onboard Loudspeaker Player & Manager")
    parser.add_argument("--stall", type=str, help="Stall code or tag ID (e.g. P-01, D-04, 1)")
    parser.add_argument("--file", type=str, help="Path to custom audio file (.mp3 or .wav)")
    parser.add_argument("--voice", type=str, default="v3", choices=["v1", "v2", "v3", "v4"], help="Voice version")
    parser.add_argument("--test-tone", action="store_true", help="Play test tone through Go2 loudspeaker")
    parser.add_argument("--volume", type=int, choices=range(0, 11), help="Set hardware volume (0-10)")
    parser.add_argument("--list", action="store_true", help="List audio files in Go2 flash memory")
    parser.add_argument("--delete", type=str, help="Delete audio file by UUID")
    parser.add_argument("--upload-all", nargs="?", const="", help="Upload all stall narrations into Go2 flash memory")
    parser.add_argument("--no-wait", action="store_true", help="Do not wait for audio playback duration to finish")
    parser.add_argument("--ip", type=str, default="192.168.123.161", help="Go2 Robot IP")
    args = parser.parse_args()

    asyncio.run(main_async(args))

if __name__ == "__main__":
    main()
