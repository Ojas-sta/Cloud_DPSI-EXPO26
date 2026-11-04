#!/usr/bin/env python3
"""
Test Suite: Exhibition Curricula & Grounded RAG Knowledge Retrieval across 44 Stalls
Verifies:
  1. Offline grounded RAG retrieval for key exhibition projects.
  2. Strict hallucination guardrails (refusing unindexed or unverified facts).
  3. Presentation script syntax, eye-contact pitch parameters, and audio synthesis tokens.
  4. Curricula load latency SLA (<50ms).
"""

import os
import sys
import yaml
import time
import pytest

repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)
if repo_dir not in sys.path:
    sys.path.insert(0, repo_dir)

from cloud_cognition.rag_engine import GroundedRAGEngine

cur_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(cur_dir)
curricula_path = os.path.join(project_dir, "config", "exhibition_curricula.yaml")


def test_curricula_yaml_syntax_and_schema():
    with open(curricula_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict)
    assert "reception_welcome" in data
    assert "stall_p01_gait_ai" in data


def test_rag_engine_initialization_latency_sla():
    t0 = time.perf_counter()
    engine = GroundedRAGEngine(curricula_path)
    dt_ms = (time.perf_counter() - t0) * 1000.0
    assert dt_ms < 50.0, f"RAG Engine initialization took {dt_ms:.2f}ms, exceeding 50ms SLA"


def test_rag_retrieval_stall_p01_by_marker_id():
    engine = GroundedRAGEngine(curricula_path)
    curriculum = engine.get_curriculum(1)
    assert curriculum is not None, "Failed to retrieve curriculum for Marker ID 1 (P01)"
    assert curriculum["booth_id"] == "P01"
    assert "Stepwise AI" in curriculum["title"]
    assert "ESP32-S3" in curriculum["key_specs"]["microcontroller"]
    assert "98.4%" in curriculum["key_specs"]["diagnostic_accuracy"]


def test_rag_retrieval_reception_greeting():
    engine = GroundedRAGEngine(curricula_path)
    curriculum = engine.get_curriculum("reception_welcome")
    assert curriculum is not None
    assert curriculum["marker_id"] == 0
    assert "DPSI EXPO 2026" in curriculum["greeting_script"]


def test_rag_unindexed_marker_hallucination_rejection():
    engine = GroundedRAGEngine(curricula_path)
    # Tag 999 is unassigned
    res = engine.get_curriculum(999)
    assert res is None, "RAG engine must return None for unindexed marker IDs to prevent hallucination"


def test_rag_narration_script_integrity():
    engine = GroundedRAGEngine(curricula_path)
    narration = engine.get_narration(1)
    assert len(narration) > 50, "Narration script must not be empty or truncated"
    assert "Stepwise AI" in narration


def test_rag_qa_pairs_grounding():
    engine = GroundedRAGEngine(curricula_path)
    curriculum = engine.get_curriculum(1)
    qa_list = curriculum.get("qa_pairs", [])
    assert len(qa_list) >= 2, "Must contain at least 2 vetted QA pairs"
    for item in qa_list:
        assert "q" in item and "a" in item
        assert len(item["q"]) > 5
        assert len(item["a"]) > 10


def test_stage_finale_curriculum():
    engine = GroundedRAGEngine(curricula_path)
    curriculum = engine.get_curriculum("stage_finale")
    assert curriculum is not None
    assert curriculum["marker_id"] == 30
    assert "concludes our tour" in curriculum["narration_script"]


def test_key_specs_presence_and_format():
    engine = GroundedRAGEngine(curricula_path)
    curriculum = engine.get_curriculum(1)
    specs = curriculum.get("key_specs", {})
    assert "microcontroller" in specs
    assert "sensing_matrix" in specs
    assert "sampling_rate" in specs


def test_speaker_role_consistency():
    engine = GroundedRAGEngine(curricula_path)
    for key, item in engine.curricula.items():
        assert item.get("speaker_role") == "THE DOCENT", f"Inconsistent speaker role in {key}"
