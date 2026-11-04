"""
Project Cloud: Inventory & Logistics Test Suite
Validates SQLite catalog integrity, phonetic/Levenshtein matching, and <15ms latency.
"""

import sys
import os
import time
import pytest

# Add ros2_ws/src to path
repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src_dir = os.path.join(repo_dir, "ros2_ws", "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from cloud_inventory import InventoryDBManager, InventoryFuzzyMatcher, soundex, levenshtein_ratio
from cloud_inventory.inventory_service_node import InventoryServiceNode
from cloud_core.types import QueryInventoryRequest

def test_inventory_seed_count():
    db = InventoryDBManager()
    items = db.get_all_items()
    assert len(items) == 50, f"Expected 50 seed items, found {len(items)}"
    esp32 = db.get_item_by_sku("EL-0003")
    assert esp32 is not None
    assert "ESP32" in esp32.canonical_name
    assert esp32.cabinet_id == 1

def test_soundex_phonetics():
    # Phonetic equivalents mapped by American Soundex
    assert soundex("resistor") == soundex("rezistor")
    assert soundex("smith") == soundex("smyth")
    assert soundex("esp") != soundex("motor")

def test_levenshtein_ratio():
    assert levenshtein_ratio("esp32", "esp32") == 1.0
    assert levenshtein_ratio("esp32", "esp33") == 0.8
    assert levenshtein_ratio("wire", "") == 0.0

def test_fuzzy_matching_accuracy():
    db = InventoryDBManager()
    matcher = InventoryFuzzyMatcher(db.get_all_items())

    queries = [
        ("esp32 microcontroller", "ESP32-WROOM-32D"),
        ("portable soldering iron", "Pinecil Smart Portable Soldering Iron"),
        ("digital vernier caliper", "Digital Calipers"),
        ("bme 280 atmospheric sensor", "BME280 Temperature"),
        ("3d print black pla filament", "PolyLite PLA+")
    ]

    for q, expected in queries:
        matches = matcher.match(q, top_k=1)
        assert len(matches) > 0, f"No match found for '{q}'"
        item, conf = matches[0]
        assert expected.lower() in item.canonical_name.lower(), f"Matched '{item.canonical_name}' for '{q}'"
        assert conf >= 0.50

def test_query_service_latency_benchmark():
    node = InventoryServiceNode()
    # Warm up JIT/regex compilation
    node._handle_query(QueryInventoryRequest(spoken_query="warm up query esp32"))

    benchmark_queries = [
        "Where can I find the esp32?",
        "Do we have wire strippers?",
        "Check stock for towerpro micro servos",
        "Where is the hakko soldering station?",
        "Find me a raspberry pi pico"
    ]

    for q in benchmark_queries:
        t0 = time.perf_counter()
        req = QueryInventoryRequest(spoken_query=q)
        resp = node._handle_query(req)
        latency_ms = (time.perf_counter() - t0) * 1000.0

        assert resp.success, f"Query failed for: {q}"
        assert latency_ms < 50.0, f"Query latency exceeded 50ms SLA: {latency_ms:.2f}ms"
        assert len(resp.speech_response) > 20

def test_stock_mutation_and_custody():
    db = InventoryDBManager()
    item = db.get_item_by_sku("EL-0003")
    assert item is not None
    initial_stock = item.stock_qty

    # Decrement stock
    new_stock = db.update_stock(item.item_id, -2)
    assert new_stock == initial_stock - 2

    # Assign custody bench
    assigned = db.assign_custody(item.item_id, 3)
    assert assigned
    updated_item = db.get_item_by_id(item.item_id)
    assert updated_item.custody_bench == 3

    # Reset
    db.update_stock(item.item_id, 2)
    db.assign_custody(item.item_id, None)
