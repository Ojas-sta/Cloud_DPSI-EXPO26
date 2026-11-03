"""
Project Cloud: Inventory Service Node (ARGUS Logistics Steward)
Provides the QueryInventory ROS 2 service for speech and text lookup.
Generates authoritative, natural speech feedback for makerspace logistics.
Reference: PROJECT.md Section 7.3 & 8.3
"""

import time
from typing import Optional
from cloud_core.bus import CloudNode
from cloud_core.types import (
    QueryInventoryRequest,
    QueryInventoryResponse,
    InventoryItem
)
from .db_manager import InventoryDBManager
from .fuzzy_matcher import InventoryFuzzyMatcher

class InventoryServiceNode(CloudNode):
    """
    ROS 2 service provider for makerspace component search.
    """
    def __init__(self, node_name: str = "inventory_service_node"):
        super().__init__(node_name)
        self.db = InventoryDBManager()
        self.matcher = InventoryFuzzyMatcher()
        self._refresh_cache()

        # Service Server
        self.srv_query = self.create_service(
            QueryInventoryRequest, "/cloud/query_inventory", self._handle_query
        )
        self.logger.info("Inventory Service Node active on /cloud/query_inventory.")

    def _refresh_cache(self):
        items = self.db.get_all_items()
        self.matcher.set_items(items)
        self.logger.info(f"Loaded and indexed {len(items)} inventory items.")

    def _handle_query(self, request: QueryInventoryRequest) -> QueryInventoryResponse:
        t0 = time.perf_counter()
        query_text = request.spoken_query.strip()
        matches = self.matcher.match(query_text, top_k=1)
        latency_ms = (time.perf_counter() - t0) * 1000.0

        if not matches:
            speech = f"I could not locate any inventory matching '{query_text}'. Please check the part number or ask the workshop steward."
            self.logger.warn(f"Query '{query_text}' had no matches ({latency_ms:.2f}ms).")
            return QueryInventoryResponse(
                success=False,
                message="No matching inventory item found.",
                item=None,
                match_confidence=0.0,
                speech_response=speech
            )

        item, confidence = matches[0]
        self.logger.info(f"Query '{query_text}' matched '{item.canonical_name}' (conf: {confidence:.2f}, {latency_ms:.2f}ms).")

        # Construct vocal response
        speech_parts = [
            f"We have {item.stock_qty} {item.unit} of {item.canonical_name}.",
            f"Located in Cabinet {item.cabinet_id:02d}, {item.drawer_code}, Shelf Tier {item.shelf_tier}, referenced under Anchor Tag {item.anchor_tag_id}."
        ]

        if item.custody_bench is not None:
            speech_parts.append(f"Notice: This tool is currently signed out to Workbench {item.custody_bench}.")
        elif item.stock_qty <= item.reorder_threshold:
            speech_parts.append(f"Attention: Stock is low. Reorder threshold is {item.reorder_threshold} {item.unit}.")

        speech_text = " ".join(speech_parts)

        return QueryInventoryResponse(
            success=True,
            message=f"Located in Cabinet {item.cabinet_id:02d}, {item.drawer_code}",
            item=item,
            match_confidence=round(confidence, 3),
            speech_response=speech_text
        )
