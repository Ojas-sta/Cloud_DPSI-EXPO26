"""
Project Cloud: Makerspace Inventory Database Manager (SQLite)
Manages tools, components, fasteners, and storage topology.
Populated from config/inventory_seed.sql.
Reference: PROJECT.md Section 7.3 & Appendix B
"""

import os
import sqlite3
import json
from typing import List, Optional, Dict, Any
from cloud_core.types import InventoryItem

class InventoryDBManager:
    """
    SQLite inventory connection manager and query engine.
    """
    def __init__(self, db_path: Optional[str] = None, seed_sql_path: Optional[str] = None):
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        if db_path is None:
            db_path = os.path.join(base_dir, "config", "cloud_inventory.db")
        if seed_sql_path is None:
            seed_sql_path = os.path.join(base_dir, "config", "inventory_seed.sql")

        self.db_path = db_path
        self.seed_sql_path = seed_sql_path
        self._init_database()

    def _get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_database(self):
        """Initializes tables and populates with seed data if fresh."""
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT count(*) FROM sqlite_master WHERE type='table' AND name='inventory_items';")
        count = cursor.fetchone()[0]

        if count == 0:
            if not os.path.exists(self.seed_sql_path):
                raise FileNotFoundError(f"Inventory seed script missing: {self.seed_sql_path}")
            with open(self.seed_sql_path, "r", encoding="utf-8") as f:
                seed_sql = f.read()
            cursor.executescript(seed_sql)
            conn.commit()
        conn.close()

    def get_all_items(self) -> List[InventoryItem]:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            SELECT item_id, sku_code, canonical_name, aliases_json, stock_qty,
                   unit, cabinet_id, drawer_code, shelf_tier, anchor_tag_id,
                   reorder_threshold, custody_bench
            FROM inventory_items
        """)
        rows = cursor.fetchall()
        items = [self._row_to_item(r) for r in rows]
        conn.close()
        return items

    def get_item_by_id(self, item_id: int) -> Optional[InventoryItem]:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            SELECT item_id, sku_code, canonical_name, aliases_json, stock_qty,
                   unit, cabinet_id, drawer_code, shelf_tier, anchor_tag_id,
                   reorder_threshold, custody_bench
            FROM inventory_items
            WHERE item_id = ?
        """, (item_id,))
        row = cursor.fetchone()
        conn.close()
        return self._row_to_item(row) if row else None

    def get_item_by_sku(self, sku_code: str) -> Optional[InventoryItem]:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("""
            SELECT item_id, sku_code, canonical_name, aliases_json, stock_qty,
                   unit, cabinet_id, drawer_code, shelf_tier, anchor_tag_id,
                   reorder_threshold, custody_bench
            FROM inventory_items
            WHERE sku_code = ?
        """, (sku_code,))
        row = cursor.fetchone()
        conn.close()
        return self._row_to_item(row) if row else None

    def update_stock(self, item_id: int, delta: int) -> Optional[int]:
        """Adjusts stock quantity by delta; prevents negative stock."""
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT stock_qty FROM inventory_items WHERE item_id = ?", (item_id,))
        row = cursor.fetchone()
        if not row:
            conn.close()
            return None

        new_qty = max(0, row["stock_qty"] + delta)
        cursor.execute("UPDATE inventory_items SET stock_qty = ? WHERE item_id = ?", (new_qty, item_id))
        conn.commit()
        conn.close()
        return new_qty

    def assign_custody(self, item_id: int, bench_id: Optional[int]) -> bool:
        conn = self._get_connection()
        cursor = conn.cursor()
        cursor.execute("UPDATE inventory_items SET custody_bench = ? WHERE item_id = ?", (bench_id, item_id))
        conn.commit()
        rows_affected = cursor.rowcount
        conn.close()
        return rows_affected > 0

    @staticmethod
    def _row_to_item(row: sqlite3.Row) -> InventoryItem:
        return InventoryItem(
            item_id=row["item_id"],
            sku_code=row["sku_code"],
            canonical_name=row["canonical_name"],
            aliases_json=row["aliases_json"],
            stock_qty=row["stock_qty"],
            unit=row["unit"],
            cabinet_id=row["cabinet_id"],
            drawer_code=row["drawer_code"],
            shelf_tier=row["shelf_tier"],
            anchor_tag_id=row["anchor_tag_id"],
            reorder_threshold=row["reorder_threshold"],
            custody_bench=row["custody_bench"]
        )
