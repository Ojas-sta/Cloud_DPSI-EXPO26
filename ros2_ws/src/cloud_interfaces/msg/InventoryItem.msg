# InventoryItem.msg: Structured makerspace hardware item
# Reference: PROJECT.md Section 7.3 & Appendix B

int32 item_id
string sku_code
string canonical_name
string aliases_json
int32 stock_qty
string unit
int32 cabinet_id
string drawer_code
int32 shelf_tier
int32 anchor_tag_id
int32 reorder_threshold
int32 custody_bench             # -1 if in storage
