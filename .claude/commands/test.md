# Slash Command: /test
**Description:** Run the full 148-test automated verification suite for Cloud-Argus-Docent.

## Execution Command
```bash
cd /Users/roopalisingh/cloud-argus-docent
/Users/roopalisingh/robostack/.pixi/envs/jazzy/bin/pytest tests/ -v
```

## Expected Outcome
- 148 tests passing across all 11 test modules.
- Verification covers: 44-stall terminal docking convergence, RAG curricula retrieval, 50% autonomy reactive local planning, emergency braking, teleop safety ramps, and arena geometry.
