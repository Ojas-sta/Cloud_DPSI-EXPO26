# Slash Command: /tour
**Description:** Run the autonomous VIP cohort tour demonstrating navigation down the concourse, docking at stalls, 180° presenter pivot, and audio narration.

## Execution Command
```bash
cd /Users/roopalisingh/cloud-argus-docent
/Users/roopalisingh/robostack/.pixi/envs/jazzy/bin/python demonstrate_cohort_tour_loop.py
```

## Tour Sequence
1. Start at South VIP Gate $(0.0\,\text{m}, 0.0\,\text{m})$.
2. Navigate East Concourse with $\ge 0.70\,\text{m}$ clearance to avoid booth corner clipping.
3. Dock at Zone 2 exhibits (`P01`, `P06`, `D01`) using apron-mounted ArUco visual servoing.
4. Perform 180° Presenter Pivot to face evaluators.
5. Deliver narration and return to South Gate.
