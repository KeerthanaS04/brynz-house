# Property capture → dimensioned floor plan

Engineering contract: `CLAUDE_updated.md`. Requirement status: `docs/requirements/traceability.csv`.
Current state: LiDAR tier implemented (rooms, walls, openings, ceilings, drift accountability); photo and video
tiers not yet implemented. Results per feature: `reports/*/README.md`.

| Document | For |
|---|---|
| `docs/capture_protocol.md` | one-page capture instructions for a non-engineer (PDF Route 2) |
| `docs/device_matrix.md` | which tier runs on which iPhone, and what accuracy each honestly delivers |
| `docs/benchmark_protocol.md` | benchmark set and laser reference measurements (template in `docs/templates/`) |
| `docs/requirements/assumptions.md` | every assumption, observation and open question |

## Setup (Windows PowerShell)

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -e ".[dev]"
```

Place the capture ZIPs in the repository root (they are not committed).

## Commands

```powershell
# Audit every ZIP in the repo root -> reports/data_audit/, docs/data_dictionary.md
python -m property_capture audit

# RGB-D baseline on one capture (ZIP or extracted folder) -> fresh outputs/run_<id>/
python -m property_capture run --input single_room.zip
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/run_my_test

# Unit tests
python -m pytest -q
```

## Run directory contents

| File | Content |
|---|---|
| `property.json` | rooms, walls, measurements (all `uncalibrated`), warnings, unobservable quantities |
| `metrics.json` | convention checks, plane fits, wall support, half-split consistency |
| `per_measurement.csv` | one row per measurement |
| `gate_results.json` | status of every gate in `configs/evaluation/gates.yaml` |
| `floorplan.png` | dimensioned plan; orange `*` = inferred wall |
| `run_info.json` | command, config, config hash, git commit, package versions, input hashes, timings |
| `intermediates/` | pose convention table, gravity estimate, fused point cloud |
| `run.log` | stage log |
