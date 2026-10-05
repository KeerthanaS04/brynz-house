# Property capture → dimensioned floor plan

Engineering contract: `CLAUDE_updated.md`. Requirement status: `docs/requirements/traceability.csv`.
Current state: Phase 0 (requirements, data audit, RGB-D baseline). See `reports/baseline/phase0_baseline.md`.

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
