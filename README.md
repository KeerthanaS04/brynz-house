# Property capture → dimensioned floor plan

One command turns a phone capture into a dimensioned floor plan (`floorplan.png`) and machine-readable
geometry (`property.json`). Engineering contract: `CLAUDE_updated.md`. Requirement status:
`docs/requirements/traceability.csv`. Evidence per feature: `reports/*/README.md`.

| Input tier | Status | What you get |
|---|---|---|
| **LiDAR** (Pro iPhone, capture app export) | implemented | rooms, walls, openings, ceiling heights, shared walls, drift on/off ablation |
| **Video** (any iPhone, Camera app) | partial | plan of the largest unbroken stretch of the walk only; scale from a pretrained depth network (`reports/video_depth`) |
| **Photo** (2–8 stills per room) | not implemented | refused with a message (no photo data supplied) |

Every measurement is `uncalibrated`: the supplied captures have no reference measurements (assumptions.md B-21).

## Setup on a fresh machine (Windows PowerShell)

Tested with Python 3.13 on Windows 11. The LiDAR tier needs no GPU. The video tier's depth network runs on an NVIDIA GPU if there is one (tested: 4 GB laptop GPU), otherwise on the CPU, more slowly.

```powershell
git clone <repository URL>; cd brynz-house
python -m venv .venv
.venv\Scripts\Activate.ps1
# NVIDIA GPU only: CUDA build of PyTorch first (skip on CPU-only machines)
pip install torch --index-url https://download.pytorch.org/whl/cu126
pip install -e ".[dev]"
python scripts/fetch_weights.py        # depth network for the video tier, ~100 MB, into weights/
python -m pytest -q                    # unit tests
```

Place the capture ZIPs in the repository root. They are not committed; ZIPs are extracted under `data/raw/` on first use.

## One command per capture

```powershell
python -m property_capture run --input single_room.zip            # LiDAR capture (tier detected)
python -m property_capture run --input IMG_0042.MOV               # video walkthrough
python -m property_capture run --input single_room.zip --tier video   # video tier on a LiDAR capture's rgb.mp4 only
```

- **Tier detection:** `--tier auto` (the default) picks the tier from the input's layout:
  - ZIP or folder with `odometry.csv` → LiDAR
  - one video file → video
  - folder of images → photo, refused with a message
- **Output:** a fresh folder, `outputs/run_<UTC time>/` by default. Runs never overwrite.
- **Config:** override any value with `--set KEY=VALUE`, e.g. `--set video.target_fps=3` for long videos.

Measured timings on this laptop (RTX 500 Ada, 4 GB):

| Capture (video length) | LiDAR run | Video run |
|---|---|---|
| single_room (~28 s) | 1.1 min | 4.2 min |
| floor_only (~84 s) | 2.6–4.9 min | 8.1 min (`--set video.target_fps=3`) |
| with_ceiling (~165 s) | 3.8–8.4 min | 14.5 min (`--set video.target_fps=3`) |

The full reproduction takes about 53 minutes. The video tier's SfM step is not deterministic: a repeat run can split the walk differently (`reports/video_depth`).

### Run folder

| File | Content |
|---|---|
| `floorplan.png` | dimensioned plan; orange `*` = inferred wall |
| `property.json` | rooms, walls, openings, measurements (each with uncertainty status and quality flags), warnings, unobservable quantities; `input_tier` says which tier |
| `per_measurement.csv` | one row per measurement |
| `metrics.json` | quality checks per stage (plane fits, wall support, half-split consistency, drift, video trajectory) |
| `gate_results.json` | status of every gate in `configs/evaluation/gates.yaml` |
| `run_info.json` | command, full config and hash, git commit, package versions, input hashes, timings |
| `intermediates/`, `run.log` | stage outputs and log |
| `video_sfm/` | video tier only: selected frames and the SfM reconstruction (step 1) |

## Other commands

```powershell
python -m property_capture audit                                   # audit every ZIP -> reports/data_audit/
python -m property_capture evaluate --references data/references.csv --run scan-01=outputs/run_x   # vs laser references
python -m property_capture video-oracle --video-run outputs/run_v --capture data/raw/<capture> --lidar-run outputs/run_l  # evaluation only
python scripts/diff_runs.py outputs/run_a outputs/run_b             # regression check: same measurements?
```

## Reproduce the evidence

```powershell
python scripts/reproduce.py                    # audit + both tiers on all three captures
python scripts/reproduce.py --tiers lidar      # LiDAR only
```

The script runs every step listed in `configs/reproduce.yaml` into a fresh `outputs/repro_<UTC time>/`, with one log per step. `REPRODUCTION.md` there records:
- the environment and GPU;
- SHA-256 hashes of the input ZIPs and the model weights;
- every command, with its exit code and time;
- the headline numbers, read back from the outputs.

Report folders under `reports/` were made at earlier commits; each run's `run_info.json` names its commit.

## Documents

| Document | For |
|---|---|
| `docs/technical_report.md` | technical report (draft): architecture, tiers, calibration, drift, error budget, fix loop, failure modes |
| `docs/capture_protocol.md` | one-page capture instructions for a non-engineer (PDF Route 2) |
| `docs/device_matrix.md` | which tier runs on which iPhone, and what accuracy each honestly delivers |
| `docs/benchmark_protocol.md` | benchmark set and laser reference measurements (template in `docs/templates/`) |
| `docs/requirements/assumptions.md` | every assumption, observation and open question |
| `docs/requirements/traceability.csv` | every PDF requirement → implementation → evidence → status |
