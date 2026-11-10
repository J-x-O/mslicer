# Phase 1 report: evaluation engine behind the current UI

Branch: `redesign/phase-1-engine` (from `redesign/phase-0-recon`).

## What changed

- `mslicer/src/engine/`: `Slot` (cache + key, one running job, latest
  wins, stale results discarded), `CancelToken`, `Debounce`, `InputHasher`
  (FNV-1a), `Status` (computing/waiting over the project), `LayerPreview`
  (on-demand single-layer slicing).
- Hollowing: cavity and drilling are slots; the 300 ms settle is a
  `Debounce`. Support mesh build: a slot keyed by the support sets'
  identity, raft, resolution and frame (the `stale` flag is gone).
- Support generation takes a cancel token (stops between placing and
  routing and per support); starting a new generation, clearing or undoing
  cancels it.
- Slicer: `slice_raster_until` (per-layer cancellation) and
  `slice_raster_layer` (one layer); `run_slicer` returns `None` when
  cancelled. A new Slice cancels a running one.
- `core/slice/key.rs`: per-model and per-print slice keys. Moving a model
  only changes its own key; colour and hidden models don't count.
- `core/slice/estimate.rs`: layer count, rough print time, resin volume,
  out-of-bounds count without slicing.
- Workspace > Stats > Evaluation shows the engine status and whether the
  last slice is outdated. Otherwise the UI is unchanged.

## What's tested

- Regression harness: unchanged (all 6 fixtures pass).
- `engine::slot` unit tests: work only on request, once; newer request
  cancels and discards; going back keeps the job; last result shown
  meanwhile; withdraw; clones.
- `engine::tests`: single layer == full slice layer (4 fixtures, incl.
  supersampling); a superseded full slice stops early (< half the layers)
  and returns nothing; a stale layer result is discarded; editing one
  model's wall leaves the other model's cavity, drilling and slice key
  alone; moving a model keeps its cavity, drilling and support mesh.
- Estimate: layer count equals the full slice's for 4 fixtures; time within 3 %.
- Smoke-launched the app (temp config dir) and loaded a fixture project.

## Deferred

- BVH build cancellation, drilling mid-run cancellation (D1.7).
- No UI consumer for `LayerPreview` yet (scrubber: Phase 4).

## Open questions

None new.
