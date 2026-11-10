# Phase 0 report: reconnaissance and regression harness

Branch: `redesign/phase-0-recon` (from `workiwork` at `c553719`).

## What changed

- `docs/redesign/architecture-notes.md`: map of the codebase (project,
  models, transforms, supports, hollowing, islands, BVH, slicing and
  post-processing, tasks, undo, panels/dock persistence, format v18) and,
  per redesign concept, where it attaches and what is in the way (§8).
- `App::slice` split: `platform_models` (world → platform space, the
  `SlicerModel` list) and `run_slicer` (slice + post-process) are now
  standalone functions in `core/slice/runner.rs`. Pure move, same order of
  models and meshes.
- Regression harness `mslicer/src/regression.rs` with six reference
  projects in `mslicer/tests/regression/` (v18 `.mslicer` files, the two
  small STLs they were built from, and `*.ref.json` references).

## What's tested

`cargo test -p mslicer regression` (≈2.5 s):

| Fixture | Covers |
|---|---|
| `teapot` | plain imported model |
| `t_shape` | the T island test model (model-level island check) |
| `hollowed` | hollowed sphere, suggested drain + a hand-placed side hole, drilled print mesh |
| `supported` | lifted T with auto supports, raft (MIDDLE preset), one manual support |
| `vlh_eff` | VLH + elephant foot fixer, 2×2 supersampling, two instances of one mesh (one turned, exposure 128, in a collection), a hidden model |
| `vector` | vector (SVG) slicing |

Plus a test that the fixtures contain what they claim. Ran 7× in a row:
stable. Full suite: `cargo test --workspace --exclude msla_format` passes;
clippy shows only pre-existing warnings.

## Deferred

- Nothing from the phase scope.

## Open questions

See `questions.md` items 1–5 (missing `ui-overview.md`, nondeterministic
vector slicing, broken `dist/msla_format`, Repair/Flip not undoable today,
a suspicious slice-level island count).
