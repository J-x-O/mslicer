# mslicer auto-support: current state & handoff

Intentional handoff document. Paste this into a fresh thread to continue the
mslicer CHITUBOX-derived auto-support work from where it left off. Everything
here is accurate as of the commits listed at the bottom.

## The one rule that overrides all others

This is a **clean-room** reimplementation. Implement **only** from the plan doc
at the repo root:

> `mslicer auto-support CHITUBOX-derived design & implementation plan.md`

Decompiled CHITUBOX source at `~/Documents/chitubox-supports-re/out-d2/` is for
the human contributor only, never for the repo. Do not port, copy, or consult
GPL/decompiled code when writing Rust. When in doubt about a constant or rule,
it must already be written down in the plan doc (section numbers like §2.5,
§3.1.1 reference that doc).

## Where things stand

Milestones M0, M1, M2, M3 are complete. M4 is in progress (slice 1 of roughly
4–5 slices done). M5 and M6 not started.

- **M0–M3 done** — point placement, routing to the plate, collision tests, tip
  clamping, contact depth, model lift, joint sphere.
- **M4 slice 1 done** — brace network (Delaunay + MST + 45° braces, plan §2.5).
- **M4 remaining** — inverted-V forks (§2.5), model-landing supports + tails
  (§2.4), stabilising legs (§2.6), and pillar pre-merge (§2.8, default-off —
  likely skippable, the `2.8` rules don't run in default auto).

## Key code locations

| Path | What it is |
| --- | --- |
| `tools/src/supports/auto/mod.rs` | `AutoPlacement::generate_supports` (Stage 1→2 driver), `route_support`, `generate_braces`, `candidate_sites`, `tip_directions` |
| `tools/src/supports/auto/grid.rs` | Stage 1 ray grid: `footprint`, `sample_surface` |
| `tools/src/supports/auto/points.rs` | Stage 1 point placement: `place_points` (coverage/dedupe) |
| `tools/src/supports/auto/tip.rs` | `clamp_tip_direction` (70° max, §2.2) |
| `tools/src/supports/auto/collision.rs` | `horn_test`/`link_test`/`main_test`/`segment_hits` (§2.7) |
| `tools/src/supports/auto/topology.rs` | **NEW (M4 s1)** brace network: Delaunay+MST+braces |
| `tools/src/supports/auto/detect.rs` | LEGACY three detectors — dead code, no longer driven |
| `tools/src/supports/parts.rs` | `SupportSet`, `Part`/`PartKind`, `add_routed` expansion, serialization |
| `tools/src/supports/mesh.rs` | `SupportPreset` (Light/Middle/Heavy), `build_supports`, `add_part` |
| `tools/examples/support_metrics.rs` | comparison harness (models in `tools/tests/models/`) |
| `mslicer/src/core/project/supports.rs` | `Supports` handle, `replace_auto`/`replace_auto_with_braces` |
| `mslicer/src/task/mesh/generate_supports.rs` | async task wired to generation + braces |
| `mslicer/src/interface/panels/supports.rs` | UI (Generate button, manual placement) |

## Architecture / data flow (important)

`generate_supports` returns `Vec<[Vector3<f32>; 3]>` — routed triplets
`[contact, tip, joint]`. The brace network is a **separate** call
`generate_braces(&routed) -> Vec<Part>`, and the two are combined into a
`SupportSet` by the *callers* (the app task `GenerateSupports` and the
`support_metrics` harness). `replace_auto_with_braces` on the app side stores
the braces on `SupportSet.braces` (a top-level field, distinct from
`support.parts`). `build_supports` already meshes `SupportSet.braces`.

`add_routed` expands a triplet into a `Support` with parts:
`ContactSphere` → `TipCone` → optional `Link` (only when `tip != joint`) →
`Joint` (sphere, radius `1.04 × pillar radius`) → `Pillar` (vertical to plate).

## Toolchain (the user must fix this)

The repo needs **rustc 1.96+** (`std::range::Range`). The user's toolchain is
rustc 1.94.1, which cannot compile the repo as-is. `rustup update` should fix
it. Until then, the workaround used throughout is:

```sh
RUSTC_BOOTSTRAP=1 \
RUSTFLAGS="-Zcrate-attr=feature(new_range_api) -Zcrate-attr=feature(rustc_attrs)" \
cargo test -p tools --target-dir /tmp/mslicer-m4-target
```

`rustc_attrs` is required by the `rustix` transitive dep. `/tmp` target dirs are
cleared between sandboxed terminal calls, so recompilation happens each time —
allow long timeouts (180–400s). If the contributor has applied `rustup update`,
these `RUSTFLAGS`/`RUSTC_BOOTSTRAP` can be dropped and it builds normally.

## Verification commands

```sh
# tools unit tests
RUSTC_BOOTSTRAP=1 RUSTFLAGS="-Zcrate-attr=feature(new_range_api) -Zcrate-attr=feature(rustc_attrs)" \
  cargo test -p tools --target-dir /tmp/mslicer-m4-target

# app compiles
RUSTC_BOOTSTRAP=1 RUSTFLAGS="-Zcrate-attr=feature(new_range_api) -Zcrate-attr=feature(rustc_attrs)" \
  cargo check -p mslicer --target-dir /tmp/mslicer-m4-target

# comparison harness (all models), from repo root
RUSTC_BOOTSTRAP=1 RUSTFLAGS="-Zcrate-attr=feature(new_range_api) -Zcrate-attr=feature(rustc_attrs)" \
  cargo run -p tools --example support_metrics --target-dir /tmp/mslicer-m4-target
```

## Comparison harness notes

Models in `tools/tests/models/` come as `<name>.stl` (input) and
`<name>_supports.stl` / `<name>_supported.stl` (CHITUBOX reference). The harness
reports: `sups` (our point count), `our_vol`/`cb_vol` (mm³ volume), triangle
counts, part-kind histogram, `inter` (part–model triangle intersections),
and gen/build ms.

- `inter` is **non-zero by design** — it counts the intentional contact sink
  (`ContactSphere`/`TipCone` start inside the model). Ignore its absolute value;
  only watch direction-of-change. A proper "unintended intersection" metric
  needs per-part face attribution (deferred).
- Point-count parity is **not meaningful yet** — CHITUBOX exports are pre-merged
  into shared trunks, and our support-merging isn't done (that's the rest of M4).

## Latest metric snapshot (after M4 slice 1)

| model | our_vol | chitubox_vol | our_braces |
| --- | --- | --- | --- |
| Viscountess_of_Blasphemy_Head | 15119 | 15311 | 698 |
| 3dbenchy | 7292 | 11535 | 227 |
| flat_overhang | 1564 | 3609 | 53 |
| t_shape | 1503 | 4858 | 73 |

The head is within ~1.3% of CHITUBOX. Others still under — expected, because
M4's trunk-merging (pre-merge) and model-landing supports aren't done yet, and
those are what CHITUBOX's reference meshes include.

## What to do next (in order)

1. **Inverted-V forks (§2.5 step 2)** — chain collinear plate pillars; emit fork
   points + two 45° links. This is the other half of the brace network slice.
2. **Model-landing supports + tails (§2.4)** — points with no plate route drop a
   pillar onto the model below and end in a short tail cone. Reuses the existing
   collision tests. New parts: `Tail`, and the slim/claw fallbacks.
3. **Stabilising legs (§2.6)** — one diagonal leg per tall, isolated pillar.
4. **Acceptance sweep** — confirm "every pillar >10 mm is braced" (or leg'd),
   braces pass `link_test`, tails sink 0.2–0.3 mm.
5. Then M5 (bases/raft/presets/UI) and, optionally, M6 (manual tools).

Suggested reading in the plan doc when starting each slice:
- §2.4 model landing, §2.5 brace network + forks, §2.6 stabilising legs,
  §2.7 collision tests (already implemented), §2.8 pre-merge (skippable),
  §3.1 part catalogue (for `Tail`/`Slim`/`Boss`/`Base` sizing).

## Pitfalls already hit (don't repeat)

- **NaN from zero-length vectors**: `Vector2::normalize()` on a zero-horizontal
  offset; `add_cylinder` on a zero-length `Link`. Guard `magnitude() <= EPSILON`.
- **Horn-test false positives at `sink=0`**: the tip cone legitimately starts at
  the contact surface. Compute sink = `contact_depth + max(radius, tip_length*0.3)`.
- **`intersect_tip`** is `#[cfg(test)]` only; don't use it in production paths.
- **`cargo fmt -p tools -- --check`** flags pre-existing formatting in
  `detect.rs` (dead code) — do NOT reformat it; revert if it gets changed.
- **Brace truncation**: braces are 45°-only; stop emitting a brace when the full
  45° drop would carry below the 3 mm stop height, rather than emitting a
  truncated (non-45°) segment. This matches §2.5 and keeps every brace link-testable.
- **`nix/flake.{nix,lock}`** are the user's files; keep them OUT of support
  commits unless deliberately bundled.

## Git state

```
84d6650 M4 (slice 1): brace network over pillar topology (plan 2.5)
14878c0 docs: refresh plan 'Current mslicer support code' section to M0-M3
6fd6aa1 M3 (slice 4): joint sphere at tip/link-pillar vertex (plan 3.1.1)
```
`main` is ahead of `origin/main` by 5 commits (unpushed) and has merged
`upstream/main` (v0.10.0). Nothing staged/uncommitted at time of writing.
