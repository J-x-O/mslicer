# Redesign decisions

Each entry: what was decided, why, and the alternatives considered.

## Process

### P1. Continue through phases without waiting for review

- **What**: Each phase is finished on its own branch
  (`redesign/phase-N-<name>`), branched from the previous phase's branch,
  with a phase report in `docs/redesign/phase-N-report.md`. Work then
  continues with the next phase instead of stopping.
- **Why**: The brief asks to stop at the end of each phase for review, but
  the follow-up instruction was to get as far as possible while away, and
  to park questions instead of stopping. Stacked branches keep each phase
  reviewable on its own (`git diff redesign/phase-0-recon..redesign/phase-1-engine`).
- **Alternatives**: Stop after Phase 0 (wastes the time away); one branch
  for everything (not reviewable per phase).

### P2. `dist/msla_format` is excluded from builds

- **What**: `cargo clippy/test --workspace --exclude msla_format`.
- **Why**: The crate doesn't compile at baseline (invalid pattern in
  `goo/encoding.rs`, missing deps). It is a generated bundle and nothing
  else depends on it. Fixing it is out of scope.
- **Alternatives**: Fix it (unrelated change), remove it from the workspace
  (changes the repository layout).

## Phase 0

### D0.1 The regression harness is a test module of the `mslicer` crate

- **What**: `mslicer/src/regression.rs` (`#[cfg(test)]`), fixtures in
  `mslicer/tests/regression/`.
- **Why**: The things to cover (project loading, hollowing caches, supports
  meshing, the app's slice preparation) live in the `mslicer` binary crate,
  which has no library target. Integration tests in `tests/` can't reach a
  binary's modules.
- **Alternatives**: Split `mslicer` into a lib + bin (large churn before
  anything else); duplicate the preparation logic in `slicer` (would test a
  copy, not the app).

### D0.2 The Slice button's preparation is shared with the harness

- **What**: `platform_models` and `run_slicer` extracted from `App::slice`;
  both the app and the harness call them.
- **Why**: Invariant 2 is about what the app produces. A harness with its
  own copy of the mm→px transform could pass while the app changed.
- **Alternatives**: Drive a headless `App` (needs a wgpu device for
  preview buffers).

### D0.3 References are committed v18 project files plus JSON hashes

- **What**: Six projects (`teapot`, `t_shape`, `hollowed`, `supported`,
  `vlh_eff`, `vector`) are built from recipes once, saved as v18
  `.mslicer`, and committed. Tests always load the file. References store
  per-layer FNV-1a hashes (runs, height, exposure, defects) and stats
  (layers, voxels, defects, slice-level island layers, model-level islands,
  print time).
- **Why**: The files double as migration inputs for the format change in
  Phase 2 (invariant 6) and keep auto-support generation (parallel, possibly
  order-sensitive) out of the comparison. FNV is stable across Rust
  versions, unlike `DefaultHasher`.
- **Alternatives**: Rebuild projects each run (tests generation determinism
  instead of slicing); store full layer data (MBs).

### D0.4 A small printer for the references

- **What**: 1920×1080 px over 76.8×43.2 mm (40 µm), 0.05 mm layers; one
  fixture with 2×2 supersampling.
- **Why**: The whole harness runs in ~2.5 s in debug builds, so it can run
  on every commit.

### D0.5 Vector output is compared loosely

- **What**: For vector slicing only the polygon count and point bounds per
  layer are hashed.
- **Why**: `join_segments` iterates a `HashSet`; which closing point of a
  polygon is dropped varies between runs. Exact hashes are flaky. Fixing the
  bug would change output (invariant 2), so it's recorded as a question.

## Phase 1

### D1.1 Dirty propagation through keys, not an explicit graph

- **What**: Each evaluation node is a `Slot<K, T>` holding its result and
  the key it was computed from. A key contains the identity of the upstream
  results it used (an `Arc` pointer, a `MeshId`) plus its own parameters.
  A node is dirty exactly when the key a consumer asks for differs from the
  stored one.
- **Why**: Nothing has to walk a graph on a change, which is what "a change
  only marks downstream dirty, it doesn't start work" asks for, and nodes
  can't be forgotten in a propagation list. Keys are typed (not only
  hashes) where equality must be tolerant: the cavity key compares the
  placement's `Shape` with a relative tolerance so turning an evenly scaled
  model doesn't recompute it.
- **Alternatives**: A node graph with dirty flags and explicit edges (more
  bookkeeping, same behaviour); hash-only keys everywhere (loses the
  tolerant comparison).

### D1.2 Jobs stay `Task`s; the engine only does bookkeeping

- **What**: `Slot::request` hands out a `Ticket { key, token }`; the job
  that uses it is an ordinary `Task` (cavity, drilling) or thread (support
  mesh, single layer), and reports back with `Slot::finish`.
- **Why**: Integrates with the task monitor and popups for failures as
  they are; no second executor.
- **Alternatives**: An engine-owned thread pool (would duplicate the task
  monitor).

### D1.3 Pending requests aren't queued; consumers ask every frame

- **What**: "One pending request per slot" is the slot's `wanted` key.
  When a superseded job winds down, the next request (consumers re-ask each
  frame, as egui is immediate-mode) starts the newest inputs.
- **Why**: Simplest way to guarantee nothing queues up; no closures with
  stale captured inputs.

### D1.4 Failed jobs keep their slot busy

- **What**: A cavity or drilling job that panics leaves the slot "running".
- **Why**: Otherwise it would be retried, with an error popup, every frame.
  This matches the old `pending`/`drilling` flags, which also stayed set.
  An input change still supersedes it, but a new job only starts after a
  restart. Worth revisiting with the issue system (Phase 3).

### D1.5 The support mesh shows the newest build even if superseded

- **What**: A support-mesh build that finishes after the supports changed
  again is still displayed, but is not "current" (`is_current()` is false,
  slicing rebuilds synchronously).
- **Why**: While dragging a support, every build is superseded before it
  finishes; discarding them would freeze the displayed mesh until the drag
  ends. Display-only use of a stale result isn't the "use" the brief forbids.

### D1.6 Full slice: explicit consumer only (Phase 1)

- **What**: The full slice carries the project's slice key and a cancel
  token. Pressing Slice again cancels a running slice. Edits don't cancel
  it; they only make it "Outdated" (Workspace > Stats > Evaluation).
- **Why**: "Current UI behaviour stays the same": today a slice runs to the
  end on a snapshot. In Phase 5, outputs and analysis become the consumers
  and edits will supersede.

### D1.7 What is not routed through slots, and why

- **BVH and mesh analysis**: inputs are immutable meshes keyed by `MeshId`;
  results never go stale (a changed mesh is a new key) and there is already
  one job per mesh (`UiState::preparing`). `Bvh::build` can't be cancelled;
  that would need a check inside the slicer crate's recursive build.
  Deferred, low value.
- **Support re-routing** (`RerouteSupports`/`SupportEdit`) already is
  at-most-one-running, newest-wins.
- **Island check**: session-only, keyed by pose and print mesh id already.
  It becomes a print-prep node with the stack in Phase 2/3.
- **Drilling** checks its token only before starting: `drill_each` has no
  chunk boundary to check at without changing `tools`.

### D1.8 Single-layer slices are pre-post-processing

- **What**: `LayerPreview` slices the layer as the slicer does, without
  VLH or the elephant foot fixer.
- **Why**: Both work on the whole stack (VLH merges layers). Exact
  otherwise; tested against the full slice for four fixtures, including 2×2
  supersampling.

## Phase 2

### D2.1 Stack rows own order/mode/bypass; large payloads stay typed on the model
- **What**: `Stack` holds `Operation { id, mode, bypassed, op }`. Hollow's
  shell/holes stay in `Model::hollow`, support sets in `Model::supports`,
  Orient's rotation in the model transform. One place per datum.
- **Why**: Moving the payloads into the enum would touch ~200 call sites at
  once with no behaviour gain. Revisit when a second Hollow per model is wanted.

### D2.2 The source is never changed; `Model::mesh` is the evaluated surface
- Repair/Flip are geometry steps evaluated in the background
  (`EvaluateGeometry`, a slot keyed on source identity + steps). Rows after
  Hollow apply to the print mesh (synchronous, cached). Repair/Flip are
  inserted before Hollow by default.

### D2.3 Modes are derived except Frozen
- Hollow: Edited iff manual holes exist. Supports: Edited iff hand-placed
  supports exist. Orient: Manual (shown for Edited) unless auto params set.
  Only Frozen is a stored user choice; this keeps undo consistent.
- Frozen (Phase 2 meaning): edits disabled in the panels, no regeneration;
  frozen Hollow keeps its last cavity/drilling whatever changes; frozen
  geometry holds the last surface. Not persisted as a baked result (a reload
  recomputes once).

### D2.4 Single-action undo when a tool adds a row
- `ModelAction::Batch` bundles "added Supports row" with the support edit,
  so one Ctrl+Z removes both.

### D2.5 Format v19
- Mesh table stores sources (deduped by source). Stack serialized after the
  hollow spec. Supports stay world-space (model-space contacts: Phase 3, v20).
- v18 migration: Orient (manual), Hollow if shell or holes, Supports if any
  sets; modes derived as above.

### D2.6 Bypass
- Hollow bypassed → not hollowed, holes kept. Supports bypassed → not
  rendered, picked or sliced. Orient can't be bypassed or removed.
- The Hollowing panel's checkbox adds the row / bypasses it.

### D2.7 Split Bodies
- Splits the **source**, places bodies like the original (today they came out
  unplaced), copies stack and hollow spec; support sets are not copied.

## Between Phase 2 and 3

### D2.8 Vector slicing made deterministic (intended output change)
- `join_segments` uses a `BTreeSet` and keeps the point that closes each
  polygon (it was dropped). SVG output changes once, then is stable; the
  `vector` reference was re-recorded and is compared exactly again.

### D2.9 The hollowed-sphere island count is a true positive
- The sphere touches the plate in one tiny spot, and its suggested drain is
  drilled exactly there, so layer 0 is empty: the model floats. The
  cascading slice-level detection then (correctly) flags every layer above.
  Not a detector bug. Follow-up: a new issue "hole removes the model's
  contact with the plate" (Phase 3), and the auto-hole rule should not vent
  at the plate contact of a model standing on the plate.

## Phase 3

### D3.1 Issues are derived, never stored
- `core::issues::collect` builds the list from the project and app state
  each frame (kind, severity, source model/operation, text, jump target).
  Nothing to keep in sync, nothing to persist. All old warnings (defective
  mesh, out of bounds, unrouted supports, islands, sealed voids, failed
  holes, slicing defects, Sliced Diff mismatch) are in it, plus the new
  ones: blocked hole, hole not draining, hole at plate contact, orientation
  possibly outdated, stranded supports, frozen result outdated.
- An issue may offer a fix beside its jump (`Issue::fix`); the first is
  "Remove all" for stranded supports (undoable).

### D3.2 Locks removed; supports re-route when the model moves
- The support tool's rotation/scale/Z locks are gone. Supports are still
  stored world-space for the pose they were made at (`frame`). A rigid move
  (XY shift) just translates them; any other pose change (rotation, scale,
  Z) marks them `needs_reroute`.
- After the pose has settled (300 ms, `Debounce<Pose>`), `ReorientSupports`
  re-routes in the background: hand-placed supports from their model-space
  contact (projected onto the surface with `Bvh::closest`, then `route_hand`),
  auto supports by regenerating with the settings recorded in the Supports
  row (`SupportAuto`), without the lift Generate applies (nothing moves the
  model implicitly).
- A hand-placed support whose contact is off the surface (>0.5 mm) or now
  faces up keeps its old route and is marked stranded (Error issue).
- Slicing and Generate wait while a re-route is pending (`Model::ready`).
- Undo restores the support snapshot taken with the pose change, so undoing
  a rotation gives back exactly the old supports.

### D3.3 A new surface re-routes instead of clearing
- Replacing/reloading a mesh, or a geometry op changing the surface, keeps
  the supports and re-routes them on the new surface (answer 12). Keyed on
  the surface's mesh identity, so it starts immediately (no settle).

### D3.4 Holes are model space; auto holes per orientation
- Hand-placed holes were already stored in model space (2.6 holds: clicks
  go through the inverse transform, the viewport always shows the final
  orientation). They are never moved; "hole not draining" reports them.
- Hollow gets an auto-hole rule (`HollowOp::auto_holes`, diameter; format
  v20): one hole at the lowest point of each void for the current
  orientation, recomputed when it changes. On by default for a new Hollow,
  off for migrated projects (old output unchanged). Dragging an auto hole
  pins it as a manual one.
- The rule doesn't vent at the plate contact of a model standing on the
  plate (D2.9 follow-up).

### D3.5 Frozen results persist (format v21)
- Answer 6: a frozen geometry surface, frozen hollow (cavity + drilled
  mesh) are written to the project; stranded support ids and the "frozen
  but outdated" flag too. Frozen supports don't re-route; re-orienting
  raises "frozen result outdated" instead.

### D3.6 Support groups deferred to Phase 4
- Answer 10 (one Supports op holding groups of region + preset + mode) needs
  a region picker (face painting or similar) and a properties surface to
  edit groups; both belong with the Phase 4 UI shell. Today's Supports op is
  the single implicit group "whole model".

## Phase 4

### D4.1 Focus follows the selection
- Properties shows a focus: the selection (default), a model's source, one
  operation, or a print node. The focus remembers the model selection it
  was set with; selecting something else (viewport click, Outliner) falls
  back to showing the selection. Focusing a model's part selects that
  model alone. No extra undo steps: focus is UI state.

### D4.2 Tools are always visible and find their operation
- The viewport toolbar keeps all tools. Picking one with a single model
  selected focuses the operation it edits; Supports adds an empty Supports
  op if missing, the hole tool hollows the model (adds or un-bypasses
  Hollow). Orient never adds anything (every model has its Orient row).
  Adding is one undo step; finding is none.

### D4.3 "Show result up to here" is viewport-only and limited
- The toggle hides the hollow (cavity, holes) and the supports when they
  come after the chosen operation. Slicing is unaffected. The viewport's
  surface already has the geometry rows before Hollow applied, so the
  toggle is disabled (with a tooltip) when such a row comes after the
  chosen op: it couldn't be shown exactly. Not saved.

### D4.4 Where the old panels went
- Hollow op → hollowing settings and holes. Supports op → support tool
  settings, with the island check in a collapsed section (islands are a
  support concern). Orient op → lay-face tool plus the model's transform.
  Print → Profile (slice config), Layer ops (post processors), Overrides
  (placeholder text: only the first-layer exposure exists today), Outputs
  (sliced result, export, remote print). The old tabs stay in View.

### D4.5 Layer scrubber for a quick look at layers
- The cut slider steps by layer height and shows "height / layer n of N";
  arrows step a layer while hovering. With the preview on (default, saved
  in the UI config), the layer at the cut is sliced on its own in the
  background and shown bottom right, downsampled to 512 px. The Sliced tab
  stays for the full sliced result and export, behind the viewport
  (user feedback: the dedicated view is important).
- The label has a fixed size: text of changing height moved the slider
  under the pointer and flickered between two values.
- The slider ends one layer above the last layer start; the end is "no
  cut" (with step snapping the old end could never be reached exactly).
- The cut is kept relative to the top of the models: when the top changes
  (rotation, lift onto supports, scale) it is rescaled, so a change no
  longer leaves the view cut through the model at a stale height.

### D4.6 Layout migration
- `UiConfig::layout_version`. A persisted dock layout of another version
  is replaced by the new default once (Outliner over Properties, Issues /
  Notes / Workspace below, viewport with Sliced behind it). Version 2
  brought the Sliced tab back: version 1 layouts only get that tab added
  next to the viewport. "Reset UI" gives the same default.

### D4.8 A turned model is set down
- Turning a model (rotation fields, Orient tool) sets it down: on the bed,
  or with supports at the height its lowest point had (keeps the lift).
  Same undo step; the rotation field's step now also restores the
  position.

### D4.7 Default stack is a global setting
- Workspace > New models: repair, hollow (auto holes), empty Supports.
  Applied when a model is imported (not to split bodies or projects), as
  part of the "model added" undo step. Default: all off, so imports behave
  as before. "Hollow above N ml" needs the volume, known only after the
  mesh analysis: not done (question).

### D4.9 The stack is a timeline in the Outliner
- User feedback: "show result up to here", freeze and remove belong in the
  Outliner. Each operation row starts with a knob on a line through the
  stack: clicking one shows the model in the viewport as it is after that
  operation (the knobs and the line below go hollow). Clicking the chosen
  knob or the last one shows everything again. Knobs where the viewport
  can't stop (a repair or flip after them is already in the surface) are
  grey and not clickable.
- Each row also has the reorder arrows, bypass, freeze and remove buttons.
  Properties only shows the operation's name and mode above its settings.
