# mslicer redesign: operation stacks and lazy evaluation

You are implementing a redesign of mslicer in my fork. mslicer is an open-source MSLA resin slicer written in Rust, using `egui`/`eframe` for the UI, `wgpu` for rendering and `egui_dock` for the docking layout.

This document describes the target concept, the invariants you must not break, and a phased plan. Work through the phases in order, and **stop at the end of each phase** for my review.

A complete inventory of the current UI and features exists as `docs/redesign/ui-overview.md` (I will place it there; if it's missing, ask me for it). Treat it as the source of truth for feature parity, but treat **the code** as the source of truth for actual behaviour. Where they disagree, trust the code and note the difference.

---

## 1. Why this redesign

Slicers like Chitubox and Lychee feel strict because their steps are **destructive**: once supports exist you can't freely rotate, slicing is a separate mode, and tools are one-shot. mslicer has some of the same problems (for example, X/Y rotation is locked in the support tool once a model has supports, and generators like Height Map are fire-and-forget popups).

The goal is not a configurable wizard. The goal is that **anything can be changed at any time without losing work**, while still offering one-click automation. We get there by modelling the data instead of the workflow.

---

## 2. The concept

### 2.1 Data model

- **Model** = a **source** + an ordered **operation stack**.
  - A source is either an imported mesh, or a **procedural source** (Height Map, Phonograph Record, Exposure Test, Internal Exposure Test) whose generator parameters stay editable after creation.
- **Scene** = all models plus their placement (position, collections, visibility).
- **Print** = the scene + a **profile** (printer, resin, base exposure) + a stack of **layer operations** (exposure remapping, elephant foot fixer, variable layer height) + **layer-range overrides** + **outputs**.
- **Layer sources** (Printed Circuit Board, Image Layer, Test Pattern) produce layers directly and enter at the print level, skipping model stacks.
- **Outputs** are targets: save `.ctb`/`.goo`/`.nanodlp`/`.svg`, send to printer, export prepared mesh. A print can have several.

### 2.2 Stack sections

A model's stack has three sections, defined by **whether an operation depends on how the model sits on the bed**:

1. **Geometry section** (model space, **user-reorderable**): Hollow (shell + manual drain holes), Mesh Repair, Flip Winding Order, and future geometry operations (booleans, emboss, shrink compensation). These don't depend on orientation; their order relative to each other depends on intent.
2. **Orientation** (a rigid transform, its own slot): manual, or auto-orient. Commutes with all model-space operations.
3. **Print-prep section** (bed space, **fixed order**): auto drain-hole placement, Supports (including raft), island checks.

Implement the stack as an ordered list where each operation declares its section. Reordering is allowed only within the geometry section.

### 2.3 Evaluation nodes vs. UI rows

A UI row is a user concept; the engine may split it into several evaluation nodes. The section rule applies to **evaluation nodes**, not rows.

**Hollow is one row with two nodes:**
- *Shell + manual holes*: model space, evaluated at the operation's position in the stack.
- *Auto hole positions* ("a hole at the lowest point of each void"): takes the orientation as an input, resolved after orientation. Re-orienting recomputes only this node; the cavity stays cached.
- Dragging or editing an auto hole pins it in model space and turns it into a manual hole.
- Manual holes are cut at the Hollow operation's stack position. If a later geometry operation covers a hole, that is allowed but must raise an issue ("hole 2 is blocked by a later operation").

### 2.4 Operation modes

Every operation is in one of three modes:
- **Auto**: fully parametric, regenerates when inputs change. Changing a parameter keeps it Auto.
- **Edited**: auto result plus manual edits layered on top (placed supports, drilled holes). Direct manual edits switch Auto → Edited.
- **Frozen**: baked, never recomputes. Only set explicitly by the user.

Manual edits are stored in **model space** (e.g. support contact points and directions), so they survive re-orientation.

### 2.5 Placement operations never move things implicitly

Orient and Arrange must **not** recompute on their own when upstream inputs change, because that would move models under the user's cursor.

- Auto-orient computes once (on import, or on explicit "Re-run") and then **holds**. Upstream changes mark it *possibly outdated* and offer a one-click re-run.
- Editing a model never re-lays out the plate.
- Rule: **nothing that moves a model in the scene recomputes implicitly.** Everything else (cavities, supports, layers) may recompute freely.

### 2.6 Viewport always shows the final orientation

Editing an upstream operation does **not** show the model in its pre-orientation pose. The viewport always renders the final oriented model. Clicks are transformed back into model space via the inverse orientation transform before being stored; rendering applies the transform. Optionally, downstream results (e.g. supports) can be hidden while editing an upstream operation; the orientation is never undone for display.

### 2.7 Manual-hole intent

Manual holes stay at the same model-space position after re-orientation, even if that's no longer the lowest point of their void. Do **not** move them. Instead raise an issue: "void 2 no longer drains, its hole is now above the resin line".

### 2.8 Auto-orient and hollowing

Auto-orient sees the hollowed mesh (Hollow is above orientation). When scoring a candidate orientation, it assumes the auto-hole rule is applied in that orientation (each void vented at its lowest point), which is cheap to simulate. Real auto-hole positions are computed once, for the chosen orientation.

### 2.9 Lazy, demand-driven evaluation

- Every evaluation node and the print hold a **cached result** plus a **hash of their inputs**.
- A change only marks downstream nodes **dirty**. It does not start work.
- Work starts only when a **consumer** requests a result: viewport rendering a mesh, the layer scrubber, the issues list, an analysis panel, or an output.
- Cache granularity is **per model, per node**. Changing one model's wall thickness invalidates its Hollow and downstream nodes only. Moving a model on the plate invalidates nothing in its stack, only print-level results.
- Cheap, slice-free information is always available: out-of-bounds (bounding box), layer count, rough print time, resin volume from mesh volume.
- The **layer scrubber slices only the layer under it**, on demand.
- A **full slice** happens only for whole-print analysis (surface area, defects plots) or an output. Print time is an estimate until a full slice exists (exact values depend on VLH).

### 2.10 Cancellation

Requested results can go stale before they finish. Handle it with:
- **Generation check**: each job carries the input hash it started with and checks between chunks (per layer, per support, per voxel slab) whether it's still current; if not, it stops. Results arriving with an outdated hash are discarded.
- **Latest-wins per cache slot**: at most one running job and one pending request per slot; nothing queues up.
- **Debounce**: drag-value widgets request work after a short pause or on release.
- **Optional idle speculation**: when nothing has changed for a few seconds, compute likely-needed results (e.g. full slice) at low priority. This is a user preference and uses the same cancellable jobs.

### 2.11 Issues

All semantic warnings become structured **issues** in one live list, each with a source (model + operation/node, or print) and a jump target. Existing warnings to preserve include: defective mesh, out of bounds, unrouted supports, unsupported islands, sealed voids, failed holes, slicing defects, layer-resolution mismatch (Sliced Diff), plus the new ones above (blocked hole, hole no longer draining, orientation possibly outdated).

### 2.12 UI surfaces (target)

- **Outliner**: scene tree. Each model expands into its source and operations (mode badge, bypass toggle, "Add operation"). A **Print** node expands into Profile, Layer ops, Overrides, Outputs.
- **Viewport**: always present. Toolbar shows Select and Move plus the tools of the selected operation (e.g. place support, mini support, drill hole, lay face on bed, paint protected faces). All tools remain reachable via shortcuts and the command palette regardless of selection. Picking a tool selects its operation (creating it if missing, e.g. the drill tool creates a Hollow operation).
- **Layer scrubber** on the viewport: clips the model at the chosen height and shows that layer. Replaces the separate Sliced tab.
- **Properties**: shows whatever is selected (model, source, operation, support, hole, print node).
- **Issues** list.
- **Live status** in the top bar (up to date / computing / stale).
- **Default stack** setting: the stack applied to newly imported models (e.g. "auto-orient, hollow above 20 ml, middle supports"). This is the one-click automation.
- **Command palette** (Ctrl+K) for every action.
- Workspace is split into **Preferences** (theme, colours, limits, update checks, Spacenav, remote-print services/webhook/ports, printer presets) and a **Diagnostics** drawer (logs, frame time, memory, task monitor). Camera controls move onto the viewport.
- Docking stays: these are default panels, users can still split, float and hide them.

---

## 3. Invariants

Never break these. If a task seems to require it, stop and ask.

1. **The app is fully usable after every phase.** The old UI keeps working until each part has a replacement.
2. **Slicing output does not change** for the same project and settings, unless a phase explicitly says so. Enforce with the regression harness from Phase 0.
3. **Never silently delete user work.** Manual supports and holes that can't be re-routed or placed stay, are marked, and raise an issue.
4. **Nothing that moves a model recomputes implicitly** (2.5).
5. **Undo/redo covers every mutating operation**, as it does today. History stays global to the project.
6. **Old projects load.** The current `.mslicer` format (version 18) must convert into the new model on load. Saving writes the new format.
7. **Project vs. global stays distinct**: stacks, scene, print overrides and outputs are per project; profiles, printer presets, default stacks and preferences are global.
8. **The viewport is always available** and can't be closed.
9. **Feature parity**: by the end, every item in section 13 of `ui-overview.md` exists somewhere.

---

## 4. Working rules

- Work on a branch per phase (`redesign/phase-N-<name>`), small focused commits.
- Run `cargo fmt`, `cargo clippy` and `cargo test` before each commit; don't introduce new warnings.
- Check actual crate versions in `Cargo.lock` before using `egui`, `egui_dock` or `wgpu` APIs. Don't guess APIs.
- Record every non-trivial decision in `docs/redesign/decisions.md` (what, why, alternatives).
- Don't push to or open PRs against upstream. This is my fork.
- When existing behaviour is unclear, read the code; if still unclear, ask rather than invent.
- At the end of each phase, write a short report: what changed, what's tested, what's deferred, open questions. Then stop.

---

## 5. Phases

### Phase 0: Reconnaissance and regression harness (no behaviour change)

- Map the codebase and write `docs/redesign/architecture-notes.md`: project/model structs, collections, transforms, support data (auto + manual sets, raft, support config), hollow spec and cavity computation, acceleration structures, slicing pipeline (raster and vector, post-processors), the background task system, undo/redo actions, panel/dock system and its persistence in `config.toml`, project serialization (v18).
- For each concept in section 2, note where it would attach and what's in the way.
- Build a **regression harness**: a set of reference projects (teapot, the T-shaped island test model, a hollowed model with holes, a supported model, a VLH + elephant-foot config) that are sliced headlessly; store per-layer hashes and key stats. Add a test that compares current output against them.
- Stop and report.

### Phase 1: Evaluation engine behind the current UI

- Implement input hashing, cached node results, dirty propagation, consumer-driven requests, cancellable jobs with generation checks, latest-wins slots and a debounce helper. Integrate with the existing task system and task monitor rather than replacing it.
- Route existing computations through it: mesh analysis/volume, acceleration structures, hollow cavity, drain-hole drilling, support generation/re-routing, island detection, full slice.
- Add **single-layer on-demand slicing**.
- Current UI behaviour stays the same (the Slice button still triggers a full slice).
- Acceptance: regression harness passes; tests show stale results are discarded and superseded jobs stop early; editing one model doesn't invalidate another model's caches.

### Phase 2: Operation stack data model and project format

- Introduce source + operation stack per model, with sections (2.2), evaluation nodes (2.3), modes (2.4).
- Initial operations: Hollow (shell, manual holes, auto-hole rule), Orient (manual transform; auto params can be a stub until Phase 7), Supports (auto params, manual contacts in model space, raft), Mesh Repair, Flip Winding Order. Today's destructive Repair and Flip become operations.
- Split Bodies stays a one-shot action: it creates new models, each with a copy of the original stack.
- Project format version bump with migration from v18. Add migration tests using the reference projects.
- Undo integration for all stack edits (add, remove, reorder, mode change, parameter change, manual edits).
- Adapt the existing panels to read and write through the stack so the current UI still works.
- Acceptance: old projects round-trip; regression harness passes; undo works for all stack edits.

### Phase 3: Re-orientation without locks, and issues

- Remove the support-tool rotation/scale/Z locks. Re-orienting a supported model re-routes manual supports from their model-space contacts and regenerates auto supports.
- Unroutable supports keep their previous route, are marked, and raise an issue.
- Implement the structured issue system (2.11), migrate all existing warnings into it, and add the new issues (blocked hole, hole no longer draining, orientation possibly outdated).
- Viewport editing in model space via inverse transform (2.6).
- Acceptance: rotate a model with manual supports and holes; nothing is lost; issues appear where expected.

### Phase 4: New UI shell

- Outliner, selection-driven Properties panel, operation-aware viewport toolbar, layer scrubber with clipped rendering, Issues panel, live status, default-stack setting (2.12).
- Keep the old panels available via the View menu until parity is confirmed.
- New default dock layout; migrate persisted layouts (on an unknown or old layout, reset to the new default once).
- Acceptance: the full prepare workflow (import, orient, hollow, holes, supports, islands, inspect layers) is possible using only the new surfaces.

### Phase 5: Print level

- Profile as a global entity (printer, resin, base exposure; existing printer presets fold in).
- Layer operations (exposure remapping, elephant foot fixer, VLH) with bypass, same as model operations. Keep the NanoDLP-with-VLH restriction.
- Layer-range exposure overrides, editable from the scrubber.
- Outputs as targets (save in each format, send to printer, export prepared mesh), several per print.
- Move all Sliced-panel analysis (resin usage, layer/file properties, surface area, defects, preview image, reconstruct mesh, detect islands) into Properties for the Print node and the scrubber.
- Remove the Slice button as a required step: outputs and whole-print analysis request a full slice on demand.
- Acceptance: regression harness passes; every Sliced-panel feature has a home.

### Phase 6: Sources

- Procedural sources for Height Map, Phonograph Record, Exposure Test, Internal Exposure Test, with editable parameters that re-run the stack.
- Layer sources for Printed Circuit Board, Image Layer, Test Pattern at the print level.
- Replace the Tools menu with an "Add source" gallery; keep Auto Layout, Quick Layout and Collect Instances as Arrange actions; Sliced Diff becomes a comparison option on the Print node; keep 3D Graphics and the experimental slicers where they fit.

### Phase 7: Auto-orient interface

- Orient operation in auto mode with objectives (fewer supports, low peel force, short print time, protect painted faces), a "paint protected faces" viewport tool, and the hold/possibly-outdated semantics (2.5).
- Scoring hook that assumes vented voids (2.8).
- Implement a **simple, clearly replaceable** placeholder algorithm (e.g. sample candidate orientations and score by overhang area and cross-section). The real algorithm is a separate task; keep the interface clean for it.

### Phase 8: Preferences, diagnostics, palette, cleanup

- Split Workspace into Preferences and the Diagnostics drawer; move Remote Print setup into Preferences and printers into Outputs.
- Command palette with every action and its shortcut.
- Optional idle speculation preference (2.10).
- Walk the parity checklist in `ui-overview.md` section 13 and report every item's new location; then remove the old panels.

---

## 6. Open questions to flag, not decide

Raise these when you reach them, with a recommendation:

- Whether per-model slice results can be cached and composited when a model only moves in XY (anti-aliasing may make this inexact).
- Where per-model relative exposure lives (model property vs. layer operation scoped to a model).
- Whether to add a "show result up to this operation" display toggle beyond hiding downstream results.
- How copying or sharing a stack between models should work (copy vs. linked preset).
- Any place where the existing code contradicts this document.
