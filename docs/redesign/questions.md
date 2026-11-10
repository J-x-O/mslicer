# Questions for review

Collected while working unattended. Each has the best guess that was
implemented (or the reason it was left), so nothing is blocked on it.

## Inputs

1. **`docs/redesign/ui-overview.md` is missing.** The brief says it would be
   placed there. Phases 0–3 don't depend on it. Feature parity work
   (Phase 4+, especially Phase 8's checklist) needs it. *Until then*: parity
   is judged against the current code (panels, menus, shortcuts), listed in
   `architecture-notes.md`.

## Phase 0

2. **Vector (SVG) slicing is nondeterministic** (`slicer/src/slicer/vector.rs`,
   `join_segments` walks a `HashSet`, and one point of each closed polygon
   is dropped depending on where the walk starts). Should this be fixed? It
   changes SVG output, so invariant 2 needs an explicit OK.
   *Recommendation*: fix it in its own commit (sort the segments, keep the
   closing point) and re-bless the `vector` reference.
3. **`dist/msla_format` doesn't compile** (pre-existing). Fix, drop from the
   workspace, or leave? *Current*: excluded with `--exclude msla_format`.
4. **Mesh Repair and Flip Winding Order aren't undoable today.** Phase 2
   turns them into stack operations, which makes them undoable. That's a
   behaviour change in the good direction; flagging it in case "as it does
   today" (invariant 5) was meant literally.
5. The slice-level island detection (Sliced panel) marks 399 of 400 layers
   of a hollowed sphere as having islands. Looks like a false positive on
   rings over a cavity. Not touched (output must not change); worth a look.

## Phase 2

6. **Frozen semantics**: should Frozen persist the baked result in the project
   file (cavity mesh, repaired mesh), or is "hold in session, recompute once
   after reload" enough? Current: the latter.
7. **Split Bodies and supports**: bodies get a copy of the stack but no
   support sets (supports belong to the whole). OK, or assign each support to
   the body its contact lies on?
8. **Split Bodies placement**: bodies now keep the original's placement
   (before: identity transform at the origin). OK?
9. **Repair/Flip default position**: inserted before Hollow. OK?
10. **One Hollow / one Supports per model** for now. OK?
11. **Scale**: kept on the transform (not part of Orient, not a geometry op).
    Should it become its own geometry operation (with shrink compensation)?
12. **Replace/Reload mesh** still clears supports (as before). Keep, or keep
    supports and flag them (Phase 3 issue)?

## Answers (2026-11-09)

1. `ui-overview.md` added.
2. Vector nondeterminism: my call → fix it in its own commit and re-bless `vector`.
3. `dist/msla_format`: my call → keep excluded (generated bundle), note it.
4. Repair/Flip undoable: yes.
5. Slice-level island false positives on hollowed models: investigate (high priority).
6. Frozen: explained; decision pending (see below).
7. Split bodies: ideally each body its own object; geometry stage may need to
   produce multiple bodies. Design pending.
8. Bodies must be individually placeable (important for later splitting features).
9. Repair/Flip before Hollow: yes.
10. One Supports op feels limiting: wants different auto-support types for
    different parts of a model. Design pending (support groups).
11. Scale stays on the transform; shrink compensation later as a geometry op.
12. Supports after a mesh change: design pending.
13. Brief's open questions: recommendations accepted.

### Follow-up answers (2026-11-09)

6. Frozen results are saved in the project file.
7. Split Bodies stays a one-shot action for now; a future Split node must keep
   the tree simple to understand.
10. One Supports operation containing support groups (region + preset + mode).
12. After a mesh change: auto supports regenerate; manual ones are projected
    onto the new surface and re-routed; failures are kept, marked, and raise
    an issue with a "remove all".

### Answers (2026-11-10)

- Support group regions: painted on the surface with a round brush that
  follows the model; precision finer than the mesh's vertices. Plan: store
  the strokes (model-space dabs: centre + radius), not a per-vertex or
  texture map (STLs have no UVs). Membership is "inside any dab"; the shader
  draws it per pixel.
- UI checks: run the app in a nested Hyprland instance (hidden special
  workspace on the host) with input sent only to its socket.
- Auto drain at the plate contact: skip it and report the sealed void.

## Phase 4 (open, best guess applied)

- Default stack "hollow above N ml": the volume is known only after the
  background mesh analysis. Guess: plain on/off now; a threshold could add
  Hollow when the analysis finishes (as its own undo step). Wanted?
- Overrides node: only the first-layer exposure exists. Should Phase 5 add
  per-range exposure overrides, or should the node go?
- "Show result up to here" can't drop a Repair/Flip placed after the chosen
  op (the viewport surface already has it). Disabled there; enough?
- The layer preview slices before the post processors (elephant foot,
  exposure remap). Show them applied too (slower)?
