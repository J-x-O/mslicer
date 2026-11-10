# Phase 3 report

Branch: `redesign/phase-3-reorient-issues`.

Done:
- Issues panel (new dock tab) with one derived list of every warning, jump
  targets (model + panel, hole, island, supports), and fixes ("Remove all"
  for stranded supports).
- Rotation/scale/Z locks removed from the support tool. Supports re-route in
  the background after the model settles; hand-placed from their contacts,
  auto regenerated with recorded settings; unroutable ones kept and marked.
- Same re-route after a surface change (reload/replace mesh, geometry ops)
  instead of clearing supports.
- Auto holes per orientation (format v20); new hole issues (not draining,
  at plate contact, blocked by a later geometry op).
- Frozen results saved (format v21).
- Deterministic vector slicing (re-blessed `vector` reference) and the
  hollowed-sphere island investigation (true positive, now an issue).

Tested: 67 mslicer tests pass, regression harness included. New tests:
tilt re-routes and loses nothing and doesn't move the model; upside-down
strands the manual support, raises the issue, "Remove all" + undo; a new
surface re-routes; hole issues follow orientation; v21 round trip.
Clippy: only pre-existing warnings.

Manual check (nested Hyprland, isolated input): opened the `supported`
fixture, rotated the model 25° with no lock; supports re-routed under the
tilted model, no issues, undo restored the rotation. This found a crash
(stale support buffers drawn while a re-route was pending), fixed.

Not done / moved:
- Support groups (D3.6) → Phase 4.
- "Optionally hide downstream results while editing upstream" (2.6) → Phase 4
  ("show result up to this operation" toggle).
