# Phase 4 report

Branch: `redesign/phase-4-ui-shell`.

Done:
- Outliner tab: models expand into Source and their operations (mode badge,
  issue mark, bypass eye, Add Operation); a Print node with Profile, Layer
  ops, Overrides, Outputs. Clicking empty space clears the selection.
- Properties tab driven by the focus (D4.1, D4.4), with "Show result up
  to here" (D4.3).
- Operation-aware toolbar (D4.2).
- Layer scrubber with single-layer preview (D4.5).
- Live status in the top bar: up to date / computing / stale, with a
  hover breakdown (computing and waiting nodes, re-route, slice state).
- Issues jump to the operation in Properties when the old tab is closed.
- New default layout with a one-time migration (D4.6).
- Default stack setting (D4.7).

Not done (carried to the next step of this phase):
- Support groups (one Supports op holding groups of painted region +
  preset + mode, regions as model-space dabs; D3.6 and answers of
  2026-11-10). Today's Supports op is still the single group "whole model".
- Command palette (spec: later).

Tested: 73 mslicer tests pass. New tests: a tool finds or adds its
operation (no undo step when found); the focus lasts while the selection
does; show-up-to hides what follows; layer downsampling; default stack.
Clippy: only pre-existing warnings.

Manual check (nested Hyprland, separate config dir, so the user's layout
is untouched): the `supported` fixture opened in the new layout; expanding
the model shows Source / Orient (Manual) / Supports (Edited); Orient
focuses with the model's transform; "Show result up to here" on Orient
hides the supports; dragging the scrubber to layer 14 shows the raft
layer preview; the hole tool added Hollow (Auto) above Orient and focused
it.

Acceptance (prepare workflow with only the new surfaces): import (File),
orient (Orient op / lay-face tool), hollow and holes (hole tool / Hollow
op), supports and islands (Supports op), inspect layers (scrubber) all
work without the old tabs; export is under Print > Outputs.

Follow-up after the user's test:
- The Sliced tab is back by the viewport (layout v2, D4.5/D4.6).
- Scrubber: fixed label size (no flicker), a reachable "no cut" end, the
  cut follows the top of the models (D4.5).
- Rotation and the Orient tool set the model down (D4.8).
- The island check ended at once: it was tied to the Islands tab, which
  the new layout doesn't open. It now also lives while Properties is
  open (checked: "All islands supported" on the `supported` fixture).
