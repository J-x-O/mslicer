# Phase 2 report (interrupted before final UI check)

Branch: `redesign/phase-2-stack`.

Done: stack data model (sections, nodes, modes, bypass), non-destructive
Repair/Flip, v19 format with v18 migration, undo for all stack edits
(`Stack`/`Batch` actions), Operations section in the Models panel, Hollowing
and Supports panels write through the stack, Split Bodies copies the stack.

Tested: 58 mslicer tests (stack unit tests; v18 migration of all fixtures;
v19 round-trip slices identically to the references; undo/redo of add,
reorder, bypass, freeze, remove-with-payload; one-step undo of support edit
+ row; Repair/Flip leave the source alone; Flip after Hollow flips the print
mesh). Regression harness passes. Manually: Operations section and Add
Operation menu checked in the running app.

Not done: final in-app check of the Repair report after the "insert before
Hollow" change; Hollow's auto-hole node is declared (`auto_holes` flag) but
not evaluated yet (Phase 3 recomputes it per orientation).
