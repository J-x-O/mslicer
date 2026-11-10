# Agent policy

Rules for agents (and people) changing mslicer. Adapted from SolidCraft's
AGENTS.md and crash-safety guide. The redesign work also follows
`docs/redesign/agent-prompt.md`.

## Crash safety comes first

A panic in the UI thread closes mslicer and loses unsaved work: hours of
placed supports, holes and orientation. Avoiding a crash comes before any
feature work.

- **No new panic sites in non-test code.** That means no `unwrap()`,
  `expect()`, `panic!`, `unreachable!`, `todo!` or `unimplemented!`.
  `scripts/panic-ratchet.sh` counts them per file with clippy, comparing
  against `scripts/panic-baseline.txt`. The baseline may only shrink: after
  removing some, run `scripts/panic-ratchet.sh --update` and commit the
  smaller baseline with the change.
- **What to write instead:**
  - Fallible work returns `anyhow::Result` (or the crate's error type) and
    uses `?`. The task system and the UI report the error; they don't crash.
  - UI code that finds a missing model, operation or layer does nothing
    that frame: `let Some(x) = … else { return };`. An id from last frame
    can be gone (undo, delete, reload).
  - Use `get()` rather than `[i]` for indexes that come from outside the
    function: files, ids, layer numbers, hit tests, user input.
  - An unfinished feature returns an error ("not supported yet: …") or is
    disabled in the UI. It never panics.
- **Untrusted input:** project files, mesh files, printer formats, network
  replies (remote print) and config files. Check that numbers are finite,
  cap sizes and counts before allocating, and refuse files that don't add
  up rather than indexing past their end. Old projects must still load.
- **Background tasks:** the task system catches panics on task threads. Do
  not rely on that: a panic there still loses the task's result and can
  leave state half updated.
- **Writing files:** write the project, config or export to a temporary
  file next to the target, then rename it over the target. A crash or a
  full disk then leaves either the old file or the new one, never a
  truncated one.
- **Locks and threads:** don't `unwrap()` a lock. Recover a poisoned lock
  (`unwrap_or_else(PoisonError::into_inner)`) or skip the frame.
- **Allowed:** `unwrap`/`expect` in tests, in `bin/` tools, and in startup
  code before the window opens, where failing fast is right. Startup code
  should still prefer `.context(...)?`. An invariant that the type system
  can't express needs a comment saying why it holds, and still goes
  through the ratchet.
- **Every crash fix comes with a regression test** that reproduces it. Put
  it in the area's existing test module, not in a new file per bug.

## Before every commit

- `cargo fmt -p mslicer` (`--all` fails on `msla_format`).
- `cargo clippy --workspace --exclude msla_format`, with no new warnings.
- `cargo test --workspace --exclude msla_format`.
- `scripts/panic-ratchet.sh`.
- Run these inside `nix develop ./nix -c …`.

## Behaviour

- Slicing output must not change unless that is the point of the change.
  The regression projects in `mslicer/src/regression.rs` check this.
- Never silently delete the user's work. Every mutating operation is one
  undo step.
- Moving a model never recomputes anything implicitly.
- A changed default keeps the old behaviour available as a setting.
- Project settings and global settings stay separate.

## UI checks

- A visible change gets a real look in the running app. Use a nested
  Hyprland instance with its own config dir. Never send synthetic input to
  the user's own session.
- Keep screenshots few, and crop them to what matters.

## Process

- Write code, comments, commit messages and docs in English.
- Keep commits small and focused, one topic each.
- Don't push to upstream or open PRs against it.
