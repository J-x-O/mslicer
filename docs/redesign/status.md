# Status and plan

Where the work stands and what comes next, so it can be picked up on
another machine. Updated 2026-11-10, branch `redesign/phase-4-ui-shell`.

## Done recently

- Outliner: the stack shown as a timeline, with show-up-to knobs; freeze and
  delete moved there from Properties (`706f302`).
- Agent policy in `AGENTS.md` (crash safety first) and the panic ratchet,
  `scripts/panic-ratchet.sh` with `scripts/panic-baseline.txt` (`81c0be7`).
- Panic-site removal, baseline 291 → **161**:
  - `3c76d39` Projects load and save without panicking:
    - The deserializers record truncation and errors.
    - `read_list` caps counted lists.
    - Saves are atomic via `common::fs::write_atomic`.
  - `4669d0d` Mesh and sliced-file loads report errors:
    - `Mesh::validate`.
    - `TaskThread::poll_result` shows an error popup.
  - `9169171` UI panels and tools:
    - A model or layer that's gone that frame is skipped.
    - Errors go to `PopupManager::report`.
    - The sliced export and the PCB SVG export are atomic.
    - The dock state is no longer an `Option`.
  - `9099041` Core and tasks:
    - Poisoned locks are recovered.
    - The network tasks report errors.
    - The sliced export refuses a format that doesn't fit.
    - Startup returns errors instead of unwrapping.

## Open: panic sites (161 left)

Get the exact list with:

```
nix develop ./nix -c cargo clippy --workspace --exclude msla_format \
  --message-format=short -- -W clippy::unwrap_used -W clippy::expect_used \
  -W clippy::panic -W clippy::unreachable -W clippy::todo -W clippy::unimplemented
```

In this order:

1. **Render (`mslicer/src/render`, about 45 sites).**
   - `buffer.write(&uniform).unwrap()`: encase writing into a `Vec` can't
     fail. Add a small helper (e.g. `render::uniform_bytes(&T) -> Vec<u8>`)
     and use it everywhere.
   - `resources.get::<…>().unwrap()` in paint callbacks, missing bind
     groups and render targets: skip the draw with
     `let Some(..) else { return }`.
   - `pass/base.rs`:
     - `try_get_buffers().unwrap()`
     - `project.model(id).unwrap()`
     - `supports.mesh().unwrap()`
     - the uniform offsets
   - `model/preview.rs`: the preview readback (map_async, poll, recv,
     `from_raw`) should return `Option`/`Result`, so a failed readback means
     no preview rather than a crash.
   - `render/mod.rs:46`: `wgpu_render_state` is startup; turn it into an
     error.
2. **remote_print crate.** It parses network input, so it matters most after
   render.
   - `manager.rs`: `protocol_version(..).unwrap()` and `unreachable!`.
   - The mqtt packets, `v1`, `v3` and `http.rs`: check lengths before
     indexing, and return errors for bad packets.
3. **common containers.**
   - `container/bitvec.rs`, `clusters.rs`, `rle/png.rs`, `rle/downsample.rs`,
     `image.rs`, `slice/format.rs`.
4. **Printer formats (untrusted input).**
   - goo `encoding.rs`, ctb `v4`/`v5`, nanodlp `layer.rs`/`file.rs`.
   - Their deserializers aren't yet checked against cut-short or damaged
     files the way projects are. Add the same kind of cut and damage tests.
5. **tools and slicer crates.**
   - tools: hollow, the PCB code, repair, supports/auto, auto_layout.
   - slicer: raster edge table, mesh, the elephant-foot fixer, util.
6. **Decide on `bin/`.** The policy allows unwraps in `bin/` tools. Either
   make the ratchet skip `*/bin/*`, or leave them counted.

Deliberately kept: the two `expect`s in `Stack::orient_inner`/`orient_mut`.
`ops` is private and every way to build a stack adds an Orient, so they
hold; there's a comment on them.

Missing regression tests: the core and task fixes in `9099041` have none.
They're mostly lock and lookup handling that is hard to trigger in a test.
Add tests where a cheap reproduction turns up, e.g. `Stack::move_by` with an
unknown id, or `supports::single` returning nothing.

Every step: fmt, clippy, test, then `scripts/panic-ratchet.sh --update`,
and one small commit per area.

## Then

1. **Support groups.** The next feature, planned before the crash-safety
   detour.
2. **Phases 5–8**, as in `agent-prompt.md`. Each phase gets its own branch,
   report and decisions entry, and open questions go in `questions.md`.

## Notes for the next session

- Build and test inside `nix develop ./nix -c …`. Run the app with
  `nix run ./nix`.
- `cargo fmt -p mslicer` (plus `-p common` etc.); `--all` fails on
  `msla_format`. `dist/msla_format` is an excluded copy; ignore it.
- Pre-existing clippy warnings, not ours:
  - `skip().next()` in the bins
  - `chunks_exact` in `vector.rs`
  - `f32::MAX` in `panels/models/mod.rs`
  - `needless_borrow` in bulk_defects
- UI checks only in a nested Hyprland instance, never on the user's own
  session.
- Don't push to upstream or open PRs against it.
