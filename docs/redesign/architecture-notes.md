# mslicer architecture notes (Phase 0)

What the code does at the start of the redesign (branch `workiwork`, commit
`c553719`, project format v18), and where each concept of the redesign would
attach. Paths are relative to the repository root. Where the code and
`ui-overview.md` disagree, these notes follow the code.

## 1. Workspace layout

| Crate | Role |
|---|---|
| `common` | Units, `SliceConfig`, `Layer`, RLE containers, the custom (de)serializer, `Progress`. |
| `slicer` | `Mesh`, `Bvh`, `Segments1D`, raster and vector slicer, post-processors (VLH, elephant foot fixer, island detection). |
| `tools` | Hollowing, supports (auto placement/routing/bracing, raft, meshing), island check, repair, generators (height map, phonograph, exposure tests, PCB, image, test pattern, 3D graphics), auto layout, sliced diff, mesh reconstruction. |
| `format/*` | `.ctb`, `.goo`, `.nanodlp`, mesh formats. |
| `remote_print` | Chitu network printing (MQTT/HTTP/UDP). |
| `mslicer` | The egui/eframe app: project, history, tasks, panels, rendering. Binary crate only (no lib target). |
| `experimental` | SDF/B-rep slicers behind features. |
| `dist/msla_format` | Generated bundle of the format crates. **Does not compile** (`0x00.1.0..=` pattern in `goo/encoding.rs`, missing `rand`/`rayon` deps). Pre-existing; excluded from builds with `--exclude msla_format`. |

Crate versions (from `Cargo.lock`): egui/eframe/egui-wgpu 0.33.3, egui_dock
0.18.0, wgpu 27.0.1.

## 2. Core data

### Project (`mslicer/src/core/project/mod.rs`)

```
Project { path, notes, slice_config: SliceConfig, support_config: SupportConfig,
          post_processing: PostProcessing { variable_layer_height, elephant_foot_fixer },
          models: Vec<Model>, collections: Vec<Collection> }
```

- `SupportConfig` (project-wide): auto placement config, the support preset,
  mesh resolution, default lift.
- `Collection { id, name, collapsed, rename }`: models point at it with
  `Model::collection: Option<CollectionId>`. Flat, one level.
- `Project::update()` runs every frame: pushes the support mesh resolution
  into every model and polls their background support-mesh builds.

### Model (`core/project/model.rs`)

```
Model { name, id, collection, mesh: Mesh, bounds: Bounds3D, bvh: Option<Arc<Bvh>>,
        supports: Supports, hollow: Hollow, base_volume, unit: MeshUnit,
        color, exposure: u8 (relative exposure), hidden, ui: ModelUi,
        warnings: MeshWarnings {Defective, OutOfBounds}, buffers (GPU), file }
```

- **Transform**: `Mesh` holds `position`, `scale`, `rotation` (Euler, xyz) and
  a cached transformation matrix (`slicer/src/geometry/mesh/mod.rs`). The
  geometry (`Arc<MeshInner>`) is in model space and shared between
  instances; `MeshId` is the `Arc` pointer. There is no separate "orientation"
  vs. "placement": rotation, scale and position are all one transform.
- Imports are centred (`Mesh::new_boxed_centered`) and kept at their source
  scale; `MeshUnit` only labels the source unit (conversion is applied to the
  scale when the unit is changed in the UI).
- `update_oob` recomputes world bounds and the out-of-bounds warning, and
  pushes the pose into the supports (which may clear them, see below).
- `Clone` gives a new id (duplicate).

### Supports (`core/project/supports.rs`, `tools/src/supports`)

- Two `Arc<SupportSet>`s: `auto` and `manual`, plus the model's `RaftConfig`.
  A `SupportSet` is supports (each with id, contact point, parts, the preset
  it was built with) and braces.
- **Coordinates are world space** for a stored pose `frame`. Moving across
  the plate or turning about z carries supports along lazily
  (`Pose::motion_to` gives an isometry). **Any other change (lift, tilt,
  scale) clears them** (`set_pose` → `clear`, `removed_by_move` notice). The
  undo action for that transform carries a `SupportsSnapshot`, so undo
  brings them back.
- The support mesh is built on a plain `std::thread` per model
  (`Supports::update`), the previous mesh shown meanwhile; `finished_mesh()`
  blocks for slicing.
- Generation (`task/mesh/generate_supports.rs`): lifts the model to
  `default_lift`, waits for hollowing and the BVH, runs
  `AutoPlacement::generate_supports` + `generate_braces` on a task thread,
  and `finish_generation` replaces the auto set (cancellation by token: any
  later edit or undo drops `pending`). Manual supports are re-braced to the
  new auto ones.
- Manual placement (`interface/panels/supports.rs`): `route_hand` from the
  hovered world point and normal on the model's own mesh, `add_manual`.
  Dragging re-routes (`replace`). Editing settings of selected supports
  re-routes them on a worker (`task/mesh/reroute_supports.rs`), keeping
  unroutable ones with their old settings (`UiState::unrouted`).
- **Rotation lock**: in the support tool, with supports present, the Models
  panel disables Align to Bed, Z position, rotation X/Y and scale
  (`interface/panels/models/model.rs`, `LOCKED_TIP`). In the select tool the
  same edits are allowed and clear the supports (undoable).

### Hollowing (`core/project/hollow.rs`, `core/hollow.rs`, `tools/src/hollow.rs`)

- `HollowSpec { shell: Option<Shell {wall, cell}>, holes: Vec<Hole> }` is
  what is saved and undone. Holes are **model space** (point, direction,
  diameter).
- `Hollow` caches `cavity: Arc<ModelCavity>` (model space; keyed by
  `CavityKey { shell, Shape, MeshId }` where `Shape = LᵀL` of the linear
  part of the transform, so moving and turning an evenly scaled model reuse
  it) and `drilled: Arc<Drilled>` (the print mesh with holes cut, keyed by
  cavity ptr + holes + shape). `print_bvh` caches the print mesh's BVH.
- The cavity is computed in **world space** at the current pose and mapped
  back. `ModelCavity::drains` are the *suggested* holes (lowest point of each
  void) computed at cavity time, i.e. for the orientation the model had
  then; they are not recomputed on rotation (rotation doesn't change the
  key). "Add drains" copies them into `spec.holes`, after which they are
  ordinary manual holes.
- `core/hollow.rs::update` (every frame): debounces shell changes
  (`SETTLE` = 300 ms), starts `HollowModel` (cancellable via `AtomicBool`,
  checked inside `tools::hollow::cavity`), then `DrillHoles` (not
  cancellable; result checked against the current holes before use).
- `print_mesh(mesh)` = outer surface + inverted cavity surface, drilled if
  available. Used by slicing, export, island check, and supports
  (`Model::support_surface`).
- Slicing refuses (popup) while any visible model's hollowing isn't ready.

### Island check (`core/islands.rs`, `tools/src/islands.rs`)

Session-only (`UiState::island_check`): `DetectIslands` per model on its
print mesh at the current pose; results are invalidated by pose
(`Pose`) and mesh id. Coverage against support contacts is computed live.
Separate from the **slice-level** island detection in the Sliced panel
(`slicer/src/post_process/island_detection.rs`), which annotates layers.

### Acceleration structures

`Bvh` per `MeshId` (`task/mesh/acceleration_structures.rs`), shared by all
instances (`store_bvh`); `UiState::preparing` prevents duplicate builds;
auto-built for every model when `config.ui.auto_prepare`. `Segments1D` (z
partitioning) is built inside the slicer per slice.

### Mesh analysis

`MeshAnalysis` on load/import: defect check (`Mesh::is_defective`) and
volume (`base_volume`, scaled live by the model's scale). One per mesh id.

### Repair, Flip, Split (`task/mesh/`)

- `MeshRepair` and `FlipWinding` **replace the mesh** via
  `Model::replace_mesh` (keeps the transform, clears supports, BVH, buffers).
  **Neither is in the undo history today.** (Relevant to invariant 5: the
  Phase 2 operations will make them undoable for the first time.)
- `SplitBodies` creates a collection with one new model per connected body;
  the original is removed.
- `ReloadModel` reloads from the source file (same `replace_mesh`).

## 3. Slicing pipeline

- `App::slice` (`core/slice/runner.rs`): collects visible models; for each,
  the print mesh and the finished support mesh (blocking), moves them into
  platform space (`platform_models`: mm → px, z offset of half a layer),
  `Slicer::new(config, models)` (layer count from max z, capped by the
  platform height), spawns a thread.
- Since Phase 0 the preparation and the slice/post-process run are shared
  functions (`platform_models`, `run_slicer`); behaviour unchanged.
- Raster (`slicer/src/slicer/raster`): per layer (×Z supersample) in
  parallel; segments from `Segments1D`; scanline rasterisation with XY
  supersampling and per-model relative exposure via priorities; Z
  downsampling; exposure remap table; layer exposure from first/transition/
  normal configs.
- Post-processing (raster only, in order): **variable layer height**
  (merges similar layers, can change layer count and heights, sets
  `unique_exposure`), then **elephant foot fixer** (erodes the first
  layers). `PostProcessing::process`.
- Vector (`slicer/src/slicer/vector.rs`): segment joining through a
  `HashSet`; **output is nondeterministic** (polygon start and one dropped
  closing point vary per run). Pre-existing; the harness only compares
  polygon counts and bounds for vector.
- Results: `SliceOperation` (progress, `Mutex<Option<SliceResult>>`,
  preview image, interactive preview models). Raster results carry
  annotations (slice-level islands), volume, print time, voxel count.
- Export: `GenericSliceData::file` → `util::export_raster` per format, or
  `SvgFile`. NanoDLP + VLH restriction lives in the Sliced panel.
- Generator tools (`generator_tool!`) produce layers directly into a
  `SliceOperation` (Exposure Test, PCB, Image, Test Pattern, Internal
  Exposure Test); mesh generators (`mesh_generator_tool!`: Height Map,
  Phonograph Record, 3D Graphics) produce a model via `GenerateMesh`.

## 4. Background work

- `TaskManager` (`task/mod.rs`): a list of `Box<dyn Task>` polled once a
  frame on the UI thread with a `TaskApp` (all of `App` except `tasks`).
  Tasks spawn work with `TaskThread::spawn` (a named `std::thread`, panics
  become popups). Status (name, details, progress) feeds the task monitor
  in the top bar after 300 ms.
- No shared executor and no cancellation primitive: hollowing uses an
  `AtomicBool`; support generation a token; support-mesh builds and drilling
  are latest-checked at completion. Rayon is used inside `tools`/`slicer`.
- Debounce: hollowing has its own settle timer; support settings edits
  coalesce ("at most one run, newest wins", `SupportEdit`).

## 5. Undo/redo (`core/history.rs`)

- Global `History { history, future: VecDeque<Action> }`, max 128 entries;
  `ModelRemoved` actions above `history_max_mesh_size` MiB are not tracked.
- `Action`: `Model { id, ModelAction }`, `SliceConfig(SliceConfigAction)`,
  `ModelAdded`, `ModelRemoved`, `CollectionAdded`, `CollectionRemoved`.
- `ModelAction`: name, color, hidden, position/scale/rotation/orientation
  (each with a `SupportsSnapshot`), `GeneratedSupports`, `Supports`,
  `Hollow(HollowSpec)`, relative exposure, collection, move (reorder).
- Each action stores the *old* value; `undo` swaps and returns the inverse.
  Drags record one action at the start (hole drag, support edits).
- Not covered: project notes, support config (project-wide preset/placement
  edits), raft edits are tracked via `Supports` snapshots, mesh repair/flip.

## 6. UI

- `interface::ui` draws the top bar, then the dock (`egui_dock`
  `DockState<Tab>`). Tabs: Hollow, Islands, Logs, Models, Notes, RemotePrint,
  SliceConfig, Sliced, Supports, Viewport, Workspace. The viewport can't be
  closed or floated (`TabViewer::is_closeable`/`allowed_in_windows`).
- Default layout: Models left; Slice Config/Supports/Islands/Notes below;
  Workspace/Remote Print below that; the viewport takes the rest.
- **Persistence**: `config.toml` in the OS config dir
  (`dirs::config_dir()/mslicer`), `Config::ui.panels: Option<Tree<Tab>>`
  (main surface only, serde). Saved on `App` drop. An unknown `Tab`
  variant fails deserialisation of the whole config and falls back to
  defaults (`load_or_default`). **Adding tabs is safe, renaming/removing a
  tab variant resets the whole config** unless handled.
- Tools (`state::Tool`): Select, Support, Orient (lay face on bed), Holes.
  Viewport toolbar paints them; the viewport dispatches clicks and drags per
  tool (`interface/panels/viewport.rs`).
- The view cut (`UiState::cut_height`, slider on the viewport) already
  clips rendering and picking at a height: the base for the layer scrubber.
- Rendering (`render/workspace`): an egui paint callback with a raw
  `*mut App`; models are drawn with their world transform; supports,
  hollow (cavity, hole marks), islands, lines and points have their own
  pipelines.
- Shortcuts: `interface/shortcuts.rs` (global) plus per-panel keys in the
  Models panel.

## 7. Project format v18 (`core/project/storage.rs`)

Big-endian custom serializer. Order: `u16 version`, description (v16+),
notes (v16+), `SliceConfig`, `PostProcessing`, model infos (mesh index,
collection, name, unit, color, exposure, hidden, position/scale/rotation,
auto + manual support sets in **world space**, raft, `HollowSpec`), unique
meshes (vertices, faces), collections, `SupportConfig` (v17+). Versions
2–18 load. The reference projects in `mslicer/tests/regression/*.mslicer`
are v18 and stay as migration inputs.

## 8. Where the redesign attaches, and what is in the way

| Concept | Attaches to | In the way |
|---|---|---|
| Model = source + stack (2.1) | `Model` gets `source` and `stack`; `mesh` becomes the source mesh. | `Model::mesh` doubles as source geometry *and* placement; Repair/Flip mutate it in place. `Mesh` bundles transform with geometry. |
| Scene vs. placement | `Model` position/collection/hidden. | Position, scale, rotation are one transform in `Mesh`; Orient must be split out of it (rotation + scale → Orient; xy position + z turn → placement). Z position is set by "align to bed" and the support lift, i.e. it is partly a derived value. |
| Geometry section (2.2) | `Hollow` (already non-destructive), `MeshRepair`, `FlipWinding`. | Repair/Flip are destructive and untracked; Hollow's cavity is computed in world space and keyed on `Shape` (scale), so scale must remain an input to geometry ops. |
| Orientation slot | New `Orient` op holding rotation (and scale). | Euler angles in `Mesh`; supports clear on tilt; history actions carry snapshots. |
| Print-prep section | Supports, auto drain placement, island check. | Supports are stored in **world space**; they must move to model space (contacts + directions) for 2.4/2.6. Island check is session-only state in `UiState`. |
| Hollow two nodes (2.3) | Split `ModelCavity` (shell) from `drains` (orientation-dependent). | Drains are computed inside `tools::hollow::cavity` in world space. Needs a separate "lowest point per void for orientation R" function; `CavityPart::lowest` exists per part but in the cavity's original frame. |
| Modes Auto/Edited/Frozen (2.4) | Per operation. Supports already separate `auto` and `manual` sets. | Generation is fire-once; there is no "regenerate when inputs change". |
| Placement never implicit (2.5) | Orient/Arrange hold their result. | Auto support generation lifts the model (implicit Z move) — keep it as part of the explicit Generate action only. |
| Viewport shows final orientation (2.6) | Already true (everything renders the world transform). | Picking stores holes in model space already; supports in world space. |
| Lazy evaluation (2.9) | New engine in `mslicer/src/engine` wrapping today's tasks. | Caches are scattered (`Hollow`, `Supports`, `bvh`, `base_volume`), each with its own staleness rules; per-frame `update` functions poll them. |
| Cancellation (2.10) | `Generation` tokens and a chunked check. | `TaskThread` has no cancel; tools functions take an `AtomicBool` only in hollowing; slicer has none (needs a per-layer check hook to stop early). |
| Single-layer slicing | `Slicer` already computes per layer independently. | `Slicer::slice_raster` slices all layers; needs a `slice_layer(i)` (straightforward: the closure body). Post-processors (VLH) change layering globally, so a single layer is only exact without VLH. |
| Issues (2.11) | New `issues` module, one live list. | Warnings live in `MeshWarnings`, `UiState::unrouted`, `Hollow::failed`/`sealed_parts`, Sliced panel defect counts, Sliced Diff, notices. |
| Outliner/Properties (2.12) | New panels; old panels stay. | The Models panel mixes tree and properties; per-tool state lives in `UiState`. |
| Layer scrubber | `UiState::cut_height` + single-layer slice. | Sliced panel's preview is a separate renderer (`render/slice_preview`). |
| Print level (2.1) | `SliceConfig` + `PostProcessing` + printers in `Config::printers`. | Exposure remap is inside `SliceConfig` and applied inside the rasteriser, not as a post-process. |
| Profiles global vs. project | `Config::printers`, `default_slice_config`. | The project carries a full `SliceConfig` copy (that is the per-project override today). |
| Persistence of layouts | `Config::ui.panels`. | Unknown tab variants reset the whole config (see §6). |

## 9. Observations to keep in mind

- `dist/msla_format` does not build; `cargo test --workspace` fails at
  baseline. Use `--exclude msla_format`.
- Pre-existing warnings: `f32::MAX` deprecated
  (`interface/panels/models/mod.rs:57`), `chunks_exact(2)`
  (`slicer/src/slicer/vector.rs:61`), several `skip(1).next()` in format and
  slicer binaries.
- The slice-level island detection flags 399 of 400 layers of the hollowed
  sphere reference (rings over a cavity). Recorded as-is in the reference.
- The model-level island check reports the teapot and the standing T as one
  island each, *on the plate* (their first layer), which is expected.
