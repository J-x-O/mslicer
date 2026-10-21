# mslicer auto-support: CHITUBOX-derived design & implementation plan

Oct 19, 2026 · @JoyJab Gamer

## Summary

mslicer's auto-supports should be rebuilt as a three-stage pipeline modelled on CHITUBOX Basic 2.3.1: pick contact points by surface coverage, route them into collision-checked trees, then build geometry from a small set of parametric parts. Each stage directly targets one of today's three defects.

| mslicer defect today | Root cause (section 3) | CHITUBOX mechanism that fixes it |
| --- | --- | --- |
| Cluttered supports | Every overhanging face, edge and low vertex gets its own support, with no coverage model or merging | Coverage-based point placement on a vertical ray grid, deduplication by distance, then merging neighbours into shared trunks (Delaunay + minimum spanning tree) |
| Links that run nearly parallel to the plate | Routing places a free middle point with no angle limit | Every connection is limited by a maximum connect angle (70° by default) and a minimum support height; otherwise the point goes straight down or lands on the model |
| Tips that pass through the model | No collision test on the tip or the link from tip to pillar | Separate collision tests for the tip (horn), the main pillar, links between pillars, cross braces and vertical drops, each against the model's triangles |

Scope: automatic point placement, routing, support geometry, raft and bracing for resin printers, with the Saturn 4 Ultra 16K as the reference printer. Out of scope: hollowing, drain holes, auto-orientation and manual support editing UI beyond what the data model needs.

The plan (section 9) lands in five milestones. Each one is shippable on its own and is checked against CHITUBOX output on the same test models.

## Provenance and clean-room rules

Everything about CHITUBOX in this doc comes from CHITUBOX Basic 2.3.1 for Linux, decompiled with Ghidra 12 in headless mode and read by hand. No CHITUBOX code is reproduced here; the doc describes behaviour, structure and constants only.

**Sources**

| Source | What it gave | Readability |
| --- | --- | --- |
| `lib/libCbdModel.so.1` (3.8 MB) | The whole algorithm: point placement, routing, geometry, raft | Not stripped: real C++ class and function names, no debug info, so struct fields are offsets |
| `lib/libCbdKernel.so.1` (31 MB) | Setting registration, the auto-support and tree-support operations | Has DWARF debug info, so field names survive |
| `bin/Resources/Conf/MachineConfig/ELEGOO/ELEGOO Saturn 4 Ultra 16K.cfgx` | Every default value, for the Light, Middle and Heavy presets | Plain JSON |

The extraction is reproducible with the flake in `~/Documents/chitubox-supports-re`. `nix run . -- libCbdModel` decompiles only the support classes listed in `filters/libCbdModel.txt`, plus one level of callees. The output is one `.c` file per class, an address index, and a call graph.

**Confidence markers used in this doc**

- **Verified**: read in the decompiled code, or taken from the JSON defaults.
- **Inferred**: deduced from function names, call order and code shape, not traced line by line. Every inferred claim is marked as such.

**Rules for the implementer**

1. Implement from this doc, not from the decompiled output. mslicer is GPL-3.0, and code translated from CHITUBOX would be a derived work of proprietary software.
2. Constants and parameter names are facts about the product's behaviour and may be used as defaults.
3. If a detail is missing, raise it as an open question (section 10) rather than reading the decompiled code during implementation. Someone else can answer it from the decompiled code and add the answer here.

## Current mslicer support code

mslicer v0.9.2 (commit `19abc4a`, 30 Aug 2026) places supports with three independent detectors and routes each one alone. Nothing checks the finished support against the model, and supports are drawn but never sliced.

### Where the code lives

| File | Lines | Role |
| --- | --- | --- |
| `tools/src/supports/mod.rs` | 175 | `SupportConfig`, `generate_supports`, `route_support`, `build_raft_mesh` |
| `tools/src/supports/detect.rs` | 196 | Overhang faces, edges and lowest vertices |
| `tools/src/supports/quazirandom.rs` | 24 | Low-discrepancy 2D point sequence for face sampling |
| `mslicer/src/project/supports.rs` | 102 | Stores auto and manual supports as 3 points each; builds the render mesh |
| `mslicer/src/windows/supports.rs` | \~140 | UI: "Generate" button and manual click placement |

A support is always exactly three points: contact point, tip end, and one middle point. The pillar then drops vertically from the middle point to z = 0.

### Current pipeline

1. **Overhang faces**: every face whose normal is within `max_angle` (30°) of straight down.
2. **Face supports**: quasi-random points over the whole build plate (about one per 50 mm²), each cast as an upward ray. Every overhanging face that ray crosses gets a support, at every height.
3. **Edge supports**: chains of edges where neighbouring face normals differ by more than 0.1 rad (5.7°), sampled every 20 mm.
4. **Point supports**: vertices lower than all their neighbours, on a downward-facing face.
5. **Dedupe**: face supports within 5 mm of an edge or point support are removed, and edge supports within 5 mm of a point support. Supports of the same kind are never deduplicated against each other.
6. **Routing** (`route_support`): start at `point + normal × tip_length` (3 mm). Step away from the closest model surface with momentum, never upward, for up to 100 steps, until a straight-down ray misses the model. That spot becomes the middle point.
7. **Mesh**: a tapered cylinder (radius 0.2 → 1.0) for the tip, a cylinder for the arm, a vertical cylinder to the plate, spheres at the joints. The raft is one convex hull around every pillar foot.

### Root causes of the three defects

**Clutter**

- Face sampling adds a support for every overhanging face along each ray, including overhangs that sit directly above other parts of the model. Those are the supports that later have to wander sideways.
- Edge detection fires on any 5.7° crease. Finely tessellated curved surfaces produce many such edges, each sampled every 20 mm.
- On curved tessellated surfaces, many vertices count as local minima.
- There is no coverage model: nothing asks whether an area is already held by a nearby support. Same-kind supports are never deduplicated.
- Every support gets its own pillar to the plate. Nothing merges neighbours into shared trunks.

**Near-horizontal arms**

- `route_support` moves freely in x and y and only limits z to "not up". Near a wall the gradient is almost horizontal, so the walk runs sideways, up to 100 steps of up to 1 mm.
- The arm from tip end to middle point is a straight line with no angle limit, so it can end up nearly flat.
- Tips follow the surface normal. On steep faces the normal is nearly horizontal, so the tip is too.

**Tips through geometry**

- Neither the tip segment nor the arm is ever tested for intersection. Only the final vertical drop is ray-tested. `Bvh::intersect_segment` exists but isn't used.
- The arm cuts the corner of the walk path, so it can pass through geometry the walk went around.
- `place_face_supports` stores the **untransformed** face normal (`mesh.normal`), while overhang detection uses the transformed one. On a rotated model, tips point the wrong way.
- `place_point_supports` checks vertex `origin_vertex` but places the support at `origin.vertex`, a neighbour. It also sums untransformed normals.
- A 3 mm tip from a thin wall can exit the far side of the wall.

### Other gaps that affect the design

- **Supports are not sliced.** Nothing outside rendering reads the support mesh; `notes.md` lists "slice supports" as a TODO. Section 9 makes this milestone 1.
- **No model lift.** Supports end at z = 0 whether or not the model is raised. CHITUBOX lifts the model by `zliftheightinsupportmode` (5 mm) first.
- **No contact depth.** The tip sphere sits on the surface instead of sinking 0.3–0.6 mm into it, giving a weak, point-like joint.
- **One giant raft.** A convex hull around all feet makes one large plate even for two distant supports.
- **Settings mismatch.** `precision` and the raft values in `SupportConfig` are ignored by the mesh builder, which hard-codes 20 segments and a 1 mm raft.

## CHITUBOX pipeline overview

&#91;embedded content: CHITUBOX support pipeline · 3 stages, 16 steps\]

Read left to right: stage 1 picks contact points from one coverage budget, stage 2 routes each point under fixed angle rules, and stage 3 turns routes into printable parts. Entry point is `cbd::SupportGenerator::autoCreateSupport`, which calls `YgAddSupport::autoSupport` (stages 1 and 2) and then the geometry builders.

## Stage 1: support point determination

CHITUBOX turns the mesh into a fine XY grid of vertical ray samples, decides which downward-facing surface samples need help, and places points greedily from the lowest surface up. Each placed point marks a 3 mm patch of surface around it as covered. This coverage model is what keeps supports sparse: a point is only added where no earlier point already holds the surface. All values below are verified unless marked inferred.

### 1.1 Grid set-up (`DetermineSupPnt::init`)

| Quantity | Rule | Example: 60 × 40 mm footprint, defaults |
| --- | --- | --- |
| Cell size `c` | `sqrt(footprint_bbox_area / 200 000)`, at least 0.1 mm | sqrt(2400 / 200 000) = 0.11 mm |
| Grid size | footprint / `c`, plus 32 empty cells of padding on every side | about 545 × 364 + 64 each way |
| Spacing in cells `n` | `round(touchtipdistance / c)`, at least 4 | 4 mm / 0.11 = 36 cells |
| Coverage radius | 0.75 × `n` cells (= 0.75 × `touchtipdistance`) | 3 mm |
| Half-coverage radius | 0.375 × `n` cells | 1.5 mm |
| Greedy fill radius | 0.8 × `n` cells | 3.2 mm |
| Neighbourhood | 8-connected; diagonal steps weighted by √2 |  |

The grid always has about 200 000 cells over the model's footprint, so its resolution scales with model size. Coverage radii are stored as precomputed discs of cell offsets (`genDelsBySpeDist`).

### 1.2 Angle thresholds

From `autosupportangle` α (default 45°), CHITUBOX uses α + 0.1° and derives three numbers. `n_z` is the z component of the outward surface normal; −1 means facing straight down.

| Threshold | Formula | Value at α = 45° | Meaning |
| --- | --- | --- | --- |
| Needs support | `n_z < −cos(α)`; α below 3° is treated as 3° | −0.706 | Surface is within α of facing straight down |
| Steep | `n_z > −cos(min(α, 44°))` | −0.719 | Surface is steep enough to hold itself |
| Slope ratio | `tan(α)` | 1.004 | Used when comparing height steps between neighbouring cells (inferred) |

At α ≥ 89.9° the code switches to fixed fallbacks: the needs-support threshold becomes −0.0017, so nearly every downward-facing surface gets supported.

### 1.3 Ray intervals (`genIntervalInfo`)

1. **Cast rays.** For each grid cell centre, cast a vertical line through the mesh and collect every triangle crossing (`getGridCenterIntervals`; 0.002 mm tolerance).
2. **Build intervals.** Pair the crossings into intervals: stretches of z where the ray is inside the model (`handleInters`). The bottom of each interval is a downward-facing surface sample. It keeps its z, its triangle and that triangle's normal.
3. **Link neighbours.** Each interval is linked to the overlapping interval in each of the 8 neighbouring columns (`createNeis`). This gives a surface graph that follows the model across the grid, including undercuts.
4. **Classify.** Average each sample's normal with its linked neighbours, normalise, and set flags from the thresholds in 1.2 (`determineFlatOrSteep`).

### 1.4 Point categories, in priority order (`DetermineSupPnt::determine`)

| # | Step | What it does |
| --- | --- | --- |
| 1 | `determineSupportedByWall` | Marks samples that a nearby wall already holds. Coverage spreads 0.4 × the coverage radius from the wall, so they need no point (inferred) |
| 2 | `determineNearBound` | Handles samples on the outer edge of an overhang, where the surface meets air. Uses 0.4 and 0.99 coverage factors (inferred) |
| 3 | `determineKeyPnts` | Places points along sharp downward edges, walking each edge with a step of `n` cells (inferred from a 4.0 factor and the edge-key bookkeeping) |
| 4 | `determineLowPnts` | Islands: every vertex lower than all its neighbours (`gatherLowestPnts`), filtered so only one point is kept per 0.5–1 cell area. Each one gets a point and marks its own coverage |
| 5 | `determineSupPnts` | Fill: covers every remaining sample that still needs support (detail below) |
| 6 | `deleteNearSupPnt` | Dedupe: of two points closer than `max(0.3 × touchtipdistance, 0.6 × pillar diameter)` (1.2 mm by default), the lower-priority one is removed |
| 7 | `recordResultSupPnts` | Writes the final points, each with position, normal and category |

### 1.5 The fill loop (`determineSupPnts`)

1. **Sort.** Put candidate samples in buckets and sort each by z, lowest first.
2. **Skip.** Ignore a sample if it's already covered, already a support, or flagged as not needing one.
3. **Reject bad spots.**
   - `isBadSupPnt`: a short segment from the sample along its normal hits the model, so the tip would have no room.
   - `checkPntNearSteepSurf`: a steep sample lies within one diagonal cell (√2).
4. **Nudge.** `adjustIntervalPos` may move the point to a neighbouring sample that is no more than 0.01 mm higher and gives a better seat.
5. **Mark coverage.** Flood outward across linked neighbours, within the coverage disc. The flood follows the surface rather than plain XY distance, with a 0.05 mm height tolerance. If the point is in half-scope territory (near other coverage), the smaller 1.5 mm disc is used. The sample is flagged as a support point.
6. **Second pass.** Push samples still uncovered into a lowest-first priority queue and repeat steps 3–5. A sample needs at least 3 or 5 uncovered neighbours, depending on its flags, to qualify. This picks up patches the first pass missed.

**Why this beats mslicer's approach:** points come from one coverage budget instead of three independent detectors. Overhangs high above other geometry are reached through their own ray intervals, not by every upward ray adding a support per surface it crosses.

### 1.6 Output per point

Position, outward unit normal, category (wall, bound, key, low or fill), and a per-point contact depth. Routing uses that depth, capped at `touchDepth` (Stage 2, 2.2).

## Stage 2: routing and topology

Every support in CHITUBOX is built from three straight parts with fixed angle rules: a tip, an optional 45° link, and a vertical pillar. Each part is collision-tested before it's accepted. No part is ever free to take an arbitrary slope, which is why CHITUBOX never produces mslicer's near-horizontal arms.

### 2.1 Order of operations (`YgAddSupport::createSupportFunc`)

| # | Function | Purpose |
| --- | --- | --- |
| 1 | `genTriSupsPos_` | Triangular lattice of pillar sites. Only in CHITUBOX's "enhanced" mode; skipped in normal auto-support (verified, 8.4) |
| 2 | `reGroupTriModel_` | Groups lattice points by part. Also enhanced mode only (verified) |
| 3 | `genNearGroundRelated` | Main path: a tip plus a vertical pillar to the plate, with lateral search as fallback (2.2, 2.3) |
| 4 | `processCanVertToGround` | Retries flagged points with a pure vertical drop (`genVerticalToGround`) |
| 5 | `processIsolatedNearGrounds` | Brace network between pillars (2.5) |
| 6 | `genTriModelCandiInfos_`, `genNearFarModelRelated` | Points with no way to the plate land on the model below (2.4) |
| 7 | `genShortSpecialSup` | Short "finger" supports across small gaps between two model surfaces |
| 8 | `genRaft` (pass 0) | Raft from pillar footprints (section 7) |
| 9 | `genSupportStructure` | Builds the meshes (section 7) |
| 10 | `addStableSupsToHighMainSups` | Extra stabilising legs on tall, isolated pillars (2.6) |
| 11 | `genRaft` (pass 1) | Raft rebuilt to include the stabilising legs |

### 2.2 The tip (verified)

Given a point `p` with outward normal `d`:

1. **Clamp the direction.** If `d` is more than `maxConnectAngle` (70°) from straight down, rotate it toward down until it is exactly 70° away. In code: if `dot(d, −z) < cos 70° = 0.342`, set `d.z = −0.342` and rescale `d.xy` so `|d| = 1`. A tip is therefore never flatter than 20° below horizontal.
2. **Contact sphere.** Its inner end sits at `p − d × depth`, where depth = min(point depth, `touchDepth`). Its outer end is at `p + d × (touchDiameter − depth)`, and its centre is midway. The sphere sinks into the model by the contact depth (0.3–0.6 mm).
3. **Cone.** It runs from the sphere centre to `tipEnd = centre + d × connectLength` (2–3 mm). It tapers from `upDiameter` at the contact to `downDiameter` at the end.
4. **Clearance before trying.** Only attempted when `p.z − ground ≥ 0.6 × pillar diameter + connectLength + 0.1 mm`. Lower points get a short-pillar variant (section 7).
5. **Horn collision test** (2.7) on the cone, using a radius of 0.6 × `downDiameter`.

With touch shape "None" there is no sphere; the cone starts at `p − d × depth`.

### 2.3 Reaching the plate (`genNearGroundRelated`)

1. **Straight drop.** If the tip passes and a vertical pillar from `tipEnd` to the plate passes the main collision test, use it. A check that the pillar's midpoint isn't inside the model guards against tunnelling through closed cavities.
2. **Otherwise, search sideways.** Collect candidate pillar sites on the XY grid within `supportmaxoffsetfromcontact` (15 mm) of the contact. The search walks the 8 compass and diagonal directions (`getGroundMainCandiPos`) and also scans grid cells (`getGroundMainCandiPos2`). A site must be clear of the model's footprint. Candidates are sorted nearest first.
3. **For each candidate at XY offset `o`, try tip directions in order.**
   - If `o` points away from the normal's horizontal part: straight down, then 45° toward the site.
   - Otherwise: the clamped normal, then 45° toward the site, then about 27° below horizontal toward the site, then straight down.
4. **For each direction:**
   1. Build the tip and run the horn test.
   2. Link from `tipEnd` to the point above the site, descending exactly as much as it travels sideways. The link is always a 45° diagonal; run the link collision test on it.
   3. Drop the pillar from the link end to the plate; run the main collision test.
   4. The first combination that passes everything wins.
5. **If nothing passes**, the point goes on to model landing (2.4).

### 2.4 Landing on the model (`genNearFarModelRelated`)

Points with no way to the plate drop a pillar onto the model below and end in a short tail cone. All rules below are verified unless marked.

1. **Contact depth.** min(touch depth, model thickness above the contact). Thickness is probed along +z over at most 2 mm, to the farthest exit face.
2. **Tip.** Try the surface normal, then normalise(normal + down), then straight down. The tip end must be in free space and pass the horn test.
3. **Drop.** Run a vertical main test from the tip end, radius = pillar radius + `supportspacingfrommodel`. The first hit must lie on the same model, unless support-in-shell is on. The drop must be ≥ max(`supportmaxsmallpillarlength`, 2 × cone length + 0.1 mm), i.e. 4.1 mm by default. Landing floor = hit z − pillar Ø − 0.01 mm.
4. **Single tail (contact-point value 1).** Gather upward-facing surface samples (n\_z ≥ 0) at or above the floor, near the tip end's XY. Each needs a clear straight line to a point 0.05 mm above it. Score each by the cosine between its normal and the tail direction, and keep the best 3. Take the first whose vertical pillar passes the main test and whose tail passes the horn test. The tail leans at most about 2 × small-pillar Ø sideways.
5. **Tail.** The tip, mirrored. It sinks `supporttailtouchdepth` along its axis, and has a sphere of `supporttouchtaildiameter` only if the tail shape is not None. Its horn test uses the tip cone's diameters.
6. **Claw foot (fallback).** Foot top = min(floor + 1.5 × cone length, tip end − 0.1 mm); after a failed single tail the factor is 0.8. From the foot top, cast 8 compass probes. Each probe is 2.2 × D² sideways and 6.6 × D² down, with D = pillar Ø in mm; diagonals are not normalised. That is about 18° off vertical, 25° on diagonals. Each hit becomes a finger, a tail from the hit to the foot top, horn-tested at 0.6 × cone bottom Ø. The finger count is `supporttouchtailpoint` (1–3): for 1, the finger best aligned with its surface normal; for 2, an opposite pair if possible; for 3, a spread triple. Accept with ≥ 1 finger and a foot ≥ 0.8 × cone length below the tip end.
7. **Small pillar (slim).** For short model-to-model gaps: slim length 2 × slim Ø, drop ≥ 4 × slim Ø, length ≤ `supportmaxsmallpillarlength`, cross test at 0.6 × slim Ø.
8. **Bracing.** Delaunay edges shorter than 1.3 × R (R = 0.5 × `touchtipdistance`), with the 20° rule from 2.5, then a minimum spanning tree (inferred). Slim pairs closer than 4 × slim Ø are joined the same way.

The contact-point option reaches the code as its label (1, 2 or 3), so the default is 1: step 4 runs first and the claw is the fallback (verified, with the wrapper offset inferred from the config classes).

### 2.5 Brace network (`processIsolatedNearGrounds`)

1. **Pre-merge.** Off with default settings (verified). The scope merge only runs when 0.5 × `touchtipdistance` × `modelbottomsupportdensity`/100 ≥ `touchtipdistance`, i.e. bottom density ≥ 200%. The lattice merge needs enhanced mode. Rules when active are in 2.8.
2. **Inverted-V forks.** Chain nearly collinear plate pillars: next pillar within 9 × pillar Ø, direction within 45° of the last step. For three in a row A, B, C, each gap ≤ 4 × pillar Ø, put a fork point F on B at B's top − (2 × pillar Ø + 0.3 × cone length). F must be above the floor: ground + pillar Ø + 0.1 mm + base or raft thickness + `supportconnminheight` (5.3 mm for Middle). B's tip end may sit at most 2.5 × (cone length + pillar Ø) above F. Two 45° links run from F down to A and C and must pass the link test. B then stops at F and is carried by A and C instead of reaching the plate (verified).
3. **Spanning tree.** Delaunay over the pillar positions, then a minimum spanning tree. Brace every tree edge no longer than `maximalsupportcrossspacingxy` (30 mm) (verified). A 1.3 × R (2.6 mm) edge filter exists, but for plate pillars only enhanced mode runs it (verified).
4. **Brace each kept edge** with `connectTwoMainSupsCommon_`:
   - Braces are 45° diagonals between the two pillars, each dropping as much as the pillars are apart.
   - The first brace starts 0.15 mm below the lower pillar top.
   - Braces repeat downward with a vertical step of `max(minimalsupportcrossspacingz, 13.5 × crossDiameter − 2 × distance)`. With defaults that's at least 2 mm.
   - They stop at `supportconnminheight` (3 mm) plus the base height above the plate.
   - Each brace is link-collision tested; a failed brace is skipped.

     crossDiameter here is the pillar Ø: `supportcrossdiameter` never reaches the algorithm (8.4), so braces are as thick as pillars (verified).
5. **Model-landing and slim pillars.** Model-landing pillars are braced along Delaunay edges shorter than 1.3 × R = 2.6 mm (verified: this pass does run in auto mode). They also get inverted-V forks with a 6 × Ø chain radius (inferred). Slim pairs closer than 4 × slim Ø are joined via a spanning tree (`processIsolatedNearModels`, `processIsolatedSlims_`).
6. **Avoid near-parallel braces.** An edge is dropped if a brace already leaves either pillar within 20° of the edge's direction: dot > 0 and |cross| < 0.342 = sin 20° (verified).

Bracing is controlled by `supportcrossenable` (on by default).

### 2.6 Stabilising legs (`addStableSupsToHighMainSups`)

At most one diagonal leg per tall, sparsely braced pillar (verified). Two switches gate it, both on by default: `autoSupport` calls the leg pass only when `supportEnhancedAutomaticEnable` is on (SupportPara 0x1e4, copied to YgAddSupport+0x2dc), and the pass itself returns at once unless `supportcrossenable` is on (0x1e2). Verified. This 0x1e4 switch is not the always-off enhanced flag at 0x2b0.

- **Candidates.** Vertical pillar segments. H = segment top − bottom. A "long link" is a tip-to-pillar link with more than 5 mm horizontal run.
- **Trigger.** H ≥ 30 mm, or a long link and H ≥ 20 mm. Also, some gap between neighbouring brace or link joints on the pillar must exceed 30 mm, unless there is a long link.
- **Start.** 2 mm below the pillar top, on the pillar surface.
- **Direction.** Horizontal d, tilted 45° down. The first d that routes wins:
  - No braces: opposite of the long link, then the 8 compass directions in 45° steps.
  - One brace u: ⊥u, −⊥u, the diagonals away from u, −u, then the diagonals toward u.
  - Two or more braces: the bisector of the largest free gap. Gaps above about 148° also get the two quarter directions. Up to 5 gaps.
- **Length.** The leg's cone is 0.6 × R for plate pillars, 0.3 × R otherwise, with R = 0.5 × `touchtipdistance`. That is 1.2 mm / 0.6 mm.
- **Routing.** As a manual support with the usual horn, link and main tests. A plate leg is then braced back to its pillar (inferred). For pillars that end in a link, only directions with model below are tried, closest surface first.
- The constants 50, 0.8, 0.4 and 1.414 belong to an enhanced-mode branch that auto-support never runs.

### 2.7 Collision tests

All tests intersect line segments with model triangles, after a bounding-box prefilter, with a 0.002 mm tolerance. Each has an on/off flag in settings (`mainCollisionDetection`, `hornCollisionDetection`, `connCollisionDetection`, `crossCollisionDetection`), all on by default.

| Test | Shape tested | How |
| --- | --- | --- |
| Horn | Cone from sphere centre to tip end | 8 lines at 45° steps around the axis, no centre line. They start at r × top Ø/bottom Ø and end at r, with r = 0.6 × cone bottom Ø. Each start is moved toward the tip by touch depth + max(0.5 × pillar Ø, 0.3 × cone length), 1.0 mm for Middle. Skipped if the cone is shorter than 0.5 × pillar Ø (verified) |
| Main | Vertical pillar | XY box grown 1.3× around the pillar, then vertical lines at the centre and at 0.5 and 1.0 of the radius. Radius = part radius + supportspacingfrommodel (verified; legs use part radius + min(max(Ø, 0.8), spacing)) |
| Link | 45° link or brace | Same multi-line approach as horn, along the link |
| Vertical | Pure vertical drop | Exact vertical-segment-versus-triangle test |
| Cross | Short finger supports | Segment test with 0.5 offsets |

**Fix for "tips through geometry":** mslicer must test every segment it creates, tip and link included, not only the final drop. Use offset lines at the part's radius, not just its centre line, so thin walls can't slip between samples.

### 2.8 Pre-merge rules (inactive by default)

These rules exist in the code but do not run with default settings (2.5). They are listed so mslicer can skip them knowingly. All verified. R = 0.5 × `touchtipdistance`, D = pillar Ø.

- **Order.** Pillars sorted by tip-end z, lowest first.
- **Grouping.** Neighbours within 0.4 × R in XY. The z tolerance is min(max(R, 0.7 × `maximalsupportcrossspacingxy`), 15 mm) in the first pass and R in the second. Leftovers join the nearest group within 0.7 × R.
- **Merge point.** At the member nearest the group centre, at height min over members of (member z − XY distance). So every member links down at 45°. Lowered further by `distforktohornend`.
- **Cancel** if the merge point is below ground + max(2 × D + 5 mm, base height + 2 × D).
- **Links** are tested with the link test at 1.2 × pillar radius. Members that fail keep their own pillar.
- **Lattice attach** (enhanced only): spare supports join a lattice pillar within min(`supportmaxoffsetfromcontact`, 15 mm), choosing the highest 45° link point above ground + D.

## Stage 3: geometry

Each support is assembled from a fixed catalogue of parts, stacked from the plate up. A support records its parts as typed segments (start, end, part kind), and meshes are generated per part. That keeps editing, slicing and the raft independent of how the route was found.

### 3.1 Part catalogue (`GenerateSupport::genOneSupport`)

| Part | CHITUBOX builder | Shape | Size from settings (Light / Middle / Heavy) |
| --- | --- | --- | --- |
| Contact sphere (top) | `genUpWardTouch` | Sphere, sunk into the model; off by default on this printer (shape None), so the cone itself sinks in | Ø `supporttouchtipdiameter` 0.5 / 0.8 / 1.0, depth 0.3 / 0.4 / 0.6 |
| Tip cone ("upward horn") | `genUpWardHorn` | Cone, Pyramid or Skate | Ø 0.3 → 0.8 / 0.4 → 1.2 / 0.6 → 1.5, length 2 / 2 / 3 |
| Joint | `genJoint` | Sphere at tip end and link ends | Pillar diameter (inferred) |
| Link / main pillar | `genMain` | Cylinder, Prism or Cube | Ø 0.8 / 1.2 / 1.5 |
| Lower main pillar | `genMain` | Same shape, second diameter | Same as the pillar: the GUI copies `supportmiddlediameter` into the second-diameter field. `supportmiddlediameter2` (1.0 / 2.0 / 2.5) never reaches the algorithm (verified) |
| Slim (small pillar) | `genSlim` | Sphere- or cone-ended thin rod for short model-to-model gaps | Ø 0.4 / 0.5 / 0.5, at most `supportmaxsmallpillarlength` 3 mm |
| Boss | `genStable` | Cone from pillar to base | Height `heightofboss` 1.0, bottom Ø `diameterofbossbottom` 2.2 |
| Base | `genBottom` | Skate, Cone, Cube, Cylinder or Prism disc | Ø 10 / 12 / 12, thickness 0.8 / 1.0 / 1.0 |
| Tail (model landing) | `genDownwardTouch` + `genDownwardHorn` | Downward cone into a lower model surface (tail sphere off by default) | Ø 0.4 / 0.6 / 2.0, depth 0.2 / 0.2 / 0.3 |
| Brace | `genMainConnSupStruct` | 45° cylinder between pillars | Ø = pillar Ø (`supportcrossdiameter` is not passed; verified) |

### 3.2 Pillar assembly cases (`HandleGenMainSup`, verified dispatch)

The pillar from tip end (or link end) to its foot uses one of seven cases.

"Raft present" means the raft's outline bounding box, set by the raft pass, is non-empty (verified). In cases 1 and 4 the pillar stands on the raft with no base. Its foot sits at raft thickness − 0.002 mm when the foot (radius = half the second diameter) lies inside the raft outline, or when the raft shape is "Bottom connected"; otherwise on the plate (verified). The 5 mm threshold is a fixed constant.

| Foot | Pillar length + 0.6 × tip length | Base shape | Case |
| --- | --- | --- | --- |
| On the model | any | n/a | 7: tail into the lower surface |
| On the plate | < 5 mm (short) | raft present (verified) | 1 |
| On the plate | < 5 mm | "None" (verified) | 3: no base or boss |
| On the plate | < 5 mm | any other | 2: base plus short pillar |
| On the plate | ≥ 5 mm (long) | raft present (verified) | 4 |
| On the plate | ≥ 5 mm | "None" (verified) | 6 |
| On the plate | ≥ 5 mm | any other | 5: base, then boss, then pillar to the tip end |

Case 5 is the common one, stacked bottom-up:

1. **Base** from z = 0 to the base thickness.
2. **Boss** from there up by `heightofboss`.
3. **Main pillar** from the boss top to the tip end.

The boss/pillar merge threshold reads `triggerheightofLowmidsupdiam` (5 mm by default), not a fixed 1 mm (verified reader; exact rule not traced). Every stacked part is shortened by 0.002 mm at the joint so faces don't coincide.

### 3.3 Raft (`SupportRaft::genRaft`)

1. **Footprint.** Project the support footprints onto the plate (`ModelProjGrid::doProj`). Optionally the model is included where it sits low.
2. **Outline.** Union the footprints with Clipper and offset them outward. Scale by `raftarearatio` (110%) and remove tiny segments (< 0.01 mm). Separate support clusters give separate raft islands, unlike mslicer's single convex hull.
3. **Build per shape.**
   - **Skate:** a solid plate `raftthickness` (1.0 mm) thick. Its outer wall slopes at `raftangle` (30°) up to `raftheight` (1.8 mm), the raised anti-peel lip (`genExtraUpwarp`).
   - **Cross grid:** a lattice fill (`FillInnerCross`) with cell polygon length and bar width of 2 mm (`raftgridpolylen`, `raftgridwidth`).
4. **Rebuild.** The raft is built twice: once after routing, and again after stabilising legs are added, so it includes them.

Defaults for the Saturn 4 Ultra 16K: no raft for Light and Middle, Skate for Heavy.

### 3.4 Model lift

In support mode the model is lifted by `zliftheightinsupportmode` (5 mm), and the plate sits at z = 0 (`getZCoordGround` returns 0). mslicer must apply the same lift before routing, otherwise the short-pillar cases dominate and bases intersect the model.

## Manual tools and editions

The paid tiers add no new auto-support algorithm that we can find. Basic 2.3.1's kernel (`libCbdKernel`) already ships region auto-support, two-click directional supports, upward tips from a joint, and merging pillars into trees. None of them checks the edition. Each one reuses Stage 1–3 code, so mslicer can offer them for little extra work after M5.

Sources: `libCbdKernel.so.1` decompiled with the extra filter seeds `EditAddUpwardSupportOp::`, `MergeSupports(Op|Job)?::`, `SupportManager::(mouse|area|select|merge…)` and `SupportGroup::onAdd`, plus `libCbdModel.so.1` (`out-d2`) and the main executable's symbols and strings. Verified or inferred is marked per claim, as elsewhere.

### 4.1 Edition differences found in the code

The one edition difference in the support code is a second, unused settings schema for Pro. Basic and Pro would run the same algorithm.

- **No edition branch.** Nothing in libCbdModel's support code branches on the edition. In the main executable, only a start-up routine (`SoftwareControl::softwareStartReady`) reads `softwareType`, `softwareVersionType` or `authorizationFlag`. `SupportConnect` and `ProjectSupportGenerator` make no licence calls (verified).
- **Two JSON schemas.** libCbdModel has `cbd::VersionBasic::SupportSettings` and `cbd::VersionPro::SupportSettings`, each with a JSON reader, a writer and an `updataSetting` that copies into SupportPara. Nothing in the executable or the other libraries constructs either one (verified), which fits 8.4: the GUI fills SupportPara itself.
- **Same flags.** Pro's `supportEnhancedAutomaticEnable` writes SupportPara 0x1e4 (the leg switch, 2.6), not the always-off 0x2b0 flag. So even the Pro schema cannot turn on the lattice, lattice merge or enhanced-leg branch in this library (verified).
- **In-shell forced off.** At start-up the Basic GUI calls the in-shell setter with a constant `false`, though the profiles ship `supportInShellEnable:1` (verified). A later change in the panel may still go through; not checked.
- **Basic-named GUI classes.** `AddSupportTypeBasic` and `SupportProfileConfigQtBasic` suggest the paid builds have fuller versions (inferred).

What each schema's `updataSetting` writes into SupportPara (verified):

| SupportPara field | Basic schema | Pro schema |
| --- | --- | --- |
| 0xa0 / 0xa8 / 0x218 pillar Ø, middle Ø 2, brace Ø | all three = pillar Ø | separate keys; brace from `supportCrossDiameter` |
| 0x248 R (lattice spacing, merge, legs) | 0.5 × touch-tip distance | `middleSupportDistance` directly |
| 0x1f0 max brace spacing XY | from `supportCrossWidth` | `maximalSupportCrossSpacingXY` |
| 0x1e2 / 0x1e3 / 0x1e4 cross, in-shell, enhanced | left at defaults | `supportCrossEnable`, `supportInShellEnable`, `supportEnhancedAutomaticEnable` |
| 0x1e8 min brace spacing Z | default | `minimalSupportCrossSpacingZ` |
| 0x200 / 0x208 / 0x210 spacing from model, max slim length, max offset from contact | defaults | own keys |
| 0x220 / 0x240 boss height, boss bottom Ø | none | `heightOfBoss`, `diameterOfBossBottom` |
| 0x230 low-mid Ø trigger height | default | `triggerHeightOfLowMidSupDiam` |
| 0x198 / 0x1a0 raft grid | none | `gridPolyLen`, `gridWidth`; shapes "Cross grid", "Hexagonal grid", "Bottom connected" |
| 0x238 bottom density | forced to 15 | forced to 100 |
| 0x150 contact point | copied | stored value + 1 (0-based in the file) |

The Basic GUI (8.4) wires most of the Pro-only rows itself. The rows that matter are the two it never connects: brace Ø and R. Pro probably lets users set both (inferred). For mslicer: expose `brace.diameter` and R, defaulting to Basic's values (pillar Ø and 0.5 × `general.point_spacing`).

The vendor's feature list, mapped to the code (all inferred; we only have Basic):

| Claimed feature (tier) | What Basic 2.3.1's code shows |
| --- | --- |
| More advanced support parameters (Advanced) | The Pro schema above: brace Ø, R, enhanced switch, grid rafts |
| Advanced tools for support placement and editing (Advanced) | Directional support, upward tips, merge, per-part handles (4.2–4.7). All the code ships in Basic's kernel |
| One-click adaptive Magic Support (Advanced) | No trace; no "magic" string relates to supports (`MagicInfo`/`MagicStep` belong to the printer file format). Possibly the 0x2b0 enhanced path, which exists but is never enabled, or code that ships only in Advanced |
| Batch support placement via freehand region support (Pro) | Basic has rectangle region auto-support (4.3). A freehand outline would only change how the point set is picked |

To settle the open rows, check a Pro build for writes to the 0x2b0 flag and for the strings `Cross grid` and `supportEnhancedAutomaticEnable` in its GUI panel.

### 4.2 Tool inventory

Each model's `SupportGroup` builds a `SupportOpManager` that creates five tool objects unconditionally, keyed by a `SupportOpName` number. `SupportManager`'s mouse handlers forward events to the active one (verified).

| # | Tool (class) | What it does | Driven by | In Basic's GUI? |
| --- | --- | --- | --- | --- |
| 1 | `AddTreeSupportOp` | Click to add one routed support (Stage 2 hand-add path, `addOneSupportByHand`) | press, hover | Yes: `support@add_mode` |
| 0 | `AddDirSupportOp` | Two-click straight support from a model point to a free point or onto a support (4.4) | press, hover | Not found: no menu action; reachability not traced |
| 2 | `EditAddUpwardSupportOp` | Double-click a joint, then add tips from it up to the model (4.5) | double-click, hover, press | Not found: the word "upward" never appears in the executable, but appears 88 times in the kernel |
| 3 | `EditSupportOp` | Drag parts with axis handles (4.7) | press, drag, release | Yes: `support@edit_mode` |
| 4 | `EditUiChangeSupportOp` | Apply panel edits to the selected supports (4.7) | panel | Yes |

Three more tools sit outside the manager:

- `MergeSupportsOp` / `MergeSupportsJob`: an undoable job that merges pillars into trees (4.6). The executable has a `mergesupport` string (verified); where the button lives was not traced.
- `SupportManager::areaSelectAutoAddSupport` with `SelectArea::selectRectAreaAddSupportPnt`: region auto-support (4.3).
- Dangling-area navigation: `support@last_dangling_area` / `support@next_dangling_area`, "Detect the lowest hanging region of the selected model" (verified strings).

### 4.3 Region auto-support (`YgAddSupport::areaAddSupport`)

Region auto-support is Stage 1 run on a subset of its own candidate points, with existing supports counted as coverage. Then Stages 2–3 run unchanged (verified unless marked).

**Input.** `SupportManager::areaSelectAutoAddSupport` takes a map from model to a set of integer point indices. The indices number Stage 1's candidates: first the grid samples (one per cell and interval, 1.1), then the vertex points (1.4) at index − grid count. The GUI builds the set by screen selection: `SelectArea::selectRectAreaAddSupportPnt` takes a `QImage`, most likely an ID buffer of the rendered candidate points under a rectangle (inferred).

**Per model:**

1. Progress bar, clear the live-preview supports, then `autoSupportInit` (the same set-up as auto-support).
2. Clear the grid's effect cells. Mark every selected sample with a "selected" bit, clearing two other state bits on it.
3. Collect the selected vertex points and sort them by z.
4. Seed coverage from existing supports. Every live tip (type 1) or slim tip (type 12) on the model is located in the grid via its contact point, nudged 0.01 mm down. Its cell comes from the grid origin and step when the support doesn't store one. Tips on selected samples, or on a matching vertex point (found by binary search on z), are collected with `casSelectSupport`. `expandFromOuterIntervalBySupportEffect_` then marks the area they already cover, so new points keep the usual spacing from them.
5. Rerun Stage 1 on the selected points: `clear`, `init`, `determineSupPnts`, `deleteNearSupPnt`, `recordResultSupPnts`. The vertex-point pass runs only if step 3 found any.
6. `adjustLowEdgeSupportPnt` (factor 0.5), then build each result point with `createSupportFunc`, the normal Stage 2–3 path. Points already flagged as done are skipped. Each point keeps its Stage 1 category.
7. Clear the "selected" bits again.

The kernel wraps the new supports in one undo record (`SupportOpIntance`) and refreshes which model each support touches.

Not settled: whether colliding existing supports collected in step 4 are deleted or only kept as coverage, and what the call's last boolean does. It switches one branch for points whose normal z is ≤ −0.5.

**For mslicer.** Run 1.1–1.5 as usual. Keep only candidates inside a selection mask (rectangle, lasso or brush in screen space, or a face set). Pre-fill the coverage grid from existing contact points. This is also the "freehand region support" of the Pro tier: only the mask shape differs.

### 4.4 Directional support (`AddDirSupportOp`)

A directional support is one straight segment from a first click to a second point the user picks. Nothing routes it. It is how users bridge from the model to an existing support, or set a tip at an angle the auto path would not choose (verified unless marked).

**First click** (`pressModelStart` / `pressSupportStart`) records the start point. Two tool flags then set what gets built: "slim" and "started on a support". Which click sets which flag is inferred from the handler names.

| Mode | Segments built, start → end |
| --- | --- |
| Normal (start on model) | One tip cone (type 1) |
| Slim | One slim segment (type 12), plus a joint sphere (type 9) at the end spanning ± 0.5 × SupportPara 0xd8 in z |
| Started on a support | One pillar segment (type 3), radius 0.5 × SupportPara 0x70 at one end, 0.5 × pillar Ø at the other |

**End point while hovering** (`genRealTimeDirSupHover`). The preview is rebuilt on every mouse move, with end radius 0.5 × SupportPara 0x70 by default.

- Over nothing: the point on the cursor ray closest to the start.
- Over a vertical pillar (XY ends within 0.002 mm): its axis at the cursor height, clamped to the pillar's z range.
- Over a link that passes `checkConnSupConnHorn`, unless its ends are within 0.3 mm of each other: the nearest point on the segment. Within 0.1 mm of an end, it moves 0.15 mm inward along the segment. The end radius is interpolated linearly along the segment.
- Over a tip: only in normal mode. Over a slim segment: only in slim mode. Over a base (types 5/6): only in slim mode, and only above the current segment's lower end.

**Second click** (`pressModelEnd` / `pressSupportEnd`) commits the preview as a support.

**For mslicer.** A two-click tool that builds a 2.2 tip or a 3.1 pillar part between two points. It snaps to pillar axes and link segments with the rules above, and runs the 2.7 collision tests for a warning colour only.

### 4.5 Upward tips from a joint (`EditAddUpwardSupportOp`)

The upward tool grows extra tip cones from an existing joint up to the model. That catches a small overhang just above a pillar without adding a new pillar (verified unless marked).

1. **Pick the joint.** Double-click a support. On a tip (type 1), the joint at its lower end is taken (`getJointSupport`). On a joint sphere (type 9), its centre must match, within 0.004 mm, either a tip's start or the upper end of a pillar segment (type 3). Anything else is ignored. Picking closes the move handles.
2. **Aim.** On each hover, the target is the model surface point under the cursor, snapped to the nearest support point with its normal (`getNearestModelSupPntNorm`). Off the model, it is the point on the cursor ray closest to the joint.
3. **Build the preview** (`genMultiUpwardHornHover`). d = normalize(joint − target).
   - If the top contact shape compares equal to a fixed string, the cone simply ends at target − d × L. The string is probably "None" (inferred).
   - Otherwise `genOneTouchInHornTip` builds a contact sphere (type 7) and the tip, from the top settings' Ø and length L (offsets 0x38 and 0x40 of the top settings record).
   - The tip (type 1) runs from that contact point down to the joint.
4. **Commit.** A click adds the previewed tip. The tool stays on the same joint, so several tips can fan out from it ("multi"). Another double-click ends the session.

No collision test runs on these tips in the code read (inferred from the call list: no `*CollisionDetection` calls).

**For mslicer.** Reuse the 2.2 tip builder with the lower end fixed at the joint. Run the horn test (2.7) and refuse targets above the 2.2 tilt clamp (`general.max_tip_angle`, 70° from vertical), which CHITUBOX does not seem to enforce here.

### 4.6 Merge supports (`MergeSupportsJob` → `SupportGroup::mergeSupport` → `YgAddSupport::postMergeMidSups`)

Merge takes pillars that already exist and joins neighbours into trees, using the same rules as the auto pre-merge (2.8). Constants are verified; the reading of the selection logic is partly inferred.

1. **Job.** `MergeSupportsOp` is an undoable operation (a `RedoUndoBase`) that starts `MergeSupportsJob` as a background task. The job runs `mergeSupport` on every selected model that has supports. It first checks a SupportPara value against 2; which field that is was not traced.
2. **Candidates.** All supports of the model, or only the selected ones if any are selected (those get deselected). Only vertical pillar segments count (`checkVertMainSup`, XY ends within 0.004 mm). Near-cylinders thinner than 0.31 mm radius (end radii within 0.01 mm) are skipped: those are slim supports.
3. **Grouping.**
   - Pillars whose foot coincides with another candidate's foot (tolerance from a group field) form one list.
   - Pillars whose foot is a joint centre are dropped.
   - The rest form a second list. In it, a pillar is flagged as stacked when another segment's end lies within 0.003 mm of its axis in XY and inside its z range ± 0.003 mm.
4. **Merge** (`postMergeMidSups`, once per list, each pillar flagged so it is not merged twice):
   - Neighbours are found with a quadtree in a box grown by 0.4 × R around each pillar; R = SupportPara 0x248, 2.0 mm in Basic.
   - Candidates join a group when within 0.6 × R and their tip-end z differs by less than 0.1 mm.
   - Merge height: the member's tip end minus `distforktohornend` (0x228). Only half that is taken off when the pillar top coincides with the tip end (0.003 mm). The result is capped by the lowest existing member.
   - In the mode with the third argument on, the joint goes 0.1 mm above the pillar's foot if the pillar is at least 0.2 mm long, else at its mid-height.
   - Floor: ground + the tallest base (type 5) bounding box + max(pillar Ø, current offset).
   - Every new link passes `connCollisionDetection` at 1.2 × the pillar radius. Joint spheres get 1.04 × the pillar radius. Parts are rebuilt with `GenerateSupport::genOneSupport` (3.1).
5. The new supports replace the old ones in the group.

**For mslicer.** Expose the M4 pre-merge as a "merge selected" command over existing pillars. It needs no new geometry code, only the selection filter and grouping above.

### 4.7 Editing, and what mslicer should build first

**Drag editing** (`EditSupportOp`). There is one preprocess and one per-frame update handler per part type: tip, tail, pillar, link ("conn main"), base, raft, slim, slim joint, claw joint, upper pillar joint, stabilising leg, upward tip and its joint. A downward view of the upward tip has its own handler. Editing a claw regenerates its fingers (`genClawSups`). `adjustPickedSupport` and `getAttachToSup` re-attach a dragged part to the support it lands on. Edits run in model space, along axis handles (`setAxis`, `getMoveDirInModelSpace`). Verified from symbol names only; the bodies were not decompiled.

**Panel edits** (`EditUiChangeSupportOp`). `updataTop`, `updataMiddle` and `updataBottom` apply a changed top, middle or bottom setting to the selected supports. `colGroundMainRelSups` collects every support tied to a ground pillar, so a change rebuilds the whole tree (inferred from names).

**Dangling areas.** Previous/next buttons step through "the lowest hanging region of the selected model" and mark it. This uses the same overhang detection as Stage 1 (inferred).

Suggested order for mslicer, after M5:

| Tool | Reuses | New code | Value |
| --- | --- | --- | --- |
| Region auto-support (4.3) | Stage 1–3 unchanged | Selection mask; coverage seeded from existing contacts | High: the paid "freehand region" feature |
| Merge selected (4.6) | M4 pre-merge | Selection filter and grouping | Medium |
| Directional support (4.4) | 2.2 tip, 3.1 parts, 2.7 tests | Two-click tool with snapping | Medium |
| Upward tips from a joint (4.5) | 2.2 tip builder | Joint picking, fan-out preview | Medium for fine overhangs |
| Per-part drag editing (4.7) | 3.1 parts | Handles per part type, re-attach on drop | High effort; can come last |

## Parameters

mslicer should replace `SupportConfig` with a preset-based config that mirrors CHITUBOX's four groups (top, middle, bottom, raft) plus general settings. It should ship CHITUBOX's Light, Middle and Heavy values for the Saturn 4 Ultra 16K as defaults. Values come from `ELEGOO Saturn 4 Ultra 16K.cfgx`; lengths are in mm.

### 8.1 Preset-dependent values

| Proposed mslicer field | CHITUBOX key | Light | Middle | Heavy |
| --- | --- | --- | --- | --- |
| `top.contact_shape` | `supporttouchtipshape` | None | None | None |
| `top.contact_diameter` | `supporttouchtipdiameter` | 0.5 | 0.8 | 1.0 |
| `top.contact_depth` | `supporttiptouchdepth` | 0.3 | 0.4 | 0.6 |
| `top.cone_shape` | `supporttipshape` | Cone | Cone | Cone |
| `top.cone_top_diameter` | `supporttipupdiameter` | 0.3 | 0.4 | 0.6 |
| `top.cone_bottom_diameter` | `supporttipdowndiameter` | 0.8 | 1.2 | 1.5 |
| `top.cone_length` | `supporttiplength` | 2.0 | 2.0 | 3.0 |
| `middle.shape` | `supportmiddleshape` | Cylinder | Cylinder | Cylinder |
| `middle.diameter` | `supportmiddlediameter` | 0.8 | 1.2 | 1.5 |
| `middle.diameter_lower` | `supportmiddlediameter2` | 1.0 | 2.0 | 2.5 |
| `middle.slim_diameter` | `supportmiddlesmallpillardiameter` | 0.4 | 0.5 | 0.5 |
| `middle.slim_contact_depth_top` | `supportmiddlesmallpillaruppercontactdepth` | 0.25 | 0.3 | 0.3 |
| `middle.slim_contact_depth_bottom` | `supportmiddlesmallpillarlowercontactdepth` | 0.25 | 0.3 | 0.3 |
| `bottom.shape` | `supportbottomshape` | Skate | Skate | Skate |
| `bottom.diameter` | `supportbottomdiameter` | 10 | 12 | 12 |
| `bottom.thickness` | `supportbottomthickness` | 0.8 | 1.0 | 1.0 |
| `bottom.tail_diameter` | `supporttouchtaildiameter` | 0.4 | 0.6 | 2.0 |
| `bottom.tail_depth` | `supporttailtouchdepth` | 0.2 | 0.2 | 0.3 |
| `raft.shape` | `raftshape` | None | None | Skate |

The shape enums are `Cone / Pyramid / Skate` (tip), `Prism / Cube / Cylinder` (pillar), `Skate / Cone / Cube / Cylinder / Prism / None` (base) and `None / Skate / Cross grid` (raft).

The contact shape options are `Sphere / None`, and the stored index 1 means None. The generator still sinks the cone by the contact depth in that case (Stage 2, 2.2).

`middle.diameter_lower` has no effect in CHITUBOX: the GUI fills both pillar diameter fields from `supportmiddlediameter` (verified, 8.4). mslicer can keep the field but should default it to `middle.diameter`.

### 8.2 Values shared by all presets

| Proposed mslicer field | CHITUBOX key | Default | Used in |
| --- | --- | --- | --- |
| `general.overhang_angle` | `autosupportangle` | 45° | Stage 1, 1.2 |
| `general.density` | `supportdensity` | 50% | Mesh detail only: any value ≥ −1 selects the coarse cylinder template. Not a spacing factor (verified) |
| `general.bottom_density` | `modelbottomsupportdensity` | 100% | Pre-merge gate only (2.5); 100% keeps it off (verified) |
| `general.point_spacing` | `touchtipdistance` | 4.0 | Grid `n`, coverage = 0.75 × spacing |
| `general.max_tip_angle` | `supportmiddlemaxconnectangle` | 70° from vertical | Tip clamp |
| `general.model_lift` | `zliftheightinsupportmode` | 5.0 | Stage 3, 3.4 |
| `general.max_lateral_offset` | `supportmaxoffsetfromcontact` | 15.0 | Sideways pillar search |
| `general.clearance_from_model` | `supportspacingfrommodel` | 0.6 | Collision margin |
| `general.margin_from_edge` | `supportmarginfromedge` | 0.5 | Edge points |
| `general.min_support_length` | `supportlengthmin` | 5.0 | Short-pillar trigger (with `triggerheightofLowmidsupdiam` 5.0) |
| `general.max_slim_length` | `supportmaxsmallpillarlength` | 3.0 | Slim supports |
| `general.pillar_spacing` | `middlesupportdistance` | 8.0 | Not passed to the algorithm (verified) |
| `brace.enabled` | `supportcrossenable` | on | Stage 2, 2.5 |
| `legs.enabled` | `supportEnhancedAutomaticEnable` | on | Stage 2, 2.6; legs also need brace.enabled |
| `brace.diameter` | `supportcrossdiameter` | 0.8 | Not passed; CHITUBOX uses the pillar Ø for braces (verified) |
| `brace.min_z_spacing` | `minimalsupportcrossspacingz` | 2.0 | Brace step floor |
| `brace.max_xy_spacing` | `maximalsupportcrossspacingxy` | 30.0 | Longest brace |
| `brace.min_height` | `supportconnminheight` | 3.0 | Lowest brace above the base |
| `brace.width` | `supportcrosswidth` | 4.0 | Not passed to the algorithm (verified) |
| `boss.height` | `heightofboss` | 1.0 | Stage 3, 3.2 |
| `boss.bottom_diameter` | `diameterofbossbottom` | 2.2 | Stage 3, 3.2 |
| `raft.area_ratio` | `raftarearatio` | 110% | Raft outline |
| `raft.thickness` | `raftthickness` | 1.0 | Raft plate |
| `raft.height` | `raftheight` | 1.8 | Raft lip |
| `raft.angle` | `raftangle` | 30° | Raft edge |
| `raft.grid_cell` / `raft.grid_width` | `raftgridpolylen` / `raftgridwidth` | 2.0 / 2.0 | Cross-grid raft |
| `collision.main/horn/link/cross` | `mainCollisionDetection` … | all on | Stage 2, 2.7 |
| `general.hidden_angle` | `supporthiddenangle` | 45° | Display only, not passed to the algorithm (inferred) |
| `general.adsorption_radius` | `radiusofadsorption` | 8.0 | Not passed to the algorithm (verified) |
| `general.avoid_holes` | `autoSupportAvoidHoles` | off | Not used by this design |

These CHITUBOX settings are UI-only and need no mslicer field: `bshowsupportguideline`, `mbrealtimeaddsup` and `enhancedAutoSupportMode`. `supportEnhancedAutomaticEnable` is not UI-only: it gates the stabilising legs (2.6), so mslicer maps it to `legs.enabled`.

### 8.3 Internal constants (not user-facing)

| Constant | Value |
| --- | --- |
| Grid cells over the footprint | 200 000, minimum cell 0.1 mm |
| Coverage / half / fill radius | 0.75 / 0.375 / 0.8 × `point_spacing` |
| Dedupe radius | max(0.3 × `point_spacing`, 0.6 × pillar Ø) |
| Surface flood height tolerance | 0.05 |
| Position nudge tolerance | 0.01 |
| Link angle | 45° descent, fixed |
| Tip clearance before use | 0.6 × pillar Ø + cone length + 0.1 |
| Collision radius for tips | 0.6 × cone bottom Ø |
| First brace offset below pillar top | 0.15 |
| Brace step | max(`min_z_spacing`, 13.5 × `brace.diameter` − 2 × pillar distance) |
| Brace edge length limit | max XY cross spacing (30 mm) for plate pillars; 1.3 × 0.5 × point\_spacing (2.6 mm) for model-landing pillars |
| Short-pillar threshold | 5 mm (length + 0.6 × cone length) |
| Joint shrink | 0.002 |
| Intersection tolerance | 0.002 |

### 8.4 How settings reach the algorithm

The GUI does not use the SDK's JSON settings path. A connector class in the main executable copies each setting into the algorithm's parameter block on change (verified). Values that differ from a plain copy:

| Setting | What the algorithm gets |
| --- | --- |
| `touchtipdistance` (4.0) | itself, plus R = 0.5 × it (2.0 mm) for lattice spacing, brace edges, legs and merging |
| `supportmiddlediameter` | pillar Ø, second Ø and cross Ø (the cross-diameter setter is never connected) |
| `supportdensity` | raw value, used only to pick the cylinder mesh template |
| `modelbottomsupportdensity` | divided by 100; gates the pre-merge |
| `maximalsupportcrossspacingxy` | the field the SDK calls `supportCrossWidth` |

No connector exists for `supportmiddlediameter2`, `supportcrosswidth`, `supporthiddenangle`, `middlesupportdistance` or `radiusofadsorption` (verified). The algorithm's built-in defaults for those fields are never overwritten. An "enhanced" flag that unlocks the lattice, lattice merge and extra legs is cleared at the start of every auto-support run (verified).

## Implementation plan for mslicer

The work splits into a quick-fix milestone and five feature milestones. Each feature milestone ends with an automated check against the model and a side-by-side comparison with CHITUBOX on the same test set. Milestones 2–5 replace `tools/src/supports/` piece by piece behind the existing "Generate" button, so the app works at every step.

### 9.1 Module layout

All new code lives in the `tools` crate except slicing integration and UI.

| Path | Contents |
| --- | --- |
| `tools/src/supports/mod.rs` | Public API: `generate(model: &Mesh, bvh: &Bvh, cfg: &SupportConfig, progress) -> SupportSet` |
| `tools/src/supports/config.rs` | `SupportConfig` with the groups from section 8; Light, Middle and Heavy presets; serde for project files |
| `tools/src/supports/grid.rs` | Ray grid, intervals and the 8-neighbour links (Stage 1, 1.1 and 1.3) |
| `tools/src/supports/points.rs` | Classification, wall, bound, key, low, fill and dedupe (1.2, 1.4, 1.5) |
| `tools/src/supports/collide.rs` | Multi-line cone, cylinder and segment tests on the BVH (2.7) |
| `tools/src/supports/route/` | `tip.rs`, `ground.rs` (2.3), `model.rs` (2.4), `brace.rs` (2.5), `stabilize.rs` (2.6) |
| `tools/src/supports/parts.rs` | Part catalogue and per-part meshing (3.1, 3.2) |
| `tools/src/supports/raft.rs` | Raft outline and meshing (3.3) |
| `tools/src/supports/debug.rs` | Optional overlays: grid flags, coverage, rejected candidates |
| `slicer/src/geometry/bvh/mod.rs` | New `intersect_ray_all` that returns every hit along a ray, sorted by distance |
| `mslicer/src/project/supports.rs` | Stores a `SupportSet` instead of `[Vec3; 3]`; builds the render mesh from parts |
| `mslicer/src/app/slice_operation.rs` | Merges support and raft meshes into the slice job |

### 9.2 Core data structures

```rust
pub struct SupportPoint { pos: Vector3<f32>, normal: Vector3<f32>, kind: PointKind, depth: f32 }
pub enum PointKind { Wall, Bound, Key, Low, Fill, Manual }

pub enum PartKind { ContactSphere, TipCone, Joint, Link, Pillar, PillarLower, Slim, Boss, Base, Tail, Brace }
pub struct Part { kind: PartKind, a: Vector3<f32>, b: Vector3<f32>, radius_a: f32, radius_b: f32, shape: Shape }

pub enum Foot { Plate { xy: Vector2<f32> }, Model { pos: Vector3<f32>, normal: Vector3<f32> } }
pub struct Support { point: SupportPoint, parts: Vec<Part>, foot: Foot, group: u32 }

pub struct SupportSet { supports: Vec<Support>, braces: Vec<Part>, raft: Option<Mesh>, lift: f32 }
```

Rules every route must satisfy, asserted in debug builds:

- **Tip:** direction within 70° of straight down.
- **Link and brace:** exactly 45° descent.
- **Pillar:** exactly vertical.
- **Model clearance:** no part closer to the model than the configured clearance, except the contact and tail ends.

### 9.3 Milestones

**M0: quick fixes in the current code** (about 1 day)

- [ ] Use transformed normals in `place_face_supports` and `place_point_supports`.
- [ ] Place point supports at `origin_vertex`, not its neighbour.
- [ ] Segment-test the tip and arm with `Bvh::intersect_segment`, and drop supports that fail.
- [ ] Reject arms flatter than 45°.

Acceptance: on a rotated test model, no tip points into the model and no arm is flatter than 45°.

**M1: supports become real geometry** (2–3 days)

- [ ] Add `SupportSet` and the part catalogue, and mesh parts with configurable segment counts.
- [ ] Lift the model by `model_lift` while supports exist.
- [ ] Merge support meshes into slicing in `slice_operation.rs`.
- [ ] Store `SupportSet` in project files.

Acceptance: a sliced .goo of a supported model shows supports in every layer preview, and they print.

**M2: point placement (Stage 1)** (4–6 days)

- [ ] `intersect_ray_all`, the ray grid at about 200 000 cells, intervals and neighbour links. Run rays in parallel with rayon.
- [ ] Classification thresholds from `overhang_angle`.
- [ ] Low points, then the two-pass greedy fill with surface-following coverage, then dedupe.
- [ ] Wall, bound and key steps. Start with low and fill only, and add these behind flags.
- [ ] Debug overlay of covered and uncovered samples.

Acceptance:

- Every downward sample that needs support is within 3 mm of a point along the surface.
- No two points are closer than the dedupe radius.
- Point counts are within ±25% of CHITUBOX on the test set.
- Placement takes under 2 s for a 500 k-triangle model.

**M3: routing to the plate (Stage 2, 2.2, 2.3 and 2.7)** (5–7 days)

- [ ] Tip clamp and cone construction, sunk by the contact depth.
- [ ] Multi-line collision tests for cone, cylinder and segment.
- [ ] Straight drop, then the lateral candidate search: 8 directions plus the grid within 15 mm, nearest first, with the tip-direction order from 2.3.
- [ ] 45° links.

Acceptance:

- Zero part–model intersections, measured by a separate exact test of every part against the model.
- No tip more than 70° from vertical, no link that isn't 45°, no pillar that isn't vertical.
- At least 95% of points routed on the test set.

**M4: topology (2.4–2.6)** (5–7 days)

- [ ] Pillar pre-merge.
- [ ] Delaunay over pillar positions (crate `spade` or `delaunator`), inverted-V forks, spanning-tree edges up to the max XY cross spacing, 45° braces with the step formula, the 2.6 mm edge pass for model-landing pillars.
- [ ] Model-landing supports with tails.
- [ ] Stabilising legs, using the rules decided in section 10.

Acceptance:

- Every pillar taller than 10 mm is braced to at least one neighbour or has stabilising legs.
- Braces pass the link test.
- Model-landing tails sink 0.2–0.3 mm and never pass through a thin wall.

**M5: bases, raft, presets and UI** (3–4 days)

- [ ] Pillar cases 1–7.
- [ ] Raft from the Clipper-style union of footprints: Skate and Cross grid. `geo` or `i_overlay` can do the polygon offset and union.
- [ ] Preset picker (Light, Middle, Heavy) and grouped settings in `windows/supports.rs`.
- [ ] Manual support placement through the same routing code.

Acceptance:

- Printed test parts on the Saturn 4 Ultra 16K detach cleanly.
- Support volume within ±30% of CHITUBOX on the test set.

**M6: manual tools** (5–8 days, optional)

- [ ] Region auto-support: selection mask over the Stage 1 candidates, coverage seeded from existing contacts (4.3).
- [ ] "Merge selected" over existing pillars, reusing the M4 pre-merge (4.6).
- [ ] Two-click directional support with pillar-axis and link snapping (4.4).
- [ ] Upward tips from a joint (4.5).

Acceptance:

- Region support on half of a test part adds supports only inside the region, and none closer than the point spacing to existing contacts.
- Every manual tool's output passes the same collision tests as auto supports, or is drawn in a warning colour.

### 9.4 Test and comparison harness

1. **Test set** (8–10 STLs in `tools/tests/models/`):
   - a flat overhang plate
   - a sphere
   - a hollow cup with an undercut
   - a figurine with thin limbs
   - a part with a thin-wall slot (catches tips going through)
   - a T-shape (overhang above geometry)
   - a 45° ramp
   - a large flat-bottomed part
2. **CHITUBOX reference.** For each model, run CHITUBOX auto-support with the matching preset and export "STL with supports". Store the exports next to the models.
3. **Metrics script** (`tools/examples/support_metrics.rs`), reported per model and preset for both generators:
   - point count
   - coverage: share of supported-needing area within 3 mm of a contact
   - intersection count
   - angle histogram for tips, links and braces
   - support volume
   - generation time
4. **CI.** `cargo test` runs the invariant checks (no intersections, angle rules) on the small models. The CHITUBOX comparison runs on demand.

### 9.5 Performance notes

- The ray grid dominates. 200 000 rays on a BVH is well under a second with rayon. Intervals need all hits, not just the first.
- Collision tests are many short segment queries, so batch them per support and reuse the BVH. Prefilter by XY bounding box, as CHITUBOX does.
- Coverage flooding is per point over a disc of about 3 mm / cell size squared (≈ 2 300 cells at the default cell size). Precompute the disc offsets once.

## Open questions and risks

All nine details below are now answered from the code; answers live in the stage sections. A few sub-points stay open and are noted there. The last three items, on the manual tools (section 4), are new and still open; none blocks M0–M5. Each can be resolved by someone other than the implementer, from the decompiled output in `~/Documents/chitubox-supports-re/out-d2/`, or by observing CHITUBOX on a test model.

### 10.1 Unresolved details

- [x] **Stabilising legs (2.6).** At most one 45° leg per pillar: H ≥ 30 mm (or ≥ 20 mm under a link with > 5 mm run) and a > 30 mm unbraced gap. Starts 2 mm below the top, length 0.6 × R (1.2 mm). Full rules in 2.6.
- [x] **Pillar pre-merge (2.5).** Off by default: it needs `modelbottomsupportdensity` ≥ 200% or enhanced mode. Rules for when it is on are in 2.8.
- [x] **Model landing (2.4).** Vertical drop of ≥ 4.1 mm onto the same model. Then a single tail to the best-aligned surface spot, or a claw of 1–3 fingers from 8 probes about 18° off vertical. See 2.4.
- [x] **`supportdensity` (50%).** Not a spacing factor. Passed raw; it only picks the coarse cylinder mesh template (8.2, 8.4).
- [x] **`supportcrosswidth` (4.0) and `supporthiddenangle` (45°).** Neither reaches the algorithm. The SDK's `supportCrossWidth` field is the max XY brace span (`maximalsupportcrossspacingxy`). The hidden angle is read only by display code (8.4).
- [x] **Lower pillar diameter.** Never used: the GUI copies `supportmiddlediameter` into both fields, so `supportmiddlediameter2` is dead (3.1, 8.1).
- [x] **Pillar cases 1 and 4.** The floats are the raft outline's bounding box; the test means "a raft exists". The pillar then stands on the raft without a base. Cases 2/3 and 5/6 split on base shape "None" (3.2).
- [x] **Triangular lattice (2.1).** Enhanced mode only, never in auto-support. Spacing R = 2.0 mm, rows 1.73 mm apart. `modelbottomsupportdensity` does not scale it (2.1, 2.8).
- [x] **Horn test sampling.** 8 lines tapering from 0.6 × top Ø to 0.6 × bottom Ø, starts pulled 1.0 mm back from the contact. No spacing margin (2.7).
- [ ] **Pro build check (4.1).** Does a Pro or Advanced build write the 0x2b0 enhanced flag (lattice, "Magic Support"?), and does its panel expose brace Ø and R? Needs a paid binary.
- [ ] **Basic reachability (4.2).** Can Basic's GUI trigger the directional and upward tools? Trace which `SupportOpName` the `SupportManager` mouse handlers pick in each mode.
- [ ] **Region details (4.3).** Are existing supports that collide with the region deleted? What does `areaAddSupport`'s last flag do?

### 10.2 Risks

| Risk | Effect | Mitigation |
| --- | --- | --- |
| Licensing: code copied from the decompiled output | GPL-3.0 project contaminated with proprietary code | Implement only from this doc. Keep decompiled files out of the mslicer repo. Route any follow-up questions through a second person |
| Grid resolution on very large plates | 200 000 cells over a full 211 × 118 mm plate gives 0.35 mm cells, too coarse for thin features | Build one grid per model, as CHITUBOX does, not per plate |
| Non-manifold or open meshes | Ray intervals can't pair crossings correctly | Detect odd crossing counts per ray and fall back to treating the column's lowest hit as a bottom surface; warn in the UI |
| Collision tests too strict on dense meshes | Many points unroutable, gaps in support | Log rejection reasons in the debug overlay; tune clearance before touching the angle rules |
| Differences from CHITUBOX on edge cases | Users see different supports than they expect | Metrics harness (9.4) on a fixed test set; accept differences that are equal or better on intersections and coverage |
