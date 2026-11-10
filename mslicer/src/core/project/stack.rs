//! A model's operation stack.
//!
//! A model is a source (its imported mesh) plus an ordered list of
//! operations, in three sections that follow from whether an operation
//! depends on how the model sits on the bed:
//!
//! 1. **Geometry** (model space, reorderable): Mesh Repair, Flip Winding
//!    Order, Hollow.
//! 2. **Orientation** (exactly one, its own slot): Orient.
//! 3. **Print prep** (bed space, fixed order): Supports.
//!
//! The stack holds each operation's header: its id, [`Mode`], whether it is
//! bypassed, and its own parameters. Operations whose state is large keep
//! it in a typed payload on the model, one per kind, as before the stack
//! existed: Hollow's shell and holes are `Model::hollow`, the support sets
//! and raft are `Model::supports`, and Orient's rotation is the model's
//! transform. That keeps one place for each piece of data while the rest of
//! the app keeps reading it where it did (see `decisions.md`, D2.1).
//!
//! The section rule applies to evaluation nodes, not rows: Hollow is one row
//! with two nodes ([`Op::nodes`]): the shell with the manual holes (model
//! space, at Hollow's place in the stack) and the auto holes (each void
//! vented at its lowest point, which needs the orientation).

use common::{
    id_type,
    serde::{Deserializer, Serializer},
};
use tools::supports::mesh::SupportPreset;

use crate::core::project::hollow::HollowSpec;

/// Where an operation (an evaluation node) sits.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Section {
    /// Model space; doesn't depend on the orientation. Reorderable.
    Geometry,
    /// The orientation itself.
    Orientation,
    /// Bed space; needs the orientation. Fixed order.
    PrintPrep,
}

/// How an operation's result comes about.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mode {
    /// Fully parametric: recomputed when its inputs change.
    #[default]
    Auto,
    /// The auto result with manual edits on top (placed supports, drilled
    /// holes). Direct manual edits switch Auto to Edited.
    Edited,
    /// Baked: never recomputed, and not editable. Only set by the user.
    Frozen,
}

impl Mode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Auto => "Auto",
            Self::Edited => "Edited",
            Self::Frozen => "Frozen",
        }
    }

    fn to_u8(self) -> u8 {
        match self {
            Self::Auto => 0,
            Self::Edited => 1,
            Self::Frozen => 2,
        }
    }

    fn from_u8(x: u8) -> Self {
        match x {
            1 => Self::Edited,
            2 => Self::Frozen,
            _ => Self::Auto,
        }
    }
}

id_type!(OperationId, u32);

/// One row of the stack.
#[derive(Clone, Debug, PartialEq)]
pub struct Operation {
    pub id: OperationId,
    pub mode: Mode,
    /// Skipped in evaluation, but kept with its settings.
    pub bypassed: bool,
    pub op: Op,
}

/// The operation kinds and their own parameters.
#[derive(Clone, Debug, PartialEq)]
pub enum Op {
    /// Welds vertices, removes degenerate and repeated faces, closes holes
    /// and fixes the winding order of the incoming mesh.
    Repair(Repair),
    /// Flips every face of the incoming mesh.
    FlipWinding,
    /// Hollows the model (shell and holes: `Model::hollow`).
    Hollow(HollowOp),
    /// The orientation (the rotation is the model's transform).
    Orient(Orient),
    /// Supports and raft (the sets: `Model::supports`).
    Supports(SupportsOp),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Repair {
    /// Vertices closer than this (mm) are welded.
    pub vertex_epsilon: f32,
}

impl Default for Repair {
    fn default() -> Self {
        Self {
            vertex_epsilon: 1e-4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HollowOp {
    /// Vent each void at its lowest point (in the current orientation) in
    /// addition to the manual holes.
    pub auto_holes: bool,
    /// The auto holes' diameter (mm).
    pub diameter: f32,
}

impl Default for HollowOp {
    /// As loaded from projects that had no auto holes: off.
    fn default() -> Self {
        Self {
            auto_holes: false,
            diameter: 2.0,
        }
    }
}

impl HollowOp {
    /// A Hollow added now: its voids drain on their own.
    pub fn new() -> Self {
        Self {
            auto_holes: true,
            ..Self::default()
        }
    }
}

/// How auto-orient weighs its objectives. A stub until auto-orient exists
/// (Phase 7).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutoOrient {
    pub fewer_supports: f32,
    pub low_peel_force: f32,
    pub short_print: f32,
    pub protect_faces: f32,
}

impl Default for AutoOrient {
    fn default() -> Self {
        Self {
            fewer_supports: 1.0,
            low_peel_force: 0.5,
            short_print: 0.25,
            protect_faces: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Orient {
    /// Auto-orient's settings, when it chose the orientation. Manual
    /// otherwise.
    pub auto: Option<AutoOrient>,
    /// Something upstream changed since auto-orient ran: its result is held
    /// (it never moves the model on its own) but may no longer be the best.
    pub outdated: bool,
}

/// The settings the auto supports were generated with.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportAuto {
    pub preset: SupportPreset,
    pub min_spacing: f32,
    pub max_angle: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SupportsOp {
    /// What the auto supports were last generated with, if they were.
    pub auto: Option<SupportAuto>,
}

/// An evaluation node of an operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Node {
    Repair,
    FlipWinding,
    /// Hollow's shell and manual holes.
    Shell,
    /// Hollow's auto holes: where each void's lowest point is, oriented.
    AutoHoles,
    Orient,
    Supports,
}

impl Node {
    pub fn name(self) -> &'static str {
        match self {
            Self::Repair => "repair",
            Self::FlipWinding => "flip",
            Self::Shell => "shell and manual holes",
            Self::AutoHoles => "auto holes",
            Self::Orient => "orientation",
            Self::Supports => "supports",
        }
    }

    pub fn section(self) -> Section {
        match self {
            Self::Repair | Self::FlipWinding | Self::Shell => Section::Geometry,
            Self::Orient => Section::Orientation,
            Self::AutoHoles | Self::Supports => Section::PrintPrep,
        }
    }
}

impl Op {
    /// The section of the row, where it is listed and can be moved.
    pub fn section(&self) -> Section {
        match self {
            Self::Repair(_) | Self::FlipWinding | Self::Hollow(_) => Section::Geometry,
            Self::Orient(_) => Section::Orientation,
            Self::Supports(_) => Section::PrintPrep,
        }
    }

    /// The evaluation nodes the row is made of.
    pub fn nodes(&self) -> &'static [Node] {
        match self {
            Self::Repair(_) => &[Node::Repair],
            Self::FlipWinding => &[Node::FlipWinding],
            Self::Hollow(_) => &[Node::Shell, Node::AutoHoles],
            Self::Orient(_) => &[Node::Orient],
            Self::Supports(_) => &[Node::Supports],
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Repair(_) => "Mesh Repair",
            Self::FlipWinding => "Flip Winding Order",
            Self::Hollow(_) => "Hollow",
            Self::Orient(_) => "Orient",
            Self::Supports(_) => "Supports",
        }
    }

    /// At most one of these per stack.
    pub fn is_singleton(&self) -> bool {
        matches!(self, Self::Hollow(_) | Self::Orient(_) | Self::Supports(_))
    }

    /// Whether the row can be bypassed. The orientation can't: the viewport
    /// always shows the model as oriented.
    pub fn can_bypass(&self) -> bool {
        !matches!(self, Self::Orient(_))
    }

    /// Whether the row can be removed.
    pub fn can_remove(&self) -> bool {
        !matches!(self, Self::Orient(_))
    }

    fn kind(&self) -> u8 {
        match self {
            Self::Repair(_) => 0,
            Self::FlipWinding => 1,
            Self::Hollow(_) => 2,
            Self::Orient(_) => 3,
            Self::Supports(_) => 4,
        }
    }

    fn same_kind(&self, other: &Self) -> bool {
        self.kind() == other.kind()
    }
}

impl Operation {
    pub fn new(op: Op) -> Self {
        Self {
            id: OperationId::new(),
            mode: Mode::Auto,
            bypassed: false,
            op,
        }
    }

    /// Whether it takes part in evaluation.
    pub fn active(&self) -> bool {
        !self.bypassed
    }
}

/// The ordered operations of a model. Always sorted by section, with
/// exactly one Orient.
#[derive(Clone, Debug, PartialEq)]
pub struct Stack {
    ops: Vec<Operation>,
}

impl Default for Stack {
    /// A new model's stack: just its orientation.
    fn default() -> Self {
        Self {
            ops: vec![Operation::new(Op::Orient(Orient::default()))],
        }
    }
}

impl Stack {
    pub fn ops(&self) -> &[Operation] {
        &self.ops
    }

    /// The rows, to change their modes and settings. Their kinds and order
    /// can only change through [`Stack::add`], [`Stack::remove`] and
    /// [`Stack::move_by`].
    pub fn ops_mut(&mut self) -> impl Iterator<Item = &mut Operation> {
        self.ops.iter_mut()
    }

    #[cfg(test)]
    pub fn len(&self) -> usize {
        self.ops.len()
    }

    pub fn get(&self, id: OperationId) -> Option<&Operation> {
        self.ops.iter().find(|o| o.id == id)
    }

    pub fn get_mut(&mut self, id: OperationId) -> Option<&mut Operation> {
        self.ops.iter_mut().find(|o| o.id == id)
    }

    pub fn index(&self, id: OperationId) -> Option<usize> {
        self.ops.iter().position(|o| o.id == id)
    }

    /// The first row matching `pick`.
    pub fn find<T>(&self, pick: impl Fn(&Op) -> Option<&T>) -> Option<(&Operation, &T)> {
        self.ops.iter().find_map(|o| pick(&o.op).map(|x| (o, x)))
    }

    pub fn hollow(&self) -> Option<(&Operation, &HollowOp)> {
        self.find(|op| match op {
            Op::Hollow(h) => Some(h),
            _ => None,
        })
    }

    #[cfg(test)]
    pub fn orient(&self) -> (&Operation, &Orient) {
        self.find(|op| match op {
            Op::Orient(o) => Some(o),
            _ => None,
        })
        .expect("a stack always has its orientation")
    }

    /// The orientation row's id and settings.
    pub fn orient_row(&self) -> (OperationId, &Orient) {
        let (op, orient) = self.orient_inner();
        (op.id, orient)
    }

    // The expects below hold: `ops` is private, every way to build a stack
    // (`default`, `repaired`) adds an Orient, and `remove` keeps it.
    fn orient_inner(&self) -> (&Operation, &Orient) {
        self.find(|op| match op {
            Op::Orient(o) => Some(o),
            _ => None,
        })
        .expect("a stack always has its orientation")
    }

    pub fn orient_mut(&mut self) -> &mut Operation {
        (self.ops.iter_mut())
            .find(|o| matches!(o.op, Op::Orient(_)))
            .expect("a stack always has its orientation")
    }

    pub fn supports(&self) -> Option<(&Operation, &SupportsOp)> {
        self.find(|op| match op {
            Op::Supports(s) => Some(s),
            _ => None,
        })
    }

    pub fn hollow_mut(&mut self) -> Option<&mut Operation> {
        (self.ops.iter_mut()).find(|o| matches!(o.op, Op::Hollow(_)))
    }

    /// Whether the model is hollowed: a Hollow row that isn't bypassed.
    pub fn hollow_active(&self) -> bool {
        self.hollow().is_some_and(|(o, _)| o.active())
    }

    /// Whether the supports take part: a Supports row that isn't bypassed.
    pub fn supports_active(&self) -> bool {
        self.supports().is_some_and(|(o, _)| o.active())
    }

    /// The active geometry rows before the first Hollow (what makes the
    /// model's surface), and after it (applied to the hollowed print mesh).
    pub fn geometry(&self) -> (Vec<&Operation>, Vec<&Operation>) {
        let geometry =
            (self.ops.iter()).filter(|o| o.op.section() == Section::Geometry && o.active());
        let mut before = Vec::new();
        let mut after = Vec::new();
        let mut hollowed = false;
        for op in geometry {
            match (&op.op, hollowed) {
                (Op::Hollow(_), _) => hollowed = true,
                (_, false) => before.push(op),
                (_, true) => after.push(op),
            }
        }
        (before, after)
    }

    /// Adds an operation at the end of its section, or returns the existing
    /// one of a kind there can only be one of. Returns its id.
    pub fn add(&mut self, op: Op) -> OperationId {
        if op.is_singleton()
            && let Some(existing) = self.ops.iter().find(|o| o.op.same_kind(&op))
        {
            return existing.id;
        }
        let section = op.section();
        let end = (self.ops.iter())
            .position(|o| o.op.section() > section)
            .unwrap_or(self.ops.len());
        // Repair and Flip fix the incoming mesh, so they go before Hollow;
        // they can be moved after it.
        let fixes_input = matches!(op, Op::Repair(_) | Op::FlipWinding);
        let hollow = self.ops.iter().position(|o| matches!(o.op, Op::Hollow(_)));
        let at = match (fixes_input, hollow) {
            (true, Some(hollow)) => hollow,
            _ => end,
        };
        let operation = Operation::new(op);
        let id = operation.id;
        self.ops.insert(at, operation);
        id
    }

    /// Removes an operation. The orientation can't be removed.
    pub fn remove(&mut self, id: OperationId) -> Option<Operation> {
        let index = self.index(id)?;
        self.ops[index]
            .op
            .can_remove()
            .then(|| self.ops.remove(index))
    }

    /// Whether the row can move up (`-1`) or down (`+1`): only within the
    /// geometry section.
    pub fn can_move(&self, id: OperationId, by: isize) -> bool {
        let Some(index) = self.index(id) else {
            return false;
        };
        let Some(other) = index.checked_add_signed(by).and_then(|i| self.ops.get(i)) else {
            return false;
        };
        self.ops[index].op.section() == Section::Geometry && other.op.section() == Section::Geometry
    }

    /// Moves a geometry row one place up (`-1`) or down (`+1`) within its
    /// section. Returns whether it moved.
    pub fn move_by(&mut self, id: OperationId, by: isize) -> bool {
        if !self.can_move(id, by) {
            return false;
        }
        let Some(index) = self.index(id) else {
            return false;
        };
        self.ops.swap(index, index.saturating_add_signed(by));
        true
    }

    /// Whether the stack is well formed: sorted by section, one Orient, at
    /// most one of each singleton.
    #[cfg(test)]
    pub fn is_valid(&self) -> bool {
        let sorted = (self.ops.windows(2)).all(|w| w[0].op.section() <= w[1].op.section());
        let singletons = (self.ops.iter().filter(|o| o.op.is_singleton()))
            .all(|a| (self.ops.iter()).filter(|b| b.op.same_kind(&a.op)).count() == 1);
        let orient = (self.ops.iter())
            .filter(|o| matches!(o.op, Op::Orient(_)))
            .count()
            == 1;
        sorted && singletons && orient
    }

    pub fn serialize<T: Serializer>(&self, ser: &mut T) {
        ser.write_u32_be(self.ops.len() as u32);
        for operation in &self.ops {
            ser.write_u32_be(operation.id.raw());
            ser.write_u8(operation.op.kind());
            ser.write_u8(operation.mode.to_u8());
            ser.write_bool(operation.bypassed);
            match &operation.op {
                Op::Repair(repair) => ser.write_f32_be(repair.vertex_epsilon),
                Op::FlipWinding => {}
                Op::Hollow(hollow) => {
                    ser.write_bool(hollow.auto_holes);
                    ser.write_f32_be(hollow.diameter);
                }
                Op::Orient(orient) => {
                    ser.write_bool(orient.outdated);
                    ser.write_bool(orient.auto.is_some());
                    let auto = orient.auto.unwrap_or_default();
                    for x in [
                        auto.fewer_supports,
                        auto.low_peel_force,
                        auto.short_print,
                        auto.protect_faces,
                    ] {
                        ser.write_f32_be(x);
                    }
                }
                Op::Supports(supports) => {
                    ser.write_bool(supports.auto.is_some());
                    if let Some(auto) = &supports.auto {
                        auto.preset.serialize(ser);
                        ser.write_f32_be(auto.min_spacing);
                        ser.write_f32_be(auto.max_angle);
                    }
                }
            }
        }
    }

    /// Reads a stack. A malformed one (from a newer or broken file) is
    /// repaired: rows out of order are sorted, extra singletons dropped and
    /// a missing orientation added.
    pub fn deserialize<T: Deserializer>(des: &mut T, version: u16) -> Self {
        let count = des.read_u32_be();
        let mut ops = Vec::new();
        for _ in 0..count {
            if des.truncated() {
                break;
            }
            let id = OperationId::from_raw(des.read_u32_be());
            OperationId::bump_past(id.raw());
            let kind = des.read_u8();
            let mode = Mode::from_u8(des.read_u8());
            let bypassed = des.read_bool();
            let op = match kind {
                0 => Op::Repair(Repair {
                    vertex_epsilon: des.read_f32_be(),
                }),
                1 => Op::FlipWinding,
                2 => Op::Hollow(HollowOp {
                    auto_holes: des.read_bool(),
                    diameter: match version {
                        ..20 => HollowOp::default().diameter,
                        _ => des.read_f32_be(),
                    },
                }),
                3 => {
                    let outdated = des.read_bool();
                    let auto = des.read_bool();
                    let params = AutoOrient {
                        fewer_supports: des.read_f32_be(),
                        low_peel_force: des.read_f32_be(),
                        short_print: des.read_f32_be(),
                        protect_faces: des.read_f32_be(),
                    };
                    Op::Orient(Orient {
                        auto: auto.then_some(params),
                        outdated,
                    })
                }
                _ => Op::Supports(SupportsOp {
                    auto: des.read_bool().then(|| SupportAuto {
                        preset: SupportPreset::deserialize(des),
                        min_spacing: des.read_f32_be(),
                        max_angle: des.read_f32_be(),
                    }),
                }),
            };
            ops.push(Operation {
                id,
                mode,
                bypassed,
                op,
            });
        }
        Self::repaired(ops)
    }

    fn repaired(mut ops: Vec<Operation>) -> Self {
        ops.sort_by_key(|o| o.op.section());
        let mut seen: Vec<u8> = Vec::new();
        ops.retain(|o| {
            let first = !seen.contains(&o.op.kind());
            seen.push(o.op.kind());
            !o.op.is_singleton() || first
        });
        let mut stack = Self { ops };
        if !stack.ops.iter().any(|o| matches!(o.op, Op::Orient(_))) {
            stack.add(Op::Orient(Orient::default()));
        }
        stack
    }

    /// The stack of a model from a project saved before stacks existed
    /// (v18 and older): its orientation (manual), Hollow if it had a shell
    /// or holes, Supports if it had any. Modes follow from what is there:
    /// holes and hand-placed supports are edits.
    pub fn migrated(hollow: &HollowSpec, auto_supports: bool, manual_supports: bool) -> Self {
        let mut stack = Self::default();
        stack.orient_mut().mode = Mode::Edited;
        if hollow.shell.is_some() || !hollow.holes.is_empty() {
            let id = stack.add(Op::Hollow(HollowOp::default()));
            let mode = Self::hollow_mode(hollow);
            if let Some(operation) = stack.get_mut(id) {
                operation.mode = mode;
            }
        }
        if auto_supports || manual_supports {
            let id = stack.add(Op::Supports(SupportsOp::default()));
            let mode = Self::supports_mode(manual_supports);
            if let Some(operation) = stack.get_mut(id) {
                operation.mode = mode;
            }
        }
        stack
    }

    /// Hollow's mode from what the user did: manual holes are edits.
    pub fn hollow_mode(spec: &HollowSpec) -> Mode {
        match spec.holes.is_empty() {
            true => Mode::Auto,
            false => Mode::Edited,
        }
    }

    /// Supports' mode from what the user did: hand-placed supports are edits.
    pub fn supports_mode(manual: bool) -> Mode {
        match manual {
            true => Mode::Edited,
            false => Mode::Auto,
        }
    }

    /// A copy with fresh ids, for a model made from this one.
    pub fn copied(&self) -> Self {
        let mut copy = self.clone();
        copy.ops.iter_mut().for_each(|o| o.id = OperationId::new());
        copy
    }
}

#[cfg(test)]
mod tests {
    use common::serde::{DynamicSerializer, SliceDeserializer};

    use super::*;

    #[test]
    fn rows_go_to_their_section_and_singletons_stay_single() {
        let mut stack = Stack::default();
        let supports = stack.add(Op::Supports(SupportsOp::default()));
        let hollow = stack.add(Op::Hollow(HollowOp::default()));
        let repair = stack.add(Op::Repair(Repair::default()));
        assert_eq!(stack.add(Op::Hollow(HollowOp::default())), hollow);
        let kinds: Vec<_> = stack.ops().iter().map(|o| o.op.name()).collect();
        // Repair fixes the input, so it goes before Hollow.
        assert_eq!(kinds, ["Mesh Repair", "Hollow", "Orient", "Supports"]);
        assert!(stack.is_valid());
        assert!(stack.get(supports).is_some());
        let _ = repair;
    }

    #[test]
    fn only_geometry_rows_reorder() {
        let mut stack = Stack::default();
        let hollow = stack.add(Op::Hollow(HollowOp::default()));
        let flip = stack.add(Op::FlipWinding);
        let supports = stack.add(Op::Supports(SupportsOp::default()));
        let orient = stack.orient().0.id;

        assert_eq!(stack.index(flip), Some(0), "before Hollow");
        assert!(stack.move_by(flip, 1));
        assert_eq!(stack.index(flip), Some(1));
        assert_eq!(stack.index(hollow), Some(0));
        // Not past the orientation, and the others don't move at all.
        assert!(!stack.move_by(flip, 1));
        assert!(!stack.move_by(orient, -1));
        assert!(!stack.move_by(supports, -1));
        assert!(stack.is_valid());
    }

    #[test]
    fn geometry_splits_around_hollow_and_skips_bypassed_rows() {
        let mut stack = Stack::default();
        let repair = stack.add(Op::Repair(Repair::default()));
        stack.add(Op::Hollow(HollowOp::default()));
        let flip = stack.add(Op::FlipWinding);
        assert!(stack.move_by(flip, 1));
        let ids = |ops: Vec<&Operation>| ops.iter().map(|o| o.id).collect::<Vec<_>>();
        let (before, after) = stack.geometry();
        assert_eq!((ids(before), ids(after)), (vec![repair], vec![flip]));
        stack.get_mut(repair).unwrap().bypassed = true;
        assert!(stack.geometry().0.is_empty());
    }

    #[test]
    fn the_orientation_stays() {
        let mut stack = Stack::default();
        let orient = stack.orient().0.id;
        assert!(stack.remove(orient).is_none());
        assert!(!stack.orient().0.op.can_bypass());
    }

    #[test]
    fn round_trips_and_repairs_bad_input() {
        let mut stack = Stack::default();
        stack.add(Op::Repair(Repair {
            vertex_epsilon: 0.01,
        }));
        let hollow = stack.add(Op::Hollow(HollowOp::new()));
        stack.get_mut(hollow).unwrap().mode = Mode::Edited;
        stack.add(Op::Supports(SupportsOp {
            auto: Some(SupportAuto {
                preset: SupportPreset::HEAVY,
                min_spacing: 2.0,
                max_angle: 0.7,
            }),
        }));
        stack.orient_mut().mode = Mode::Frozen;
        stack.orient_mut().op = Op::Orient(Orient {
            auto: Some(AutoOrient::default()),
            outdated: true,
        });

        let mut ser = DynamicSerializer::new();
        stack.serialize(&mut ser);
        let data = ser.into_inner();
        let back = Stack::deserialize(&mut SliceDeserializer::new(&data), 20);
        assert_eq!(back, stack);

        // Out of order, two Hollows, no Orient.
        let bad = vec![
            Operation::new(Op::Supports(SupportsOp::default())),
            Operation::new(Op::Hollow(HollowOp::default())),
            Operation::new(Op::Hollow(HollowOp::default())),
        ];
        let fixed = Stack::repaired(bad);
        assert!(fixed.is_valid());
        assert_eq!(fixed.len(), 3);
    }
}
