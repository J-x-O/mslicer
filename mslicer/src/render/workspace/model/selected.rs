use std::range::Range;

/// What a range of faces is showing. These are bits, so ranges can overlap:
/// a hovered selected support is both.
#[derive(Clone, Copy)]
#[repr(u32)]
pub enum FaceState {
    Hovered = 1,
    Selected = 2,
    /// Not drawn, as while a support is being dragged.
    Hidden = 4,
    /// Outlined in the accent colour, keeping its own colour.
    Outlined = 8,
}

/// Face ranges with a [`FaceState`], for the base shader.
#[derive(Default)]
pub struct Selected {
    ranges: Vec<[u32; 4]>,
}

impl Selected {
    /// The ranges, with touching ones of the same state merged: the shader
    /// checks every range for every pixel, and selecting all supports would
    /// otherwise mean one range per support.
    pub fn into_inner(mut self) -> Vec<[u32; 4]> {
        self.ranges.sort_by_key(|x| (x[2], x[0]));
        let mut merged: Vec<[u32; 4]> = Vec::with_capacity(self.ranges.len());
        for range in self.ranges {
            match merged.last_mut() {
                Some(last) if last[2] == range[2] && range[0] <= last[1] => {
                    last[1] = last[1].max(range[1]);
                }
                _ => merged.push(range),
            }
        }
        merged.sort_by_key(|x| x[0]);
        merged
    }

    pub fn set_face(&mut self, idx: u32, state: FaceState) {
        self.ranges.push([idx, idx + 1, state as u32, 0]);
    }

    pub fn set_range(&mut self, range: Range<u32>, state: FaceState) {
        self.ranges.push([range.start, range.end, state as u32, 0]);
    }

    /// Gives every face of the object `state`.
    pub fn set_all(&mut self, state: FaceState) {
        self.ranges.push([0, u32::MAX, state as u32, 0]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn touching_ranges_of_one_state_merge() {
        let mut selected = Selected::default();
        selected.set_range((4..8).into(), FaceState::Selected);
        selected.set_range((0..4).into(), FaceState::Selected);
        selected.set_range((10..12).into(), FaceState::Selected);
        selected.set_range((2..6).into(), FaceState::Hovered);
        assert_eq!(
            selected.into_inner(),
            [[0, 8, 2, 0], [2, 6, 1, 0], [10, 12, 2, 0]]
        );
    }
}
