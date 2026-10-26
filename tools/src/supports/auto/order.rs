//! The two orderings placement depends on: an unstable introsort for the key
//! points and a binary min-heap for the sweep queue.
//!
//! On a flat face many candidates share one z, so which of them is taken
//! first is decided purely by how the sort and the heap break ties, and that
//! in turn decides where the whole lattice of points lands. Both are written
//! out step by step here instead of using `sort_unstable_by` or
//! `BinaryHeap`, whose tie orders are unspecified and may change between Rust
//! releases. The tests pin the exact permutations.

/// Below this length a partition is left for the final insertion sort.
const SORT_THRESHOLD: usize = 16;

/// Unstable introsort: median-of-three pivot, unguarded partition, heap sort
/// past the depth limit, one insertion sort at the end.
pub(super) fn introsort<T: Copy>(v: &mut [T], less: impl Fn(&T, &T) -> bool) {
    if v.len() < 2 {
        return;
    }
    let depth = 2 * (usize::BITS - 1 - v.len().leading_zeros()) as usize;
    introsort_loop(v, depth, &less);
    final_insertion_sort(v, &less);
}

fn introsort_loop<T: Copy>(mut v: &mut [T], mut depth: usize, less: &impl Fn(&T, &T) -> bool) {
    while v.len() > SORT_THRESHOLD {
        if depth == 0 {
            heap_sort(v, less);
            return;
        }
        depth -= 1;
        let cut = partition_pivot(v, less);
        let (low, high) = v.split_at_mut(cut);
        introsort_loop(high, depth, less);
        v = low;
    }
}

/// Moves the median of the second, middle and last elements to the front and
/// partitions the rest around it. Returns the start of the upper part.
fn partition_pivot<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) -> usize {
    let (last, mid) = (v.len() - 1, v.len() / 2);
    let (a, b, c) = (1, mid, last);
    let median = if less(&v[a], &v[b]) {
        if less(&v[b], &v[c]) {
            b
        } else if less(&v[a], &v[c]) {
            c
        } else {
            a
        }
    } else if less(&v[a], &v[c]) {
        a
    } else if less(&v[b], &v[c]) {
        c
    } else {
        b
    };
    v.swap(0, median);
    // The pivot at the front stops both scans, so they need no bounds check.
    let (mut lo, mut hi) = (1, v.len());
    loop {
        while less(&v[lo], &v[0]) {
            lo += 1;
        }
        hi -= 1;
        while less(&v[0], &v[hi]) {
            hi -= 1;
        }
        if lo >= hi {
            return lo;
        }
        v.swap(lo, hi);
        lo += 1;
    }
}

fn heap_sort<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) {
    let len = v.len();
    // Heapify, then repeatedly move the top to the end.
    let mut parent = (len - 2) / 2;
    loop {
        let value = v[parent];
        sift_down(v, parent, len, value, less);
        if parent == 0 {
            break;
        }
        parent -= 1;
    }
    let mut end = len;
    while end > 1 {
        end -= 1;
        let value = v[end];
        v[end] = v[0];
        sift_down(v, 0, end, value, less);
    }
}

/// Walks the hole at `top` down to a leaf along the larger child, then sifts
/// `value` back up from there.
fn sift_down<T: Copy>(
    v: &mut [T],
    top: usize,
    len: usize,
    value: T,
    less: &impl Fn(&T, &T) -> bool,
) {
    let mut hole = top;
    let mut child = top;
    while child < (len - 1) / 2 {
        child = 2 * (child + 1);
        if less(&v[child], &v[child - 1]) {
            child -= 1;
        }
        v[hole] = v[child];
        hole = child;
    }
    if len.is_multiple_of(2) && child == (len - 2) / 2 {
        child = 2 * (child + 1);
        v[hole] = v[child - 1];
        hole = child - 1;
    }
    let mut parent = hole.wrapping_sub(1) / 2;
    while hole > top && less(&v[parent], &value) {
        v[hole] = v[parent];
        hole = parent;
        parent = hole.wrapping_sub(1) / 2;
    }
    v[hole] = value;
}

fn final_insertion_sort<T: Copy>(v: &mut [T], less: &impl Fn(&T, &T) -> bool) {
    let head = v.len().min(SORT_THRESHOLD);
    for i in 1..head {
        let value = v[i];
        if less(&value, &v[0]) {
            v.copy_within(0..i, 1);
            v[0] = value;
        } else {
            linear_insert(v, i, less);
        }
    }
    for i in head..v.len() {
        linear_insert(v, i, less);
    }
}

fn linear_insert<T: Copy>(v: &mut [T], mut i: usize, less: &impl Fn(&T, &T) -> bool) {
    let value = v[i];
    // The head block already holds the minimum, so the scan always stops; the
    // bound only keeps a bad comparator from indexing out of range.
    while i > 0 && less(&value, &v[i - 1]) {
        v[i] = v[i - 1];
        i -= 1;
    }
    v[i] = value;
}

/// A binary min-heap of intervals keyed on z, with a fixed tie order: a pop
/// walks the hole down along the smaller child (the right one on ties), then
/// sifts the displaced last item up.
#[derive(Default)]
pub(super) struct MinHeap {
    items: Vec<(usize, f32)>,
}

impl MinHeap {
    pub(super) fn push(&mut self, interval: usize, z: f32) {
        self.items.push((interval, z));
        let hole = self.items.len() - 1;
        self.sift_up(hole, 0, (interval, z));
    }

    pub(super) fn pop(&mut self) -> Option<usize> {
        let top = *self.items.first()?;
        let last = self.items.pop()?;
        let len = self.items.len();
        if len > 0 {
            let mut hole = 0;
            let mut child = 0;
            while child < (len - 1) / 2 {
                child = 2 * (child + 1);
                if self.items[child].1 > self.items[child - 1].1 {
                    child -= 1;
                }
                self.items[hole] = self.items[child];
                hole = child;
            }
            if len.is_multiple_of(2) && child == (len - 2) / 2 {
                child = 2 * (child + 1);
                self.items[hole] = self.items[child - 1];
                hole = child - 1;
            }
            self.sift_up(hole, 0, last);
        }
        Some(top.0)
    }

    fn sift_up(&mut self, mut hole: usize, top: usize, value: (usize, f32)) {
        while hole > top {
            let parent = (hole - 1) / 2;
            if self.items[parent].1 <= value.1 {
                break;
            }
            self.items[hole] = self.items[parent];
            hole = parent;
        }
        self.items[hole] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::{MinHeap, introsort};

    /// Keys in 0..5 from a linear congruential generator, so most compare
    /// equal.
    fn tied_keys(n: u32) -> Vec<(u32, f32)> {
        let mut s = 12345u32.wrapping_add(n);
        (0..n)
            .map(|i| {
                s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
                (i, ((s >> 16) % 5) as f32)
            })
            .collect()
    }

    /// Ties must come out in exactly this order: placement on flat faces
    /// depends on it.
    #[test]
    fn introsort_tie_order_is_fixed() {
        let expected: [(u32, &[u32]); 2] = [
            (
                40,
                &[
                    25, 16, 30, 14, 13, 27, 33, 9, 8, 36, 38, 23, 5, 3, 29, 22, 0, 31, 7, 6, 39,
                    28, 20, 34, 26, 24, 21, 15, 10, 19, 18, 17, 12, 32, 11, 35, 4, 37, 2, 1,
                ],
            ),
            (
                100,
                &[
                    0, 28, 29, 26, 94, 51, 55, 17, 57, 60, 70, 86, 1, 3, 80, 72, 8, 75, 56, 77, 58,
                    48, 93, 43, 83, 32, 74, 84, 87, 27, 19, 20, 7, 6, 5, 4, 9, 65, 63, 15, 10, 54,
                    89, 73, 82, 25, 2, 39, 24, 37, 85, 22, 40, 41, 42, 23, 34, 45, 30, 71, 11, 69,
                    68, 67, 66, 76, 12, 61, 13, 88, 16, 98, 64, 97, 96, 21, 47, 44, 38, 95, 99, 92,
                    91, 90, 81, 79, 78, 62, 59, 53, 52, 50, 49, 46, 36, 35, 33, 31, 18, 14,
                ],
            ),
        ];
        for (n, order) in expected {
            let mut v = tied_keys(n);
            introsort(&mut v, |a, b| a.1 < b.1);
            let ids: Vec<u32> = v.iter().map(|e| e.0).collect();
            assert_eq!(ids, order, "n = {n}");
        }
    }

    /// Pop order under ties, each pop followed by one push as in the sweep.
    #[test]
    fn min_heap_tie_order_is_fixed() {
        let mut s = 777u32;
        let mut key = || {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
            ((s >> 16) % 4) as f32
        };
        let mut heap = MinHeap::default();
        let mut z = Vec::new();
        for i in 0..30 {
            z.push(key());
            heap.push(i, z[i]);
        }
        let mut pops = Vec::new();
        while let Some(i) = heap.pop() {
            pops.push(i);
            if z.len() < 50 {
                let next = z.len();
                z.push(z[i] + key());
                heap.push(next, z[next]);
            }
        }
        let expected = [
            0, 12, 3, 10, 7, 18, 13, 14, 30, 35, 36, 27, 28, 2, 41, 9, 44, 45, 21, 19, 20, 49, 32,
            34, 40, 47, 6, 37, 5, 26, 11, 29, 4, 8, 1, 15, 25, 24, 23, 33, 42, 48, 43, 17, 16, 22,
            46, 31, 38, 39,
        ];
        assert_eq!(pops, expected);
    }
}
