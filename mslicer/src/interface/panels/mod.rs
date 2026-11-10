use std::{
    mem,
    ops::{Deref, DerefMut},
};

mod hollow;
mod islands;
mod issues;
mod logs;
mod models;
mod notes;
mod outliner;
mod properties;
pub mod remote_print;
mod scrubber;
mod slice_config;
mod sliced;
pub mod supports;
mod tabs;
mod viewport;
mod workspace;

use egui_dock::{DockState, Node, NodeIndex, Tree};

use crate::core::config::Config;
pub use tabs::{Tab, Tabs};

/// The version of the default layout. A persisted layout of another
/// version is replaced by the default once (version 1, from before the
/// Sliced tab was back by the viewport, only gets that tab).
const LAYOUT_VERSION: u32 = 2;

pub struct Panels {
    pub dock_state: DockState<Tab>,
    /// The dock area has the real state this frame; `dock_state` holds a
    /// stand-in until it is given back.
    taken: bool,
}

impl Panels {
    pub fn new(config: &mut Config) -> Self {
        let mut dock_state = DockState::new(vec![Tab::Viewport]);
        let surface = dock_state.main_surface_mut();

        // A layout from before the current panels is reset once.
        let version = config.ui.layout_version;
        config.ui.layout_version = LAYOUT_VERSION;
        match &mut config.ui.panels {
            Some(past_state) if version == LAYOUT_VERSION => *surface = mem::take(past_state),
            Some(past_state) if version == 1 => {
                *surface = mem::take(past_state);
                if surface.find_tab(&Tab::Sliced).is_none()
                    && let Some((node, _)) = surface.find_tab(&Tab::Viewport)
                {
                    surface[node].append_tab(Tab::Sliced);
                }
            }
            _ => default_dock_layout(surface),
        }

        match surface.find_tab(&Tab::Viewport) {
            Some((ni, ti)) => surface.set_active_tab(ni, ti),
            None => *surface = Tree::new(vec![Tab::Viewport]),
        }

        Self {
            dock_state,
            taken: false,
        }
    }

    pub fn take_inner(&mut self) -> DockState<Tab> {
        self.taken = true;
        mem::replace(&mut self.dock_state, DockState::new(vec![Tab::Viewport]))
    }

    /// Gives back the state from `take_inner`, unless the layout was reset
    /// in the meantime.
    pub fn replace_inner(&mut self, state: DockState<Tab>) {
        if mem::take(&mut self.taken) {
            self.dock_state = state;
        }
    }

    /// Focuses a tab, adding it as a sibling to the viewport if it doesn't
    /// already exist
    pub fn focus_tab(&mut self, tab: Tab) {
        if let Some(panel) = self.find_tab(&tab) {
            self.set_active_tab(panel);
        } else {
            self.main_surface_mut().push_to_first_leaf(tab);
        }
    }

    pub fn is_open(&self, tab: Tab) -> bool {
        self.find_tab(&tab).is_some()
    }

    pub fn reset_ui(&mut self) {
        self.dock_state = DockState::new(vec![Tab::Viewport]);
        self.taken = false;
        let surface = self.main_surface_mut();
        default_dock_layout(surface);
    }

    pub fn checkbox(&mut self, tab: Tab, callback: impl FnOnce(&mut bool)) {
        let existing = self.find_tab(&tab);
        let mut open = existing.is_some();
        callback(&mut open);

        if !open && let Some(tab) = existing {
            self.remove_tab(tab);
        } else if open && existing.is_none() {
            self.add_window(vec![tab]);
        }
    }

    pub fn update(&mut self, window_width: f32) {
        let surface = self.main_surface_mut();
        let Some((mut node, _)) = surface.find_tab(&Tab::Viewport) else {
            return;
        };

        while let Some(parent) = node.parent()
            && parent != NodeIndex::root()
        {
            node = parent;
        }

        let Node::Horizontal(split) = &mut surface[NodeIndex::root()] else {
            return;
        };

        let previous_width = split.rect.width();
        let min_size = 325.0 / window_width;
        let change = previous_width / window_width;
        split.fraction = if node.is_left() {
            1.0 - (1.0 - split.fraction) * change
        } else {
            split.fraction * change
        }
        .clamp(min_size, 1.0);
    }
}

/// The viewport with the Sliced tab behind it; the Outliner over
/// Properties left of it, with Issues, Notes and the Workspace below. The
/// old panels are in the View menu.
fn default_dock_layout(surface: &mut Tree<Tab>) {
    if let Some((node, _)) = surface.find_tab(&Tab::Viewport)
        && surface.find_tab(&Tab::Sliced).is_none()
    {
        surface[node].append_tab(Tab::Sliced);
    }
    let [_old_node, new_node] = surface.split_left(NodeIndex::root(), 0.22, vec![Tab::Outliner]);
    let [_old_node, new_node] = surface.split_below(new_node, 0.35, vec![Tab::Properties]);
    surface.split_below(new_node, 0.7, vec![Tab::Issues, Tab::Notes, Tab::Workspace]);
    if let Some((node, tab)) = surface.find_tab(&Tab::Viewport) {
        surface.set_active_tab(node, tab);
    }
}

impl Deref for Panels {
    type Target = DockState<Tab>;

    fn deref(&self) -> &Self::Target {
        &self.dock_state
    }
}

impl DerefMut for Panels {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.dock_state
    }
}

#[cfg(test)]
mod tests {
    use egui_dock::DockState;

    use super::{Panels, Tab};
    use crate::core::config::Config;

    #[test]
    fn panels_survive_a_frame_without_the_viewport() {
        let mut panels = Panels::new(&mut Config::default());

        // The dock area has the state; a reset in the meantime wins.
        let taken = panels.take_inner();
        panels.reset_ui();
        panels.replace_inner(DockState::new(vec![Tab::Notes]));
        assert!(panels.is_open(Tab::Viewport));
        drop(taken);

        // A layout without the viewport leaves the split alone.
        panels.dock_state = DockState::new(vec![Tab::Notes]);
        panels.update(1000.0);
    }
}
