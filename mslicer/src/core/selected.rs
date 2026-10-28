use std::{collections::HashSet, iter};

use itertools::Either;
use tools::supports::SupportId;

use crate::core::project::{CollectionId, model::ModelId};

pub enum SelectedPrinter {
    Project,
    Custom(usize),
    Preset(usize, usize),
}

#[derive(Default)]
pub enum SelectedModel {
    #[default]
    None,
    Models(HashSet<ModelId>),
    Collection(CollectionId),
}

#[derive(Default)]
pub struct SelectedSupports {
    supports: HashSet<SelectedSupport>,
    /// The most recently selected support.
    last: Option<SelectedSupport>,
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
pub struct SelectedSupport {
    pub model: ModelId,
    pub support: SupportId,
}

impl SelectedModel {
    pub fn clear(&mut self) {
        *self = SelectedModel::None;
    }

    pub fn has_any(&self) -> bool {
        match self {
            SelectedModel::None => false,
            SelectedModel::Models(set) => !set.is_empty(),
            SelectedModel::Collection(..) => true,
        }
    }

    pub fn model_clicked(&mut self, id: ModelId, shift: bool) {
        match self {
            SelectedModel::None | SelectedModel::Collection(_) => {
                let mut set = HashSet::new();
                set.insert(id);
                *self = SelectedModel::Models(set);
            }
            SelectedModel::Models(set) => {
                if shift {
                    if set.contains(&id) {
                        set.remove(&id);
                    } else {
                        set.insert(id);
                    }
                } else {
                    set.clear();
                    set.insert(id);
                }
            }
        }
    }

    pub fn select_model(&mut self, id: ModelId) {
        match self {
            SelectedModel::None | SelectedModel::Collection(_) => {
                let mut set = HashSet::new();
                set.insert(id);
                *self = SelectedModel::Models(set);
            }
            SelectedModel::Models(set) => {
                set.insert(id);
            }
        }
    }

    pub fn selected_models(&self) -> impl Iterator<Item = ModelId> {
        match self {
            SelectedModel::None | SelectedModel::Collection(_) => Either::Left(iter::empty()),
            SelectedModel::Models(set) => Either::Right(set.iter().copied()),
        }
    }

    pub fn contains_model(&self, id: ModelId) -> bool {
        match self {
            SelectedModel::Models(set) => set.contains(&id),
            _ => false,
        }
    }

    pub fn single_model(&self) -> Option<ModelId> {
        match self {
            SelectedModel::Models(set) if set.len() == 1 => set.iter().next().copied(),
            _ => None,
        }
    }

    pub fn has_models(&self) -> bool {
        match self {
            SelectedModel::Models(set) => !set.is_empty(),
            _ => false,
        }
    }

    pub fn contains_collection(&self, id: CollectionId) -> bool {
        match self {
            SelectedModel::Collection(collection) => *collection == id,
            _ => false,
        }
    }

    pub fn collection_clicked(&mut self, id: CollectionId, shift: bool) {
        match self {
            SelectedModel::Models(set) if !shift || set.is_empty() => *self = Self::Collection(id),
            SelectedModel::Collection(group) if id == *group => self.clear(),
            SelectedModel::Collection(_) | SelectedModel::None => *self = Self::Collection(id),
            _ => {}
        }
    }

    pub fn single_collection(&self) -> Option<CollectionId> {
        match self {
            SelectedModel::Collection(id) => Some(*id),
            _ => None,
        }
    }
}

impl SelectedSupport {
    pub fn new(model: ModelId, support: SupportId) -> Self {
        Self { model, support }
    }
}

impl SelectedSupports {
    pub fn iter(&self) -> impl Iterator<Item = &SelectedSupport> {
        self.supports.iter()
    }

    pub fn clear(&mut self) {
        self.supports.clear();
        self.last = None;
    }

    /// The support whose settings the panel shows: the most recently selected
    /// one, else any.
    pub fn focus(&self) -> Option<SelectedSupport> {
        (self.last.filter(|x| self.supports.contains(x)))
            .or_else(|| self.supports.iter().next().copied())
    }

    pub fn for_model(&self, model: ModelId) -> impl Iterator<Item = SupportId> {
        self.supports
            .iter()
            .filter(move |x| x.model == model)
            .map(|x| x.support)
    }

    pub fn count(&self) -> usize {
        self.supports.len()
    }

    pub fn model_count(&self) -> usize {
        let mut models = HashSet::new();
        for support in self.supports.iter() {
            models.insert(support.model);
        }

        models.len()
    }

    pub fn select_support(&mut self, model: ModelId, support: SupportId) {
        let key = SelectedSupport::new(model, support);
        self.supports.insert(key);
        self.last = Some(key);
    }

    /// Selects just the clicked support, or with `shift` toggles it in the
    /// selection.
    pub fn support_clicked(&mut self, model: ModelId, support: SupportId, shift: bool) {
        let key = SelectedSupport::new(model, support);

        if !shift {
            self.supports.clear();
            self.supports.insert(key);
        } else if self.supports.contains(&key) {
            self.supports.remove(&key);
            return;
        } else {
            self.supports.insert(key);
        }
        self.last = Some(key);
    }

    /// Keeps only the supports for which `exists` holds.
    pub fn retain(&mut self, mut exists: impl FnMut(ModelId, SupportId) -> bool) {
        self.supports.retain(|x| exists(x.model, x.support));
    }
}

impl Default for SelectedPrinter {
    fn default() -> Self {
        SelectedPrinter::Preset(0, 0)
    }
}
