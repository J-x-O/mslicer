use std::{iter, ops::Deref, sync::Arc, time::Instant};

use common::{
    misc::IteratorExt,
    progress::{CombinedProgress, Progress},
    slice::{Layer, SliceConfig, VectorLayer, print_time},
    units::{CubicMilimeters, Milimeters},
};
use image::RgbaImage;
use itertools::Itertools;
use parking_lot::{Mutex, MutexGuard};
use tracing::info;

use crate::{
    core::{
        slice::{
            annotations::Annotations,
            result::{RasterSliceResult, SliceResult, VectorSliceResult},
        },
        state::PreviewModel,
    },
    engine::{CancelToken, InputHash},
    render::camera::Camera,
    util::management::LazyTextureId,
};

pub mod annotations;
pub mod estimate;
pub mod key;
pub mod result;
pub mod runner;

#[derive(Clone)]
pub struct SliceOperation {
    inner: Arc<SliceOperationInner>,
}

pub struct SliceOperationInner {
    start_time: Instant,
    /// What was sliced, if it was the project ([`key::slice_key`]): the
    /// result is current while the project's key is the same.
    pub key: Option<InputHash>,
    /// Stops the slice once it is superseded.
    pub cancel: CancelToken,
    pub progress: Progress,
    pub post_processing_progress: CombinedProgress<2>,
    pub result: Mutex<Option<SliceResult>>,

    pub previews: Mutex<Option<PreviewImage>>,
    pub interactive_previews: Option<InteractivePreviews>,
}

pub struct PreviewImage {
    pub image: Arc<RgbaImage>,
    pub texture: LazyTextureId,
}

pub struct InteractivePreviews {
    pub camera: Mutex<Camera>, // hmph.
    pub models: Vec<PreviewModel>,
}

impl SliceOperation {
    pub fn new(
        slice: Progress,
        post_process: CombinedProgress<2>,
        interactive_previews: Option<InteractivePreviews>,
    ) -> Self {
        Self {
            inner: Arc::new(SliceOperationInner {
                start_time: Instant::now(),
                key: None,
                cancel: CancelToken::new(),
                progress: slice,
                post_processing_progress: post_process,
                result: Mutex::new(None),

                previews: Mutex::new(Default::default()),
                interactive_previews,
            }),
        }
    }
}

impl SliceOperation {
    /// Records which project inputs this slice is of.
    pub fn with_key(self, key: InputHash) -> Self {
        // A shared operation keeps no key, so it never counts as current.
        let mut inner = match Arc::try_unwrap(self.inner) {
            Ok(inner) => inner,
            Err(inner) => return Self { inner },
        };
        inner.key = Some(key);
        Self {
            inner: Arc::new(inner),
        }
    }
}

impl SliceOperationInner {
    pub fn needs_previews(&self) -> bool {
        self.previews.lock().is_none()
    }

    pub fn add_preview(&self, image: RgbaImage) {
        *self.previews.lock() = Some(PreviewImage {
            image: Arc::new(image),
            texture: LazyTextureId::empty(),
        });
    }

    /// The preview image, once the slice has made one.
    pub fn preview(&self) -> Option<Arc<RgbaImage>> {
        Some(self.previews.lock().as_ref()?.image.clone())
    }

    pub fn add_raster_result(
        &self,
        config: SliceConfig,
        layers: Vec<Layer>,
        triangles: Option<u64>,
    ) {
        let heights = iter::once(Milimeters::new(0.0))
            .chain(layers.iter().map(|x| x.height))
            .tuple_windows()
            .map(|(a, b)| b - a);
        let volume = (layers.iter().zip(heights))
            .map(|(l, h)| l.area as f32 * config.pixel_area() * h)
            .fold(CubicMilimeters::new(0.0), |a, b| a + b)
            .convert();

        let voxels = (layers.iter())
            .flat_map(|x| x.data.iter().filter(|x| x.value != 0).map(|x| x.length))
            .sum::<u64>();

        let elapsed = self.start_time.elapsed();
        info!("Raster slice operation completed in {:?}", elapsed);

        let variable_layer_height = !(layers.iter())
            .map(|x| x.height.raw())
            .tuple_windows()
            .map(|(a, b)| b - a)
            .all_equal_float(0.001);

        let raster = RasterSliceResult {
            voxels,
            volume,
            print_time: print_time(layers.iter()),

            layers,
            annotations: Arc::new(Annotations::default()),
            detected_islands: false,
        };

        self.result().replace(SliceResult {
            config,
            elapsed,
            fresh: true,
            sliced: true,
            triangles,

            variable_layer_height,
            inner: raster.into(),
        });
    }

    pub fn add_vector_result(
        &self,
        config: SliceConfig,
        layers: Arc<Vec<VectorLayer>>,
        triangles: Option<u64>,
    ) {
        let elapsed = self.start_time.elapsed();
        info!("Vector slice operation completed in {:?}", elapsed);

        self.result().replace(SliceResult {
            config,
            elapsed,
            fresh: true,
            sliced: true,
            triangles,

            variable_layer_height: false,
            inner: VectorSliceResult { layers }.into(),
        });
    }

    pub fn set_loaded(&self) {
        if let Some(result) = self.result().as_mut() {
            result.sliced = false;
        }
    }

    pub fn result(&self) -> MutexGuard<'_, Option<SliceResult>> {
        self.result.lock()
    }
}

impl Deref for SliceOperation {
    type Target = SliceOperationInner;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}
