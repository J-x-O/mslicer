use std::{fs::File, io::Write, path::PathBuf, sync::Arc};

use anyhow::{Context, Result};
use clone_macro::clone;
use common::{
    fs::write_atomic,
    progress::{CombinedProgress, Progress},
    serde::DynamicSerializer,
    slice::{
        Layer, SliceConfig,
        format::{Format, RasterFormat},
    },
};
use image::RgbaImage;

use crate::{
    core::slice::{SliceOperation, result::GenericSliceData},
    interface::panels::Tab,
    task::{PollResult, Task, TaskApp, TaskStatus, thread::TaskThread},
};

pub struct SaveSliced {
    progress: CombinedProgress<2>,
    file_name: String,
    handle: TaskThread<Result<()>>,
}

impl SaveSliced {
    pub fn new(
        (format, file, config, preview): (Format, GenericSliceData, SliceConfig, Arc<RgbaImage>),
        path: PathBuf,
    ) -> Self {
        let progress = CombinedProgress::new();
        let file_name = super::file_name(&path);
        let handle = TaskThread::spawn(clone!([progress], move || {
            let file = file.file(&progress[0], &config, &preview, format)?;

            let mut serializer = DynamicSerializer::new();
            file.serialize(&mut serializer, &progress[1]);
            let bytes = serializer.into_inner();
            write_atomic(&path, |out| out.write_all(&bytes))
                .with_context(|| format!("Couldn't save `{}`", path.display()))
        }));
        SaveSliced {
            progress,
            file_name,
            handle,
        }
    }
}

impl Task for SaveSliced {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll_result(app, "Failed to Write Slice Result")
            .into_poll_result(|_| PollResult::complete())
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Writing Slice Result".into(),
            details: Some(format!("Saving to {}", self.file_name)),
            progress: self.progress.progress(),
        })
    }
}

/// What a sliced file gives: its settings, layers and previews.
type Loaded = (SliceConfig, Vec<Layer>, Vec<RgbaImage>);

pub struct LoadSliced {
    progress: Progress,
    operation: Option<SliceOperation>,
    handle: TaskThread<Result<Loaded>>,
}

impl LoadSliced {
    pub fn new(path: PathBuf) -> Self {
        let progress = Progress::new();
        let operation = SliceOperation::new(
            Progress::already_complete(),
            CombinedProgress::already_complete(),
            None,
        );

        let handle = TaskThread::spawn(clone!([progress], move || {
            let ext = path.extension().unwrap_or_default().to_string_lossy();
            let format = RasterFormat::from_extension(&ext).with_context(|| {
                format!("`{}` isn't a sliced format mslicer reads", path.display())
            })?;

            let file =
                File::open(&path).with_context(|| format!("Couldn't open `{}`", path.display()))?;
            let file = slicer::util::load_sliced(&format, file)
                .with_context(|| format!("Couldn't read `{}`", path.display()))?;
            Ok((file.slice_config(), file.layers(&progress), file.previews()))
        }));

        Self {
            progress,
            operation: Some(operation),
            handle,
        }
    }
}

impl Task for LoadSliced {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll_result(app, "Failed to Load Sliced File")
            .into_poll_result(|(config, layers, image)| {
                let Some(operation) = self.operation.take() else {
                    return PollResult::complete();
                };
                operation.add_raster_result(config, layers, None);
                (image.into_iter()).for_each(|x| operation.add_preview(x));
                operation.set_loaded();

                app.slice_operation.replace(operation);
                app.panels.focus_tab(Tab::Sliced);

                PollResult::complete()
            })
    }

    fn status(&self) -> Option<TaskStatus<'_>> {
        Some(TaskStatus {
            name: "Load Sliced".into(),
            details: None,
            progress: self.progress.progress(),
        })
    }
}
