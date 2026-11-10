use std::path::Path;

use poll_promise::Promise;
use rfd::{AsyncFileDialog, FileHandle};

use crate::task::{PollResult, Task, TaskApp};

type Callback = Box<dyn FnOnce(&mut TaskApp, &Path, &mut Vec<Box<dyn Task>>)>;
type MultiCallback = Box<dyn FnOnce(&mut TaskApp, Vec<&Path>, &mut Vec<Box<dyn Task>>)>;

pub struct FileDialog {
    func: Option<Callback>,
    promise: Promise<Option<FileHandle>>,
}

pub struct MultiFileDialog {
    func: Option<MultiCallback>,
    promise: Promise<Option<Vec<FileHandle>>>,
}

impl FileDialog {
    fn new(
        file: impl Future<Output = Option<FileHandle>> + Send + 'static,
        callback: impl FnOnce(&mut TaskApp, &Path, &mut Vec<Box<dyn Task>>) + 'static,
    ) -> Self {
        let promise = Promise::spawn_async(file);
        Self {
            func: Some(Box::new(callback)),
            promise,
        }
    }

    pub fn pick_file(
        (name, extensions): (impl Into<String>, &[impl ToString]),
        callback: impl FnOnce(&mut TaskApp, &Path, &mut Vec<Box<dyn Task>>) + 'static,
    ) -> Self {
        let file = AsyncFileDialog::new()
            .add_filter(name, extensions)
            .pick_file();
        Self::new(file, callback)
    }

    pub fn save_file(
        (name, extensions): (impl Into<String>, &[impl ToString]),
        callback: impl FnOnce(&mut TaskApp, &Path, &mut Vec<Box<dyn Task>>) + 'static,
    ) -> Self {
        let file = AsyncFileDialog::new()
            .add_filter(name, extensions)
            .save_file();
        Self::new(file, callback)
    }
}

impl MultiFileDialog {
    fn new(
        file: impl Future<Output = Option<Vec<FileHandle>>> + Send + 'static,
        callback: impl FnOnce(&mut TaskApp, Vec<&Path>, &mut Vec<Box<dyn Task>>) + 'static,
    ) -> Self {
        let promise = Promise::spawn_async(file);
        Self {
            func: Some(Box::new(callback)),
            promise,
        }
    }

    pub fn pick_files(
        (name, extensions): (impl Into<String>, &[impl ToString]),
        callback: impl FnOnce(&mut TaskApp, Vec<&Path>, &mut Vec<Box<dyn Task>>) + 'static,
    ) -> Self {
        let file = AsyncFileDialog::new()
            .add_filter(name, extensions)
            .pick_files();
        Self::new(file, callback)
    }
}

impl Task for FileDialog {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let Some(data) = self.promise.ready() else {
            return PollResult::pending();
        };
        // Cancelled, or already handled.
        let (Some(handle), Some(func)) = (data, self.func.take()) else {
            return PollResult::complete();
        };
        let path = handle.path();
        let mut tasks = Vec::new();
        func(app, path, &mut tasks);
        PollResult::complete().with_tasks(tasks)
    }
}

impl Task for MultiFileDialog {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        let Some(data) = self.promise.ready() else {
            return PollResult::pending();
        };
        // Cancelled, or already handled.
        let (Some(handle), Some(func)) = (data, self.func.take()) else {
            return PollResult::complete();
        };
        let paths = handle.iter().map(|x| x.path()).collect();
        let mut tasks = Vec::new();
        func(app, paths, &mut tasks);
        PollResult::complete().with_tasks(tasks)
    }
}
