use std::net::Ipv4Addr;

use anyhow::{Result, anyhow};

use remote_print::manager::RemotePrintManager;
use tracing::trace;

use crate::{
    core::state::RemotePrintConnectStatus,
    task::{PollResult, Task, TaskApp, thread::TaskThread},
};

pub struct PrinterConnect {
    handle: TaskThread<Result<()>>,
}

pub struct PrinterScan {
    handle: TaskThread<Result<()>>,
}

impl PrinterConnect {
    pub fn new(remote_print: &RemotePrintManager, address: Ipv4Addr) -> Self {
        let handle = match remote_print.inner() {
            Some(inner) => TaskThread::spawn(move || inner.add_printer(address)),
            None => TaskThread::complete(Err(anyhow!("Remote print isn't running."))),
        };

        Self { handle }
    }
}

impl PrinterScan {
    pub fn new(remote_print: &RemotePrintManager, broadcast: Ipv4Addr) -> Self {
        let handle = match remote_print.inner() {
            Some(inner) => TaskThread::spawn(move || inner.scan(broadcast)),
            None => TaskThread::complete(Err(anyhow!("Remote print isn't running."))),
        };

        Self { handle }
    }
}

impl Task for PrinterConnect {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll_result(app, "Failed to Add Printer")
            .into_poll_result(|_| {
                let state = &mut app.state;
                state.remote_print_connecting = RemotePrintConnectStatus::None;
                state.working_address.clear();
                PollResult::complete()
            })
    }
}

impl Task for PrinterScan {
    fn poll(&mut self, app: &mut TaskApp) -> PollResult {
        self.handle
            .poll_result(app, "Failed to Scan for Printers")
            .into_poll_result(|_| {
                trace!("Finished scanning for printers");
                app.state.remote_print_connecting = RemotePrintConnectStatus::None;
                PollResult::complete()
            })
    }
}
