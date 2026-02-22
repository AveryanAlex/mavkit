pub mod file;
pub mod types;

pub use file::{format_param_file, parse_param_file};
pub use types::{Param, ParamProgress, ParamStore, ParamTransferPhase, ParamType, ParamWriteResult};

use crate::error::VehicleError;
use crate::Vehicle;

/// Handle to parameter operations on a `Vehicle`.
pub struct ParamsHandle<'a> {
    vehicle: &'a Vehicle,
}

impl<'a> ParamsHandle<'a> {
    pub(crate) fn new(vehicle: &'a Vehicle) -> Self {
        Self { vehicle }
    }

    pub async fn download_all(&self) -> Result<ParamStore, VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::ParamDownloadAll { reply })
            .await
    }

    pub async fn write(&self, name: String, value: f32) -> Result<Param, VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::ParamWrite {
                name,
                value,
                reply,
            })
            .await
    }

    pub async fn write_batch(
        &self,
        params: Vec<(String, f32)>,
    ) -> Result<Vec<types::ParamWriteResult>, VehicleError> {
        self.vehicle
            .send_command(|reply| crate::command::Command::ParamWriteBatch { params, reply })
            .await
    }
}
