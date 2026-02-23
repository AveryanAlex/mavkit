use pyo3::prelude::*;

macro_rules! py_enum_convert {
    ($py:ty, $rs:ty, [$($v:ident),+ $(,)?]) => {
        impl From<$rs> for $py {
            fn from(v: $rs) -> Self {
                match v {
                    $(<$rs>::$v => <$py>::$v,)+
                }
            }
        }
        impl From<$py> for $rs {
            fn from(v: $py) -> Self {
                match v {
                    $(<$py>::$v => <$rs>::$v,)+
                }
            }
        }
    };
}

// --- State enums ---

#[pyclass(name = "SystemStatus", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PySystemStatus {
    Unknown,
    Boot,
    Calibrating,
    Standby,
    Active,
    Critical,
    Emergency,
    Poweroff,
}

py_enum_convert!(
    PySystemStatus,
    mavkit::SystemStatus,
    [
        Unknown,
        Boot,
        Calibrating,
        Standby,
        Active,
        Critical,
        Emergency,
        Poweroff,
    ]
);

#[pyclass(name = "VehicleType", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyVehicleType {
    Unknown,
    FixedWing,
    Quadrotor,
    Hexarotor,
    Octorotor,
    Tricopter,
    Helicopter,
    Coaxial,
    GroundRover,
    Generic,
}

py_enum_convert!(
    PyVehicleType,
    mavkit::VehicleType,
    [
        Unknown,
        FixedWing,
        Quadrotor,
        Hexarotor,
        Octorotor,
        Tricopter,
        Helicopter,
        Coaxial,
        GroundRover,
        Generic,
    ]
);

#[pyclass(name = "AutopilotType", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyAutopilotType {
    Unknown,
    Generic,
    ArduPilotMega,
    Px4,
}

py_enum_convert!(
    PyAutopilotType,
    mavkit::AutopilotType,
    [Unknown, Generic, ArduPilotMega, Px4,]
);

#[pyclass(name = "GpsFixType", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyGpsFixType {
    NoFix,
    Fix2d,
    Fix3d,
    Dgps,
    RtkFloat,
    RtkFixed,
}

py_enum_convert!(
    PyGpsFixType,
    mavkit::GpsFixType,
    [NoFix, Fix2d, Fix3d, Dgps, RtkFloat, RtkFixed,]
);

// --- Mission enums ---

#[pyclass(name = "MissionType", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyMissionType {
    Mission,
    Fence,
    Rally,
}

py_enum_convert!(PyMissionType, mavkit::MissionType, [Mission, Fence, Rally]);

#[pyclass(name = "MissionFrame", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyMissionFrame {
    Mission,
    GlobalInt,
    GlobalRelativeAltInt,
    GlobalTerrainAltInt,
    LocalNed,
    Other,
}

py_enum_convert!(
    PyMissionFrame,
    mavkit::MissionFrame,
    [
        Mission,
        GlobalInt,
        GlobalRelativeAltInt,
        GlobalTerrainAltInt,
        LocalNed,
        Other,
    ]
);

#[pyclass(name = "IssueSeverity", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyIssueSeverity {
    Error,
    Warning,
}

py_enum_convert!(PyIssueSeverity, mavkit::IssueSeverity, [Error, Warning]);

// --- Transfer enums ---

#[pyclass(name = "TransferDirection", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyTransferDirection {
    Upload,
    Download,
}

py_enum_convert!(
    PyTransferDirection,
    mavkit::TransferDirection,
    [Upload, Download]
);

#[pyclass(name = "TransferPhase", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyTransferPhase {
    Idle,
    RequestCount,
    TransferItems,
    AwaitAck,
    Completed,
    Failed,
    Cancelled,
}

py_enum_convert!(
    PyTransferPhase,
    mavkit::TransferPhase,
    [
        Idle,
        RequestCount,
        TransferItems,
        AwaitAck,
        Completed,
        Failed,
        Cancelled,
    ]
);

// --- Param enums ---

#[pyclass(name = "ParamTransferPhase", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyParamTransferPhase {
    Idle,
    Downloading,
    Writing,
    Completed,
    Failed,
}

py_enum_convert!(
    PyParamTransferPhase,
    mavkit::ParamTransferPhase,
    [Idle, Downloading, Writing, Completed, Failed,]
);

#[pyclass(name = "ParamType", eq, frozen, from_py_object)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PyParamType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Real32,
}

py_enum_convert!(
    PyParamType,
    mavkit::ParamType,
    [Uint8, Int8, Uint16, Int16, Uint32, Int32, Real32,]
);
