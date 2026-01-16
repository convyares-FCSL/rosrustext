use std::{error::Error as StdError, fmt};

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    #[cfg(feature = "ros2")]
    Rclrs(rclrs::RclrsError),

    Core(rosrustext_core::error::CoreError),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            #[cfg(feature = "ros2")]
            Error::Rclrs(e) => write!(f, "rclrs error: {e}"),

            Error::Core(e) => write!(f, "core error: {e}"),
        }
    }
}

impl StdError for Error {
    fn source(&self) -> Option<&(dyn StdError + 'static)> {
        match self {
            #[cfg(feature = "ros2")]
            Error::Rclrs(e) => Some(e),

            Error::Core(e) => Some(e),
        }
    }
}

#[cfg(feature = "ros2")]
impl From<rclrs::RclrsError> for Error {
    fn from(e: rclrs::RclrsError) -> Self {
        Error::Rclrs(e)
    }
}

impl From<rosrustext_core::error::CoreError> for Error {
    fn from(e: rosrustext_core::error::CoreError) -> Self {
        Error::Core(e)
    }
}
