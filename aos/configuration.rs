use std::path::Path;

use thiserror::Error;

use aos_configuration_fbs::aos::Configuration as RustConfiguration;
use aos_flatbuffers::{Flatbuffer, NonSizePrefixedFlatbuffer};

autocxx::include_cpp! (
#include "aos/configuration.h"
#include "aos/configuration_for_rust.h"
#include "aos/configuration_generated.h"

safety!(unsafe)

generate!("aos::Configuration")
generate!("aos::Channel")
generate!("aos::Node")
block!("flatbuffers::String")
block!("flatbuffers::Verifier")

generate!("aos::configuration::GetChannelForRust")
);

#[cxx::bridge]
mod ffi2 {
    #[namespace = "aos::configuration"]
    unsafe extern "C++" {
        include!("aos/configuration_for_rust.h");
        fn MaybeReadConfigForRust(path: &str, extra_import_paths: &[&str]) -> Vec<u8>;
    }
}

pub use ffi::aos::{Channel, Configuration, Node};

#[derive(Clone, Copy, Eq, PartialEq, Debug, Error)]
pub enum ChannelLookupError {
    #[error("channel not found")]
    NotFound,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug, Error)]
pub enum ReadConfigError {
    #[error("duplicate imports or invalid paths")]
    ReadFailed,
}

impl Configuration {
    pub fn get_channel(
        &self,
        name: &str,
        typename: &str,
        application_name: &str,
        node: &Node,
    ) -> Result<&Channel, ChannelLookupError> {
        // SAFETY: All the input references are valid pointers, and we're not doing anything with
        // the result yet. It doesn't store any of the input references.
        let channel = unsafe {
            ffi::aos::configuration::GetChannelForRust(self, name, typename, application_name, node)
        };
        if channel.is_null() {
            Err(ChannelLookupError::NotFound)
        } else {
            // SAFETY: We know this is a valid pointer now, and we're returning it with the lifetime
            // inherited from `configuration` which owns it.
            Ok(unsafe { &*channel })
        }
    }
}

/// # Panics
///
/// `path` must be valid UTF-8.
pub fn read_config_from(
    path: &Path,
) -> Result<impl Flatbuffer<RustConfiguration<'static>>, ReadConfigError> {
    read_config_from_import_paths(path, &[])
}

/// # Panics
///
/// `path` and all members of `extra_import_paths` must be valid UTF-8.
pub fn read_config_from_import_paths(
    path: &Path,
    extra_import_paths: &[&Path],
) -> Result<impl Flatbuffer<RustConfiguration<'static>>, ReadConfigError> {
    let extra_import_paths: Vec<_> = extra_import_paths
        .iter()
        .map(|p| p.to_str().expect("Paths must be UTF-8"))
        .collect();
    let buffer = ffi2::MaybeReadConfigForRust(
        path.to_str().expect("Paths must be UTF-8"),
        &extra_import_paths,
    );
    if buffer.is_empty() {
        return Err(ReadConfigError::ReadFailed);
    }
    // SAFETY: The C++ code returns a valid flatbuffer (unless it returned an error, which we
    // checked above).
    return Ok(unsafe { NonSizePrefixedFlatbuffer::new_unchecked(buffer) });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_config() {
        let config = read_config_from(Path::new("aos/testdata/config1.json")).unwrap();
        assert!(
            config
                .message()
                .channels()
                .unwrap()
                .iter()
                .find(|channel| channel.type_() == Some(".aos.bar"))
                .is_some(),
            "Failed to find the .aos.bar channel: {:?}",
            config.message()
        );
    }
}
