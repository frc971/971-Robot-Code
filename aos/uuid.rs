autocxx::include_cpp! (
#include "aos/uuid.h"
#include "aos/uuid_for_rust.h"

safety!(unsafe)

generate_pod!("aos::UUID")
);

pub use ffi::aos::UUID;

impl From<UUID> for uuid::Uuid {
    fn from(uuid: UUID) -> uuid::Uuid {
        uuid::Uuid::from_bytes(uuid.data)
    }
}

impl AsRef<uuid::Uuid> for UUID {
    fn as_ref(&self) -> &uuid::Uuid {
        uuid::Uuid::from_bytes_ref(&self.data)
    }
}

impl From<uuid::Uuid> for UUID {
    fn from(uuid: uuid::Uuid) -> UUID {
        UUID {
            data: *uuid.as_bytes(),
        }
    }
}
