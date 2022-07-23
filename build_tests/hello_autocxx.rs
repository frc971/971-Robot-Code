use autocxx::include_cpp;

include_cpp! (
#include "build_tests/hello_autocxx.h"
generate!("plain_function")
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plain_function() {
        assert_eq!(autocxx::c_int::from(971), unsafe { ffi::plain_function() });
    }
}
