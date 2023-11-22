use std::{
    env,
    ffi::{CString, OsStr, OsString},
    os::unix::prelude::OsStrExt,
    sync::Once,
};

use clap::{
    error::{ContextKind, ContextValue},
    Arg, ArgAction, Error, Parser,
};

autocxx::include_cpp! (
#include "aos/init_for_rust.h"

safety!(unsafe)

generate!("aos::InitFromRust")
generate!("aos::GetCppFlags")
generate!("aos::FlagInfo")
generate!("aos::SetCommandLineOption")
generate!("aos::GetCommandLineOption")
);

/// Initializes AOS.
pub fn init() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        // We leak the `CString` with `into_raw`. It's not sound for C++ to free
        // it but `InitGoogleLogging` requries that it is long-lived.
        let argv0 = std::env::args()
            .map(|arg| CString::new(arg).expect("Arg may not have NUL"))
            .next()
            .expect("Missing argv[0]?")
            .into_raw();
        // SAFETY: argv0 is a well-defined CString.
        unsafe {
            ffi::aos::InitFromRust(argv0);
        }
    });
}

/// Trait used to append C++ gFlags to a clap CLI.
pub trait WithCppFlags: Parser {
    /// Parses the comannd line arguments while also setting the C++ gFlags.
    fn parse_with_cpp_flags() -> Self {
        Self::parse_with_cpp_flags_from(env::args_os())
    }

    /// Like [`parse_with_cpp_flags`] but read from an iterator.
    fn parse_with_cpp_flags_from<I, T>(itr: I) -> Self
    where
        I: IntoIterator<Item = T>,
        T: Into<OsString> + Clone,
    {
        let cxxflags = ffi::aos::GetCppFlags();
        let cxxflags: Vec<CxxFlag> = cxxflags
            .iter()
            .map(|flag| CxxFlag::from(flag))
            .filter(|flag| flag.name != "help" && flag.name != "version")
            .collect();

        let mut command = Self::command()
            .next_help_heading("Flags from C++")
            .args(cxxflags.iter().cloned());

        let matches = command.clone().get_matches_from(itr);

        for cxxflag in cxxflags {
            let Some(mut value) = matches.get_raw(&cxxflag.name) else {
                continue;
            };
            // We grab the last match as GFlags does.
            let value = value.next_back().unwrap();
            cxxflag.set(value).unwrap_or_else(|_| {
                let mut error = Error::new(clap::error::ErrorKind::InvalidValue);

                // Let user know how they messed up.
                error.insert(
                    ContextKind::InvalidArg,
                    ContextValue::String(format!("--{}", cxxflag.name)),
                );
                error.insert(
                    ContextKind::InvalidValue,
                    ContextValue::String(
                        value
                            .to_owned()
                            .into_string()
                            .expect("Invalid UTF-8 String"),
                    ),
                );
                error.format(&mut command).exit()
            })
        }

        match Self::from_arg_matches(&matches) {
            Ok(flags) => flags,
            Err(e) => e.format(&mut command).exit(),
        }
    }
}

impl<T: Parser> WithCppFlags for T {}

#[derive(Clone)]
#[allow(unused)]
struct CxxFlag {
    name: String,
    ty: String,
    description: String,
    default_value: String,
    filename: String,
}

struct SetFlagError;

impl CxxFlag {
    /// Sets the command gFlag to the specified value.
    fn set(&self, value: &OsStr) -> Result<(), SetFlagError> {
        unsafe {
            let name = CString::new(self.name.clone()).expect("Flag name may not have NUL");
            let value = CString::new(value.as_bytes()).expect("Arg may not have NUL");
            if ffi::aos::SetCommandLineOption(name.as_ptr(), value.as_ptr()) {
                Ok(())
            } else {
                Err(SetFlagError)
            }
        }
    }

    fn get_option(name: &str) -> String {
        unsafe {
            let name = CString::new(name).expect("Flag may not have NUL");
            ffi::aos::GetCommandLineOption(name.as_ptr()).to_string()
        }
    }
}

impl From<&ffi::aos::FlagInfo> for CxxFlag {
    fn from(value: &ffi::aos::FlagInfo) -> Self {
        Self {
            name: value.name().to_string(),
            ty: value.ty().to_string(),
            description: value.description().to_string(),
            default_value: value.default_value().to_string(),
            filename: value.filename().to_string(),
        }
    }
}

impl From<CxxFlag> for Arg {
    fn from(value: CxxFlag) -> Self {
        assert_ne!(&value.name, "help");
        Arg::new(&value.name)
            .long(&value.name)
            .help(&value.description)
            .default_value(&value.default_value)
            .action(ArgAction::Set)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use super::*;

    #[derive(Parser)]
    #[command()]
    struct App {
        #[arg(long)]
        myarg: u64,
    }

    // We are sharing global state through gFlags. Use a mutex to prevent races.
    static MUTEX: Mutex<()> = Mutex::new(());

    #[test]
    fn simple_rust() {
        let _guard = MUTEX.lock();
        let app = App::parse_with_cpp_flags_from(&["mytest", "--myarg", "23"]);
        assert_eq!(app.myarg, 23);
    }

    #[test]
    fn set_cxx_flag() {
        let _guard = MUTEX.lock();
        let app = App::parse_with_cpp_flags_from(&[
            "mytest",
            "--alsologtostderr",
            "true",
            "--myarg",
            "23",
        ]);
        assert_eq!(app.myarg, 23);
        assert_eq!(CxxFlag::get_option("alsologtostderr"), "true");
    }
}
