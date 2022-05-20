//! A tool that postprocesses cargo-raze output to do what we want.
//!
//! Currently this is limited to removing extraneous BUILD files which cargo-raze places in all the
//! third_party packages we feed to it, which we don't want to use. We're hand-writing BUILD files
//! for these dependencies, without intending them to be used as separate Bazel workspaces, so it's
//! easiest for them to all reference the top-level //third_party/cargo package.
use std::{
    env, fs,
    io::{self, ErrorKind},
    path::Path,
};

use anyhow::Context;
use toml::Value;

fn filter_not_found(result: io::Result<()>) -> io::Result<()> {
    match result {
        Err(e) if matches!(e.kind(), ErrorKind::NotFound) => Ok(()),
        r => r,
    }
}

fn main() -> anyhow::Result<()> {
    let argv: Vec<_> = env::args().collect();
    let workspace_path = Path::new(&argv[1]);
    let cargo_toml_path = workspace_path.join("Cargo.toml");
    eprintln!("Loading Cargo.toml from {:?}", cargo_toml_path);
    let cargo_toml_contents = fs::read(&cargo_toml_path)
        .with_context(|| format!("Failed to read Cargo.toml: {:?}", cargo_toml_path))?;
    let cargo_toml_contents = std::str::from_utf8(&cargo_toml_contents).with_context(|| {
        format!(
            "Failed to interpret Cargo.toml contents as UTF-8: {:?}",
            cargo_toml_path
        )
    })?;
    let cargo_toml: Value = cargo_toml_contents
        .parse()
        .with_context(|| format!("Failed to parse Cargo.toml contents: {:?}", cargo_toml_path))?;

    let package_aliases_dir = cargo_toml["workspace"]["metadata"]["raze"]["package_aliases_dir"]
        .as_str()
        .with_context(|| {
            format!(
                "Found non-string package_aliases_dir in Cargo.toml: {:?}",
                cargo_toml_path
            )
        })?;

    let workspace_members = cargo_toml["workspace"]["members"]
        .as_array()
        .with_context(|| {
            format!(
                "Did not find workspace members in Cargo.toml: {:?}",
                cargo_toml_path
            )
        })?;
    for member in workspace_members.iter() {
        let member = member.as_str().with_context(|| {
            format!(
                "Found non-string workspace member in Cargo.toml: {:?}",
                cargo_toml_path
            )
        })?;

        // First delete the BUILD file.
        let member_build = workspace_path
            .join(member)
            .join(package_aliases_dir)
            .join("BUILD.bazel");
        filter_not_found(fs::remove_file(&member_build)).with_context(|| {
            format!(
                "Failed to remove workspace member BUILD.bazel: {:?}",
                member_build
            )
        })?;

        // Then go and delete each folder in reverse order, but only if it's now empty to avoid
        // overeager deletion. The file we're deleting should be the only thing in these folders,
        // but if somebody wrote something else for some reason then we don't want to silently
        // delete it.
        let mut folder_path = workspace_path.join(member);
        for d in Path::new(package_aliases_dir)
            .components()
            .scan(&mut folder_path, |a, b| {
                **a = a.join(b);
                Some(a.clone())
            })
            .collect::<Vec<_>>()
            .iter()
            .rev()
        {
            filter_not_found(fs::remove_dir(d)).with_context(|| {
                format!(
                    "Failed to remove workspace member package_aliases directory: {:?}",
                    d
                )
            })?;
        }
    }
    eprintln!("All done");
    Ok(())
}
