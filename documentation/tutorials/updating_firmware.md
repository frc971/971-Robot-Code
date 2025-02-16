# How to update ctre firmware for motors

## Update allwpilib

1. First update allwpilib in our `third_party/` folder

Run the script `./doc/allwpilib_subtree.sh`

Within the file there is instructions on how to run it and how it works.
An example of using the script with the respective `<year>` would look like:
`./doc/allwpilib_subtree.sh add third_party/allwpilib_<year> https://github.com/wpilibsuite/allwpilib main`

**NOTE:** You can not have unstaged or staged changes while this is running, everything must be committed

This should take a while but will move allwpilib into your specified folder

## Update ctre package versions in WORKSPACE

1.  Next you need to update the ctre packages in the WORKSPACE file so they are the latest version.

2.  To update the ctre package versions in the WORKSPACE file, first go to
    `https://github.com/CrossTheRoadElec/Phoenix-Releases/releases`

3.  Find the `Vendordep URL` for `General Use`, the url should look something like:
    `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json`

4.  The url is a json file with information on ctre packages. Pay attention to
    the version numbers as you will need to update these in the WORKSPACE file, as well
    as the sha256 for the files with the updated version.

    Example:

    - I see artifact `api-cpp` has a current url of:
      `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/25.2.1/api-cpp-25.2.1-headers.zip`
      in the WORKSPACE file and the latest version is `25.2.2`.

    - First rename the version number (this it how it would be in this example):
      `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/25.2.2/api-cpp-25.2.2-headers.zip`

    - Next, use `wget <url>` to download the file on your computer:
      `wget https://maven.ctr-electronics.com/release/com/ctre/phoenix6/api-cpp/25.2.2/api-cpp-25.2.2-headers.zip`

    - Once it is downloaded, you can run `sha256sum <file>` to get the sha256:
      `sha256sum api-cpp-25.2.2-headers.zip`
      (Should print out the sha256 of the file)

    - Update the url and sha256 within the WORKSPACE file for the given http_archive

    **NOTE:** This will not build with the updated urls until you follow step 6 below,
    or update Build-Dependencies (instructions below as well)

5.  Lastly, the `ctre_phoenix6_arm64` archive can't be updated using these methods,
    instead you need to create the url yourself from a tar file

    - First, with the updated `<year>` and `<version>`, use `wget` to download:
      `https://deb.ctr-electronics.com/libs/<year>/packages/phoenix6/<version>/phoenix6_<version>_arm64.deb`

      (This link was derived from the apt package with install directions in
      `https://v6.docs.ctr-electronics.com/en/2023-v6/docs/installation/installation.html` under non-FRC)

    - Then, run `dpkg -x <deb_file> phoenix6`, using the debian file you downloaded in place of `<deb_file>` this will create
      a folder called phoenix6 with the file data within it.

    - Run `tar czf phoenix6_<version>_arm64.tar.gz phoenix6` to create a tar file with the respective data in it,
      specify `<version>` to keep things consistent

    - After creating the tar file, you can use it in the `BUILD` file, for quick testing you
      can use `file://<path_to_your_tar>` to use it as a `url` you should move this to Build-Dependencies.

6.  **EXTREMELY IMPORTANT**: If you want to check the code builds after after updating the dependencies without uploading them to Build-Dependencies,
    in `tools/dependency_rewrite` you need to comment out the lines `block` and the `rewrite` for ctre:

    ```
    # block *
    # rewrite maven.ctr-electronics.com/(.*) software.frc971.org/Build-Dependencies/maven.ctr-electronics.com/$1
    ```

    If you want to upload them to Build-Dependencies use the instructions below.

## Uploading dependencies to Build-Dependencies

1. Once you are certain that everything builds and the dependency urls are correct, you can start
   adding them to Build-Dependencies using buildkite.

   - Go to the `Spartan Robotics` buildkite page and go to `Deploy Artifact`.

   - Press `New Build` and name the build including the <dependency> and the <version> you are updating it to

     Example: `Update ctre_phoenix6_tools_athena to 25.2.2`

   - Put the url of the dependency in the `Dependency Url` box, then press `Create Build`
