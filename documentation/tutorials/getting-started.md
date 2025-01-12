# Getting Started

To get started with programming in our codebase you'll need to setup access with the buildserver, gerrit, and buildkite.
this guide walks you through both.

* [Setting up Gerrit](#setting-up-gerrit)
* [Setting up the code](#setting-up-the-code)
    * [Buildserver](#buildserver)
    * [Locally](#locally)
* [Running the code](#running-the-code)
    * [Bazel commands for building, testing, and deploying the code](#bazel-commands-for-building-testing-and-deploying-the-code)

To access our code you will need to request access to both Gerrit and the Build Server.
Gerrit is a tool we use to manage git patches similar to github.
While the Build Server is required in order to run code on non-linux machines*.

\* It is possible to run the code locally on windows through WSL

## Setting up Gerrit
To setup Gerrit you can follow these steps
1. Fill out the 971 system access request form pinned in the `#coding` channel in slack.
    1. You need to have already signed up for the team and fully filled out all forms, as well as have passed the safety test.
        1. [More information on signing up](https://frc971.org/join)
2. Wait for Stephan Massalt to see your request and slack you with your credentials.
3. Add your email address when first signing into Gerrit. [Link](https://software.frc971.org/gerrit/settings/#Profile)

The process for buildkite is the same as the one for gerrit, except select buildkite in the system access request form.

## Setting up the code

Now that gerrit is set up you'll need a way to run the code, if you're on linux you can run it [locally](#locally).
While on Mac and Windows you'll need to use the [buildserver](#buildserver).

### Buildserver

To setup the buildserver you can follow these steps
1. Generate an SSH Key to use with the buildserver
    1. This is needed in order to connect to the server
    2. **Note:** Powershell is needed for windows on this step
    ```bash
    ssh-keygen -t ed25519 -f ~/.ssh/id_971_ed25519
    chmod 600 ~/.ssh/id_971_ed25519
    ```
2. Fill out the 971 system access form pinned in the `#coding` channel in slack
    1. This has the same requirements as the form for Gerrit
    2. You will need to run `cat ~/.ssh/id_971_ed25519.pub` for the key used in the form. This is your public key, you may share this, however `~/.ssh/id_971_ed25519` is a private key. Never share your private key.

3. Wait for a slack message from Stephan Massalt saying you have access to the Buildserver

4. Setting up an SSH aliases
    1. This makes it easier to access the buildserver with IDEs like vscode.
    2. **Note:** Similar to earlier, Powershell is required for this step on windows.
        1. You need to place these contents into `~/.ssh/config`
        ```bash
        Host frc971
          Port 2222
          ForwardAgent yes
          IdentitiesOnly yes
          User [YOUR_GERRIT_USERNAME]
          HostName build.frc971.org
          IdentityFile ~/.ssh/id_971_ed25519
        ```
    3. Run
    ```bash
    ssh build "echo If you\'re seeing this, it means everything setup correctly"
    ```
    and if you aren't seeing any errors, it has been setup correctly. 
5. Connecting to the build server
    1. You can connect either through vscode ([instructions](setup-ssh-vscode.md)) or manually via the terminal by conecting with `ssh frc971`


### Locally

These steps assume you are on the most up to date debian linux, however they can be adapted to most other distributions.

1. Updating your system
    1. This can be done by running `sudo apt update && sudo apt upgrade`
2. Installing packages needed for development
    1. `sudo apt install git python3`
3. Installing bazel
    1. Follow the steps [here](https://bazel.build/install/ubuntu)

## Running the Code

To run the code you'll need to download the repository from [gerrit](https://software.frc971.org/gerrit/admin/repos/971-Robot-Code), make sure to select ssh and not http. Click on SSH, and clone with commit message hook. Copy the command, and run it locally on terminal.
To learn more about git, open a terminal and run `man git`, or see [git(1)](https://manpages.debian.org/buster/git-man/git.1.en.html) (especially the NOTES section).

Once the repositoy is selected you'll want to make sure to configure your name, email on git. This is required to ensure you're following the [contributing guidelines above](#contributing). You can do this by running these following commands:
```bash
cd 971-Robot-Code
git config user.email "<YOUR_EMAIL_HERE>"
git config user.name "<YOUR_NAME>"
```
We use [Bazel](http://bazel.io) to build the code. Bazel has [extensive docs](https://docs.bazel.build/versions/master/build-ref.html), including a nice [build encyclopedia reference](https://docs.bazel.build/versions/master/be/overview.html), and does a great job with fast, correct incremental rebuilds.

### Bazel commands for building, testing, and deploying the code
  * Build and test everything (on the host system, for the roborio target-- note, this may take a while):
```
bazel test //...
bazel build --config=roborio -c opt //...
```
  * Build the code for a specific robot:
```console
# For the roborio:
bazel build --config=roborio -c opt //y2020/...
```
```
# For the raspberry pi:
bazel build --config=armv7 -c opt //y2020/...
```

  * Configuring a roborio: Freshly imaged roboRIOs need to be configured to run the 971 code
at startup.  This is done by using the setup_roborio.sh script.
```console
bazel run -c opt //frc971/config:setup_roborio -- roboRIO-XXX-frc.local
```

  * Download code to a robot:
```console
# For the roborio
bazel run --config=roborio -c opt //y2020:download_stripped -- roboRIO-971-frc.local
```
This assumes the roborio is reachable at `roboRIO-971-frc.local`.  If that does not work, you can try with a static IP address like `10.9.71.2` (see troubleshooting below)
```console
# For the raspberry pi's
bazel run --config=armv7 -c opt //y2020:pi_download_stripped -- 10.9.71.101
```
NOTE:
  1. The raspberry pi's require that you have your ssh key installed on them in order to copy code over
  2. They are configured to use the IP addresses 10.X.71.Y, where X is 9, 79, 89, or 99 depending on the robot number (971, 7971, 8971, or 9971, respectively), and Y is 101, 102, etc for pi #1, #2, etc.

  * Downloading specific targets to the robot
    1. Generally if you want to update what's running on the robot, you can use the `download_stripped` (or `pi_download_stripped`) targets.  These will rsync only the changed files, and so are pretty efficient.
    2. If you have a need to compile a specific module, you can build stripped versions of the individual modules by adding "_stripped" to the module name.  For example, to build the calibration code (`//y2020/vision:calibration`) for the pi (`armv7`), run:
    ```console
    bazel run --config=armv7 -c opt //y2020/vision:calibration_stripped
    ```
    You will then need to manually copy the resulting file over to the robot.

