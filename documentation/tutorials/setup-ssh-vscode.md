# Setting up access the build server using ssh on vscode

## Prequisites
1. Installed vscode, more info at: https://code.visualstudio.com/

2. Have credentials to access the build server, this should include ssh keys and the ssh config file for the build server: [Setting up access to a workspace on the build server](../../README.md#Setting-up-access-to-a-workspace-on-the-build-server)

## Setting up ssh config
1. Paste this into your `~/.ssh/config` file, this assumes that your private key is named `id_ed25519`

```
Host frc971
    HostName build.frc971.org
    User <SVN username>
    Port 2222
    IdentityFile ~/.ssh/id_ed25519
```

## Configuring vscode
1. Open vscode.

2. Navigate to the extensions menu and search for `Remote Development`. This should be authored by Microsoft.

3. Install `Remote Development`

4. At the bottom-left of your screen, you should see a button with two arrows pointing at each other, click it.

5. Click `Connect to Host...` and then `frc971`

6. Click `Open Folder` and then select `971-ROBOT-CODE`

## Useful extensions
- GitLens by Eric Amodio
- C/C++ by Microsoft
- Live Share by Microsoft
