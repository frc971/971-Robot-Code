Deploying the scouting application
================================================================================
The scouting application is deployed to `scouting.frc971.org` via `bazel`:
```console
$ bazel run //scouting/deploy
(Reading database ... 119978 files and directories currently installed.)
Preparing to unpack .../frc971-scouting-server_1_amd64.deb ...
Removed /etc/systemd/system/multi-user.target.wants/scouting.service.
Unpacking frc971-scouting-server (1) over (1) ...
Setting up frc971-scouting-server (1) ...
Created symlink /etc/systemd/system/multi-user.target.wants/scouting.service → /etc/systemd/system/scouting.service.
Connection to scouting.frc971.org closed.
```

You will need SSH access to the scouting server. You can customize the SSH host
with the `--host` argument.

The Blue Alliance API key
--------------------------------------------------------------------------------
You need to set up an API key on the scouting server so that the scraping logic
can use it. It needs to live in `/var/frc971/scouting/tba_config.json` and look
as follows:
```json
{
    "api_key": "..."
}
```

Starting and stopping the application
--------------------------------------------------------------------------------
When you SSH into the scouting server, use `systemctl` to manage
`scouting.service` like any other service.
```console
$ sudo systemctl stop scouting.service
$ sudo systemctl start scouting.service
$ sudo systemctl restart scouting.service
```
