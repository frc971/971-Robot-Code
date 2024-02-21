# Roborio GDB

The roborio has a gdb-server command so we need to set the rio as our server and connect to it with some host device.

## Steps

### Rio
This command will start the gdb-server program on the rio.
```bash
gdbserver localhost:8898 program
```

### Host
```bash
# this is the gdb-multiarch package on debian. Its needed because your host machine
# likely isn't using the architecture the rio uses.
gdb-multiarch
# This connects us using an extended remote to the roborio, which lets us use the run command.
target extended-remote ip:8898
# Here we specify the name of program from the rio side
set remote exec-file program
# This makes it so we don't stop when getting realtime signals
handle SIG33 pass nostop noprint
# From here on you can use gdb like you normally would
# So we can run our program with some args
r args
# To exit the gdb-server program on the rio side we need to run this command.
monitor exit
```
