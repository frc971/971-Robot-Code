# Creating a new extension

Change directories into the directory of interest.

    $ cd path/to/package

The run the creation script and pass the new directory name as an argument.

    $ ../.../tools/foxglove/create-foxglove-extension <path>

The script will automatically set up all the Bazel hooks so you can compile
the extension.

    $ bazel build //path/to/package/<path>:extension
