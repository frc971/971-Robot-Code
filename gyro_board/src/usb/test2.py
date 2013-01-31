import pyudev
context = pyudev.Context()

devices = context.list_devices()
print devices
