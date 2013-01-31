#!/usr/bin/python

global DeviceName, AddAction, RemoveAction
DeviceName = 'usb_device_46d_c408_noserial' # Logitech Trackball
AddAction = 'xmodmap -e "pointer = 3 2 1 4 5 6 7 9 8"'
RemoveAction = 'xmodmap -e "pointer = 1 2 3 4 5 6 7 8 9"'

import dbus # needed to do anything
import dbus.decorators # needed to receive messages
import dbus.glib # needed to receive messages
import gobject # needed to loop & monitor
import os # needed to 

def add_device(*args, **keywords):
    Path = args[0].split('/')
    if Path[-1] == DeviceName: # Device found
        os.system(AddAction)
        
def remove_device(*args, **keywords):
    Path = args[0].split('/')
    if Path[-1] == DeviceName: # Device found
        os.system(RemoveAction)

bus = dbus.SystemBus()  # connect to system bus
hal_manager_obj = bus.get_object('org.freedesktop.Hal', '/org/freedesktop/Hal/Manager')
hal_manager = dbus.Interface(hal_manager_obj, 'org.freedesktop.Hal.Manager')

# Add listeners for all devices being added or removed
bus.add_signal_receiver(add_device, 'DeviceAdded', 'org.freedesktop.Hal.Manager',
                        'org.freedesktop.Hal', '/org/freedesktop/Hal/Manager')
bus.add_signal_receiver(remove_device, 'DeviceRemoved', 'org.freedesktop.Hal.Manager',
                        'org.freedesktop.Hal', '/org/freedesktop/Hal/Manager')

# get list of all devices, determine if device is connected
device_names = hal_manager.GetAllDevices()
for name in device_names:
    Path = name.split('/')
    if Path[-1] == DeviceName: # Device found
        os.system(AddAction)
        break # no need to keep looking

# monitor
loop = gobject.MainLoop()
loop.run()
