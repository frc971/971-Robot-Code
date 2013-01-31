#!/usr/bin/ruby

device_number = 0
max_device_number = 1
device = nil
while !(device)
	begin
		device = File.open("/dev/aschuh#{device_number}","r+")
	rescue
		puts("Opening /dev/aschuh#{device_number} failed")
		sleep(0.2)
		device_number += 1
		if(device_number == max_device_number)
			device_number = 0
		end
	end
end

# Set the device in debug mode to view debug prints.
device.ioctl(1,254)
while true
	print device.read(1)
end
