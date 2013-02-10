require "socket"
$sock = UDPSocket.new()
$sock.connect("10.59.71.2",9123)
def short(val)
	val += 1.0
	if(val < 0)
		val = 0
	end
	val = (val * 256 * 128).to_i
	v1 = val / 256
	v2 = val % 256
	if(v1 > 255)
		v1 = 255
		v2 = 255
	end
	return(v1.chr + v2.chr)
end
def jaguar(port,val)
	$sock.send("j#{port.chr}#{short(val)}",0)
end
trap(2) do
	jaguar(4,0)
	jaguar(3,0)
	exit!
end
while true
	jaguar(4,Math.cos(Time.now.to_f))
	jaguar(3,Math.cos(Time.now.to_f))
	sleep(0.01)
end
