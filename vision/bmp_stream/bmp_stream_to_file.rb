require "socket"
$header = File.open("header.bmp") { |f| f.read(54)} 


sock = TCPSocket.new(ARGV[0] || "fitpc",8020)



require "rubygems"
require "gtk2"



File.open("output.bmp_stream","w+") do |file|
	400.times do |i|
		data = sock.read(320 * 240 * 3)
		file.print(data)
		puts "frame: #{i}"
	end
end
