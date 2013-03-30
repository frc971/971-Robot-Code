require "socket"
$header = File.open("header.bmp") { |f| f.read(54)} 


sock = File.open("output.bmp_stream")
#TCPSocket.new(ARGV[0] || "fitpc",8020)



require "rubygems"
require "gtk2"


$image = Gtk::Image.new()
$window = Gtk::Window.new()
$window.add($image)
$window.show_all()
$window.signal_connect("delete-event") { Gtk.main_quit}

loader = Gdk::PixbufLoader.new
loader.write($header)
data = sock.read(320 * 240 * 3)
loader.last_write(data)
loader.close
$image.pixbuf = loader.pixbuf
$oldtime = Time.now
Gtk.timeout_add(1000) do 
	loader = Gdk::PixbufLoader.new
	loader.write($header)
	data = sock.read(320 * 240 * 3)
#	(640 * 480).times do |i| #BGR -> RGB
#		b,g,r = data[i * 3 + 0],data[i * 3 + 1],data[i * 3 + 2]
#		data[i * 3 + 0],data[i * 3 + 1],data[i * 3 + 2] = r,g,b
#	end
	loader.last_write(data)
	loader.close
	new_time = Time.now()
	puts 1.0 / (new_time - $oldtime)
	$oldtime = new_time
	$image.pixbuf = loader.pixbuf
end

Gtk.main()
