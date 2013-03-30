require "socket"

require "bmp"
sock = TCPSocket.new(ARGV[0] || "fitpc",8020)
$width = 640 / 2
$height = 480 / 2
$header = BMPHeader.new($width,$height).text()



require "rubygems"
require "gtk2"


$image = Gtk::Image.new()
$window = Gtk::Window.new()
$window.add($image)
$window.show_all()
$window.signal_connect("delete-event") { Gtk.main_quit}

loader = Gdk::PixbufLoader.new
loader.write($header)
data = sock.read($width * $height * 3)
loader.last_write(data)
loader.close
$image.pixbuf = loader.pixbuf
$oldtime = Time.now
i = 0
Gtk.idle_add do
	loader = Gdk::PixbufLoader.new
	loader.write($header)
	data = sock.read($width * $height * 3)
#	(640 * 480).times do |i| #BGR -> RGB
#		b,g,r = data[i * 3 + 0],data[i * 3 + 1],data[i * 3 + 2]
#		data[i * 3 + 0],data[i * 3 + 1],data[i * 3 + 2] = r,g,b
#	end
	loader.last_write(data)
	loader.close
	new_time = Time.now()
	puts 1 / (new_time - $oldtime)
	$oldtime = new_time
	$image.pixbuf = loader.pixbuf
end

Gtk.main()
