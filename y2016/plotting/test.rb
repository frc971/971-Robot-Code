require "sinatra"

# Must use sinatra server for this because fetch() in javascript
# cannot load from the filesystem.
get '/test.html' do
	File.read("./test.html")
end

get '/responsive_plotter.js' do
	content_type 'text/javascript'
	File.read("./responsive_plotter.js")
end

get '/test_data' do
	content_type 'binary/octet-stream'
	data = ""
	# Use ./test_data if it exists. Otherwise random data.
	if (File.exists?("./test_data"))
		data = File.read("./test_data")
	else
		1000.times do
			data += (1024.times.collect do |i|
				(rand() * 480  - 160)
			end.pack("F*"))
		end
	end
	data
end

