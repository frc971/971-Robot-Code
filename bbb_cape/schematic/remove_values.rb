#!/usr/bin/env ruby

ElementRegex = /[ \t]*(Element[\[(].+) "(.*)" "(.*)" "(.*)" (.*)/

File.open('cape.pcb') do |f|
	$start_lines = f.readlines
end

File.open('cape.pcb', 'w') do |f|
	$start_lines.each do |line|
		match = ElementRegex.match(line)
		if match
			footprint = match[2]
			refdes = match[3]
			f.puts "#{match[1]} \"#{footprint}\" \"#{refdes}\" \"\" #{match[5]}"
		else
			f.puts line
		end
	end
end
