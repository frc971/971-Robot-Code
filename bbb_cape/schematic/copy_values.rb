#!/usr/bin/env ruby

system 'rm temp.*'
system 'gsch2pcb cape.gsch2pcb -o temp > /dev/null 2>&1'

$elements = {}

ElementRegex = /[ \t]*(Element[\[(].+) "(.*)" "(.*)" "(.*)" (.*)/

File.open('temp.pcb') do |f|
	f.readlines.each do |temp_element|
		match = ElementRegex.match(temp_element)
		if match
			refdes = match[3]
			value = match[4]
			$elements[refdes] = value
		end
	end
end

File.open('cape.pcb') do |f|
	$start_lines = f.readlines
end

File.open('cape.pcb', 'w') do |f|
	$start_lines.each do |line|
		match = ElementRegex.match(line)
		if match
			footprint = match[2]
			refdes = match[3]
			value = match[4]
			f.puts "#{match[1]} \"#{footprint}\" \"#{refdes}\" \"#{$elements[refdes] || value}\" #{match[5]}"
		else
			f.puts line
		end
	end
end
