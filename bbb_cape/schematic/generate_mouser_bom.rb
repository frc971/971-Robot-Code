#!/usr/bin/env ruby

# This generates something designed to be copied into Mouser's BOM creation
# copy/paste box. It turns out that it's usually easier to just manually add
# everything to your cart with the "EZBuy" thingie, but it's still a reasonable
# format.
#
# Usage: generate_mouser_bom.rb FILE [COPIES]

lines = File.open(ARGV[0]) do |f|
  lines = f.readlines
  lines.shift
  lines.collect do |line|
    line.split(', ')
  end
end

$parts = {}

def print_part(pn_string)
	#puts pn + '|1'
	pn = pn_string.intern
	if $parts[pn]
		$parts[pn] = $parts[pn] + 1
	else
		$parts[pn] = 1
	end
end

lines.each do |line|
	pn = line[4]
	if pn.index(';')
		parts = pn.split('; ')
		parts.each do |part_string|
			part = part_string.match(/(.+) x([0-9]+)?/)
			if part
				name = part[1]
				number = part[2]
				number.to_i.times do
					print_part name
				end
			else
				print_part part_string
			end
		end
	else
		print_part pn unless pn.empty?
	end
end

times = (ARGV[1] || 1).to_i
$parts.each do |pn, number|
	puts "#{pn}|#{number * times}"
end
