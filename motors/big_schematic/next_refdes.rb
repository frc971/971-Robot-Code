#!/usr/bin/env ruby

require './gschem_file'

if ARGV.size < 2
  puts "Usage: next_refdes.rb file.sch BASE [quantity] [start]"
  exit 1
end

filenames = ARGV.select do |name|
  name.include? '.'
end
leftover = ARGV - filenames

refdes_pattern = /^refdes=(.+)$/
this_pattern = /^#{leftover[0]}(\d+)$/
used = []

get_schematic_filenames(filenames).each do |name|
  File.open(name, 'r') do |f|
    f.readlines.each do |line|
      match = refdes_pattern.match(line)
      if match
        refdes = this_pattern.match(match[1])
        if refdes
          used.push(refdes[1].to_i)
        end
      end
    end
  end
end

if leftover.length > 1
  todo = leftover[1].to_i
else
  todo = 1
end

if leftover.length > 2
  start = leftover[2].to_i
else
  start = 1
end

i = start
while todo > 0
  if !used.include?(i)
	puts "#{leftover[0]}#{i}"
    todo -= 1
  end
  i += 1
end
