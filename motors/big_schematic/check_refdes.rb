#!/usr/bin/env ruby

require './gschem_file'
require 'pp'

if ARGV.size == 0
  filenames = Dir.glob('**/*.{sch,sym}')
else
  filenames = get_schematic_filenames ARGV
end

seen = {}

filenames.each do |filename|
  file = GschemSchematic.new(filename)

  file.components.each do |component|
    refdes = component.refdes
    slot = component[:slot]
    if !refdes
      if !component.is_power
        puts "Warning: #{component.inspect} does not have a refdes."
      end
    elsif !seen.has_key? refdes
      seen[refdes] = [[filename, slot]]
    else
      if seen[refdes][0][1] == nil && slot == nil
        puts "Error: duplicate unslotted component #{refdes} in #{seen[refdes].collect { |a| a[0] }.inspect}."
      elsif (seen[refdes][0][1] == nil) != (slot == nil)
        puts "Error: slotted and unslotted component #{refdes} at #{seen[refdes].collect { |a| "#{a[1] || 'none'} in #{a[0]}" }.inspect}."
      else
        seen[refdes].each do |_, s|
          if s == slot
            puts "Error: duplicate slotted component #{refdes}:#{slot} in #{seen[refdes].collect { |a| a[0] if a[1] == slot }.compact.inspect}."
            break
          end
        end
      end
      seen[refdes].push [filename, slot]
    end
  end
end
