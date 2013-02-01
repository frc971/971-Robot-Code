#!/usr/bin/ruby

#=~ and !~ are regex (not) match operators. they set $1 etc

Suffix = "_1.31.2013"
PartsLists = ['parts.txt']

Mouser = 1
Digikey = 2
Coilcraft = 3
Analog_com = 4
Unknown = 5

class Component
	attr_accessor :desc, :quant, :ordering, :names, :other_names, :maybe
	def source
		@source
	end
	def finish
		#puts inspect
		return false if !desc || !quant || !ordering || !names
		@other_names ||= []
		puts "warning: #{names.size + other_names.size} names (#{names.inspect} #{other_names.inspect}) but ordering #{quant} of '#{desc}'" if names.size + other_names.size != quant

		case ordering
		when /mouser/
			@source = Mouser
		when /digikey/
			@source = Digikey
		when /coilcraft/
			@source = Coilcraft
		when /analog\.com/
			@source = Analog_com
		else
			@source = Unknown
		end

		true
	end
	def inspect
		"Component{desc=>#{desc.inspect},quant=>#{quant.inspect},ordering=>#{ordering.inspect},names=>#{names.inspect},other_names=>#{other_names.inspect},maybe=#{maybe}}"
	end
	def main_quant
		names.size
	end
	def desc
		if @names && @desc && @desc =~ /webench #{@names[0].to_s.downcase} \((.*)\)/
			$1
		else
			@desc
		end
	end
	def bom
		"#{@names.join(',')}\t#{part_num}\t#{desc}"
	end
	def names=(names)
		@names = expand names
	end
	def other_names=(names)
		@other_names = expand names
	end
	def expand(names)
		r = []
		names.each do |name|
			if name =~ /^([a-zA-Z$]*)(\d+)-(\d+)([a-zA-Z$]*)$/
				$2.to_i.upto($3.to_i) do |num|
					r.push $1 + num.to_s + $4
				end
			else
				r.push name
			end
		end
		r
	end
	def part_num
		case @source
		when Mouser
			lines = `wget '#{ordering}' -O -`.lines
			if (!$?.success?)
				$stderr.puts "determing part # for #{ordering.inspect} failed #$?"
				exit(1)
			end
			found = 0
			lines.each do |line|
				if /^Mouser Part #:/ =~ line
					found = 5
				end
				if (found == 1)
					@result = line.chomp
				end
				if (found > 0)
					found -= 1
				end
			end
			@result
		when Digikey
			#`wget '#{ordering}' -O - 2>/dev/null | grep 'Digi-Key Part Number' | sed 's/.*\td id=reportpartnumber\(.*\)\<\/td.*/\1/g'`
			puts "warning: digikey ordering link #{ordering} not recognized" unless ordering =~ /.*\/(.*)\/.*$/
			$1
		when Coilcraft
			puts "warning: coilcraft product page #{ordering} not recognized" unless ordering =~ /^.* (.*)$/
			$1
		when Analog_com
			puts "warning: using hard-coded gyro part number for part(s) #{names.inspect}"
			"ADXRS450"
		else
			puts "warning: couldn't figure out how to get a part number out of #{ordering}"
		end
	end
end
class Board
	def name
		@name
	end
	def initialize(name, *sections)
		@name = name
		parse sections
	end
	def parse(sections)
		#puts sections.join(", ")
		PartsLists.each do |f|
			file = File.open(f)
			section = ''
			blank = true # last line blank
			@comps = []
			comp = nil
			file.readlines.each do |l|
				l = l.chomp
				case l
				when /^\/\/ ([a-zA-Z0-9 $.+\-,]*)$/
					#print 'matched ' + $1
					if blank
						section = $1
					elsif comp
						names = $1.split(/[, ]+/)
						if !comp.maybe || sections.include?(names.last)
							puts "warning: names of '#{$1}' before other stuff" if !comp.ordering || !comp.desc || !comp.quant
							names.pop if comp.maybe
							comp.names = names
						elsif comp.maybe && !section.include?(names.last)
							names.pop
							comp.other_names = names
						end
					end
				when /^(\d\d?) x ([a-zA-Z 0-9().\-+\/%"@,]*)$/
					if !comp && sections.include?(section) || section == 'Common'
						comp = Component.new
						comp.maybe = section == 'Common'
					end
					if comp
						puts "warning: desc of '#{$2}' and quant of '#{$1}' after ordering and/or names" if comp.ordering || comp.names
						comp.desc = $2
						comp.quant = $1.to_i
					end
				when /^\t(analog\.com|http[a-zA-Z 0-9?\/:=%-.]*)$/
					if comp
						puts "warning: ordering info of '#{$1}' before desc and/or quant" if !comp.desc || !comp.quant
						comp.ordering = $1
					end
				else
					puts "warning: no match to #{l}" if !l.empty? && l !~ /^\/\//
				end
				blank = l == ''
				if comp != nil && blank
					@comps.push comp if comp.finish
					comp = nil
				end
				#puts section
			end
			file.close
		end
	end
	def write_bom
		File.open(@name + Suffix + '/' + @name + '.parts_mapping.txt', 'w+') do |f|
			@comps.each do |comp|
				f.puts comp.bom
			end
		end
	end
	def check # makes sure everything in the parts list is in the bom
		parsed = []
		File.open(@name + Suffix + '/' + @name + '.parts.txt') do |f|
			data = false
			f.readlines.each do |l|
				if data && l.chomp.length > 0
					puts "warning: parts.txt line '#{l.chomp}' not recognized" unless l =~ /^([\d\w$.]*) /
					parsed.push $1
				elsif l =~ /^Part.*Value/
					data = true
				end
			end
		end
		in_bom = []
		@comps.each do |comp|
			comp.names.each do |name|
				puts "warning: #{name} was in the bom twice" if in_bom.member? name
				in_bom.push name
			end
		end
		parsed.each do |p|
			unless in_bom.delete p
				puts "warning: '#{p}' was found in the parts.txt but not in the bom"
			end
		end
		in_bom.each do |c|
			puts "warning: '#{c}' was found in the bom but not the parts.txt"
		end
	end
	def add_to_order(source, mult)
		parts = []
		@comps.each do |c|
			if c.source == source
				parts.push c
			end
		end
		case source
		when Mouser
			parts.each do |part|
				puts "#{part.part_num}|#{part.main_quant * mult}"
			end
=begin
			data = "__LASTFOCUS:
__EVENTTARGET:
__EVENTARGUMENT:
__VIEWSTATE:%2FwEPDwUIMzY5NjMzODcPZBYCZg9kFgQCAQ9kFgICCA8WAh4EaHJlZgUVLi4vY3NzL2Jhc2UtZW4tVVMuY3NzZAIDD2QWAmYPZBYIZg9kFhQCAQ9kFgJmDxYCHgRUZXh0BQ4oODAwKSAzNDYtNjg3M2QCBQ8WAh4LXyFJdGVtQ291bnQCBhYMZg9kFgQCAQ8PFgQeC05hdmlnYXRlVXJsBRQvc2VhcmNoL2RlZmF1bHQuYXNweB8BBQhQcm9kdWN0c2RkAgMPFgIfAgIFFgoCAQ9kFgICAQ8PFgQfAwUVL3NlYXJjaC9kZWZhdWx0LmFzcHg%2FHwEFEVZJRVcgQUxMIFBST0RVQ1RTZGQCAg9kFgICAQ8PFgQfAwUIL25ld2VzdC8fAQUPTkVXRVNUIFByb2R1Y3RzZGQCAw9kFgICAQ8PFgQfAwUFL25ldy8fAQUMTmV3IFByb2R1Y3RzZGQCBA9kFgICAQ8PFgQfAwUPL25ld3RlY2hub2xvZ3kvHwEFEE5ldyBUZWNobm9sb2dpZXNkZAIFD2QWAgIBDw8WBB8DBRkva25vd2xlZGdlL2tub3dsZWRnZS5hc3B4HwEFGFByb2R1Y3QgS25vd2xlZGdlIENlbnRlcmRkAgEPZBYEAgEPDxYEHwMFDi9zdXBwbGllcnBhZ2UvHwEFCVN1cHBsaWVyc2RkAgMPFgIfAgIEFggCAQ9kFgICAQ8PFgQfAwUPL3N1cHBsaWVycGFnZS8%2FHwEFElZJRVcgQUxMIFNVUFBMSUVSU2RkAgIPZBYCAgEPDxYEHwMFDi9uZXdzdXBwbGllcnMvHwEFDU5ldyBTdXBwbGllcnNkZAIDD2QWAgIBDw8WBB8DBRIvc3VwcGxpZXJjYXRlZ29yeS8fAQUdU3VwcGxpZXJzIGJ5IFByb2R1Y3QgQ2F0ZWdvcnlkZAIED2QWAgIBDw8WBB8DBSUvY2F0YWxvZ1JlcXVlc3QvY2F0YWxvZ2Rvd25sb2Fkcy5hc3B4HwEFEURvd25sb2FkIExpbmVjYXJkZGQCAg9kFgQCAQ8PFgQfAwUcL0NhdGFsb2dSZXF1ZXN0L0NhdGFsb2cuYXNweB8BBQdDYXRhbG9nZGQCAw8WAh8CAgYWDAIBD2QWAgIBDw8WBB8DBR0vQ2F0YWxvZ1JlcXVlc3QvQ2F0YWxvZy5hc3B4Px8BBRFWSUVXIEFMTCBDQVRBTE9HU2RkAgIPZBYCAgEPDxYEHwMFEy9jYXRhbG9ndmlld2VyLmFzcHgfAQUQRW5oYW5jZWQgQ2F0YWxvZ2RkAgMPZBYCAgEPDxYEHwMFEy9tb2JpbGVjYXRhbG9nLmFzcHgfAQUeTkVXISBNb2JpbGUgQ29tcGF0aWJsZSBDYXRhbG9nZGQCBA9kFgICAQ8PFgQfAwUiL0NhdGFsb2dSZXF1ZXN0L01vdXNlckNhdGFsb2cuYXNweB8BBQtQREYgQ2F0YWxvZ2RkAgUPZBYCAgEPDxYEHwMFJi9DYXRhbG9nUmVxdWVzdC9DYXRhbG9nRG93bmxvYWRzLmFzcHg%2FHwEFGERvd25sb2FkIHRoZSBQREYgQ2F0YWxvZ2RkAgYPZBYCAgEPDxYEHwMFKy9TdWJzY3JpcHRpb25DZW50ZXIvTWFuYWdlU3Vic2NyaXB0aW9uLmFzcHgfAQUUQ2F0YWxvZyBTdWJzY3JpcHRpb25kZAIDD2QWBAIBDw8WBB8DBRIvc2VydmljZXNhbmR0b29scy8fAQUQU2VydmljZXMgJiBUb29sc2RkAgMPFgIfAgIIFhACAQ9kFgICAQ8PFgQfAwUTL3NlcnZpY2VzYW5kdG9vbHMvPx8BBRlWSUVXIEFMTCBTRVJWSUNFUyAmIFRPT0xTZGQCAg9kFgICAQ8PFgQfAwUdL015TW91c2VyL0FjY291bnRTdW1tYXJ5LmFzcHgfAQUXTG9nIEluIC8gQ3JlYXRlIEFjY291bnRkZAIDD2QWAgIBDw8WBB8DBREvRVpCdXkvRVpCdXkuYXNweB8BBQtFWiBCdXkgVG9vbGRkAgQPZBYCAgEPDxYEHwMFHy9PcmRlckhpc3RvcnkvUHJvamVjdHNWaWV3LmFzcHgfAQUPUHJvamVjdCBNYW5hZ2VyZGQCBQ9kFgICAQ8PFgQfAwUJL2JvbXRvb2wvHwEFFE5FVyEgQk9NIEltcG9ydCBUb29sZGQCBg9kFgICAQ8PFgQfAwUNL2FjY2VsZXJhdG9yLx8BBRJTZWFyY2ggQWNjZWxlcmF0b3JkZAIHD2QWAgIBDw8WBB8DBREvUXVvdGUvUXVvdGUuYXNweB8BBQZRdW90ZXNkZAIID2QWAgIBDw8WBB8DBSwvU3Vic2NyaXB0aW9uQ2VudGVyL01hbmFnZVN1YnNjcmlwdGlvbi5hc3B4Px8BBRJFbWFpbCBTdWJzY3JpcHRpb25kZAIED2QWBAIBDw8WBB8DBSQvT3JkZXJIaXN0b3J5L09yZGVySGlzdG9yeUxvZ2luLmFzcHgfAQUNT3JkZXIgSGlzdG9yeWRkAgMPFgIfAgIGFgwCAQ9kFgICAQ8PFgQfAwUlL09yZGVySGlzdG9yeS9PcmRlckhpc3RvcnlMb2dpbi5hc3B4Px8BBQ1PcmRlciBIaXN0b3J5ZGQCAg9kFgICAQ8PFgQfAwUfL09yZGVySGlzdG9yeS9JbnZvaWNlc1ZpZXcuYXNweB8BBQhJbnZvaWNlc2RkAgMPZBYCAgEPDxYEHwMFHC9PcmRlckhpc3RvcnkvQ2FydHNWaWV3LmFzcHgfAQUSU2F2ZWQvU2hhcmVkIENhcnRzZGQCBA9kFgICAQ8PFgQfAwUbL09yZGVySGlzdG9yeS9Cb21zVmlldy5hc3B4HwEFClNhdmVkIEJPTXNkZAIFD2QWAgIBDw8WBB8DBSAvT3JkZXJIaXN0b3J5L1Byb2plY3RzVmlldy5hc3B4Px8BBQ9Qcm9qZWN0IE1hbmFnZXJkZAIGD2QWAgIBDw8WBB8DBR0vT3JkZXJIaXN0b3J5L1F1b3Rlc1ZpZXcuYXNweB8BBQZRdW90ZXNkZAIFD2QWBAIBDw8WBB8DBQ8vSGVscC9IZWxwLmFzcHgfAQUESGVscGRkAgMPFgIfAgIFFgoCAQ9kFgICAQ8PFgQfAwUQL0hlbHAvSGVscC5hc3B4Px8BBRRWSUVXIEFMTCBIRUxQIFRPUElDU2RkAgIPZBYCAgEPDxYEHwMFCS9hYm91dHVzLx8BBQxBYm91dCBNb3VzZXJkZAIDD2QWAgIBDw8WBB8DBQkvY29udGFjdC8fAQUKQ29udGFjdCBVc2RkAgQPZBYCAgEPDxYEHwMFDi9mZWVkYmFjay5hc3B4HwEFCEZlZWRiYWNrZGQCBQ9kFgICAQ8PFgQfAwUOL2hlbHBwYWdlLyNGQVEfAQUERkFRc2RkAgcPZBYCAgEPDxYCHghJbWFnZVVybAUjfi9pbWFnZXMvaW50ZXJuYXRpb25hbC9mbGFncy91cy5naWZkZAIIDxAPFgYeDURhdGFUZXh0RmllbGQFDExhbmd1YWdlTmFtZR4ORGF0YVZhbHVlRmllbGQFD0xhbmd1YWdlQ3VsdHVyZR4LXyFEYXRhQm91bmRnZBAVAgdFbmdsaXNoCEVzcGHDsW9sFQIFZW4tVVMFZXMtTVgUKwMCZ2cWAWZkAgkPEA8WAh4HVmlzaWJsZWhkZBYAZAILDw8WBh8DBTp%2BL0N1cnJlbmN5U2VsZWN0aW9uLmFzcHg%2FcmVkaXJlY3R1cmw9JTJmRVpCdXklMmZFWkJ1eS5hc3B4HwEFA1VTRB8IaGRkAgwPDxYEHwEFA1VTRB8IZ2RkAg4PDxYEHghDc3NDbGFzc2UeBF8hU0ICAmRkAhEPDxYCHwMFZGh0dHBzOi8vd3d3Lm1vdXNlci5jb20vTXlNb3VzZXIvTW91c2VyTG9naW4uYXNweD9xcz0wZ1owZ3YwS0R3cy9zakF4QTNmTWx5TG85eTRoajJRZ2YlMjUyYi9rWW1VZXkydz1kZAIZD2QWBmYPDxYEHwQFI34vaW1hZ2VzL2ludGVybmF0aW9uYWwvZmxhZ3MvdXMuZ2lmHg1BbHRlcm5hdGVUZXh0BQ1Vbml0ZWQgU3RhdGVzZGQCAg8PFgIfAQUNVW5pdGVkIFN0YXRlc2RkAggPFgIfAgIBFgJmD2QWDAIBDw8WBB4PQ29tbWFuZEFyZ3VtZW50BQRVU0R1HwEFClVTIERvbGxhcnNkZAIDDw8WAh8BZWRkAgUPDxYCHwFlZGQCBw8PFgIfAWVkZAIJDw8WAh8BZWRkAgsPDxYCHwFlZGQCAQ9kFi4CAg8PFgQfAQUOUHJvZHVjdCBGaW5kZXIfAwUVfi9zZWFyY2gvZGVmYXVsdC5hc3B4ZGQCBA8PFgQfAwUVfi9zZWFyY2gvZGVmYXVsdC5hc3B4HwEFEVZJRVcgQUxMIFBST0RVQ1RTZGQCBg8PFgQfAwUbfi9PcHRvZWxlY3Ryb25pY3MvXy9OLTVnNXYvHwEFD09wdG9lbGVjdHJvbmljc2RkAggPDxYEHwMFKX4vT3B0b2VsZWN0cm9uaWNzL0xFRC1MaWdodGluZy9fL04tNzRnOXQvHwEFDExFRCBMaWdodGluZ2RkAgoPDxYEHwMFHn4vRW1iZWRkZWQtU29sdXRpb25zL18vTi01ZzFrLx8BBRJFbWJlZGRlZCBTb2x1dGlvbnNkZAIMDw8WBB8DBRp%2BL1NlbWljb25kdWN0b3JzL18vTi01Z2NiLx8BBQ5TZW1pY29uZHVjdG9yc2RkAg4PDxYEHwMFJH4vRGlzY3JldGUtU2VtaWNvbmR1Y3RvcnMvXy9OLTZocGU3Lx8BBRogLSBEaXNjcmV0ZSBTZW1pY29uZHVjdG9yc2RkAhAPDxYEHwMFIH4vSW50ZWdyYXRlZC1DaXJjdWl0cy9fL04tNmo3M2svHwEFHCAtIEludGVncmF0ZWQgQ2lyY3VpdHMgKElDcylkZAISDw8WBB8DBR5%2BL0RldmVsb3BtZW50LVRvb2xzL18vTi02aHBlcS8fAQUUIC0gRGV2ZWxvcG1lbnQgVG9vbHNkZAIUDw8WBB8DBR5%2BL0NpcmN1aXQtUHJvdGVjdGlvbi9fL04tNWczYy8fAQUSQ2lyY3VpdCBQcm90ZWN0aW9uZGQCFg8PFgQfAwUefi9QYXNzaXZlLUNvbXBvbmVudHMvXy9OLTVnNzMvHwEFElBhc3NpdmUgQ29tcG9uZW50c2RkAhgPDxYEHwMFGX4vSW50ZXJjb25uZWN0cy9fL04tNWczeS8fAQUNSW50ZXJjb25uZWN0c2RkAhoPDxYEHwMFFn4vV2lyZS1DYWJsZS9fL04tNWdnbC8fAQUMV2lyZSAmIENhYmxlZGQCHA8PFgQfAwUdfi9FbGVjdHJvbWVjaGFuaWNhbC9fL04tNWcxeC8fAQURRWxlY3Ryb21lY2hhbmljYWxkZAIeDw8WBB8DBRN%2BL1NlbnNvcnMvXy9OLTVnZWovHwEFB1NlbnNvcnNkZAIgDw8WBB8DBRZ%2BL0VuY2xvc3VyZXMvXy9OLTVnM28vHwEFCkVuY2xvc3VyZXNkZAIiDw8WBB8DBR5%2BL1RoZXJtYWwtTWFuYWdlbWVudC9fL04tNWdmcy8fAQUSVGhlcm1hbCBNYW5hZ2VtZW50ZGQCJA8PFgQfAwURfi9Qb3dlci9fL04tNWdiaC8fAQUFUG93ZXJkZAImDw8WBB8DBRx%2BL1Rlc3QtTWVhc3VyZW1lbnQvXy9OLTVnZjMvHwEFElRlc3QgJiBNZWFzdXJlbWVudGRkAigPDxYEHwMFGn4vVG9vbHMtU3VwcGxpZXMvXy9OLTVnZzEvHwEFEFRvb2xzICYgU3VwcGxpZXNkZAIqDw8WBB8DBQZ%2BL25ldy8eB1Rvb2xUaXAFD05FV0VTVCBQcm9kdWN0c2RkAiwPDxYEHwMFDH4va25vd2xlZGdlLx8NBRhQcm9kdWN0IEtub3dsZWRnZSBDZW50ZXJkZAIuDw8WBB8DBSx%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AAAB8KAoACZGQCAw8WAh8QBQZjZW50ZXIWAmYPDxYGHxECCR8SGwAAAAAAgEtAAQAAAB8KAoACZGQCDg9kFgRmD2QWAgIBDxAPFgIfCGhkZBYAZAIBD2QWAgIBDw8WAh8DBUNqYXZhc2NyaXB0Ok9wZW5XaW5kb3coJy4uL0VzdGltYXRlU2hpcHBpbmcvRXN0aW1hdGVTaGlwcGluZy5hc3B4Jyk7ZGQCAw9kFhICCg8PFgQfAQUIQWJvdXQgVXMfAwUKfi9hYm91dHVzL2RkAgsPDxYEHwMFCn4vY29udGFjdC8fAQUKQ29udGFjdCBVc2RkAgwPDxYEHwMFDH4vcHJlc3Nyb29tLx8BBQpQcmVzcyBSb29tZGQCDQ8PFgQfAwUKfi9xdWFsaXR5Lx8BBQdRdWFsaXR5ZGQCDg8PFgQfAwUQfi9lbnZpcm9ubWVudGFsLx8BBQ1FbnZpcm9ubWVudGFsZGQCDw8PFgQfAwUTfi9lZHVjYXRpb25hbHNhbGVzLx8BBRFFZHVjYXRpb25hbCBTYWxlc2RkAhAPDxYEHwMFCn4vY2FyZWVycy8fAQUQQ2FyZWVycyBAIE1vdXNlcmRkAh4PZBYEAgEPZBYCZg8PZBYCHg1vbmNvbnRleHRtZW51BWZqYXZhc2NyaXB0OmFsZXJ0KCdDb3B5aW5nIFByb2hpYml0ZWQgYnkgTGF3IC0gSEFDS0VSIFNBRkUgaXMgYSBUcmFkZW1hcmsgb2YgU2NhbkFsZXJ0Jyk7IHJldHVybiBmYWxzZTtkAgcPFgIfAQUEMjAxMmQCHw8WAh8BBUM8YSBocmVmPSJodHRwOi8vZGUubW91c2VyLmNvbS9kZS1ERS9QYXJ0cy9pbmRleC5odG1sIj5HZXJtYW55PC9hPiAgZBgBBR5fX0NvbnRyb2xzUmVxdWlyZVBvc3RCYWNrS2V5X18WAgUmY3RsMDAkTmF2SGVhZGVyJGNoa1N0b2NrZWRNb3VzZXJIZWFkZXIFLGN0bDAwJE5hdkhlYWRlciRjaGtSb0hTQ29tcGxpYW50TW91c2VySGVhZGVyCkcEbU7GQmYQFyN%2F0MbXFktX2Zs%3D
ctl00%24NavHeader%24ddlLanguage:en-US
ctl00%24NavHeader%24hidSelectedCurrency:-1
ctl00%24NavHeader%24txt1:
as_values_046:
ctl00%24NavHeader%24lblTrdTerm:
ctl00%24NavHeader%24lblIsNewTerm:
ctl00$ContentMain$hNumberOfLines:#{parts.length}
ctl00$ContentMain$txtNumberOfLines:#{parts.length}"
			i = 0
			parts.each do |part|
				puts "#{part} #{i}"
				i += 1
				data += "
ctl00$ContentMain$txtCustomerPartNumber#{i}:#{part.part_num}
ctl00$ContentMain$txtCustomerPartNumber#{i}:
ctl00$ContentMain$txtQuantity#{i}:#{part.quant}"
			end
			data += "
ctl00$ContentMain$btnAddToOrder:Add+to+My+Order"
			puts `wget --header='Cookie: CARTCOOKIEUUID=#{cartcookie}' --post-data='#{data}' 'http://www.mouser.com/EZBuy/EZBuy.aspx' -O -`
#CARTCOOKIEUUID=7538cfb6-6585-4b71-8c55-0c3e4c409c97
=end
		when Digikey
			parts.each do |part|
				puts "#{part.main_quant * mult} #{part.part_num}"
			end
=begin
			i = 0
			data = ''
			parts.each do |part|
				data += "
ctl00$ctl00$mainContentPlaceHolder$mainContentPlaceHolder$txtQty1:#{part.quant}
ctl00$ctl00$mainContentPlaceHolder$mainContentPlaceHolder$txtPart1:#{part.part_num}
ctl00$ctl00$mainContentPlaceHolder$mainContentPlaceHolder$txtCref1:"
			end
			puts `wget --header='Cookie: TS50f921=#{cartcookie}' --post-data='#{data}' 'http://ordering.digikey.com/Ordering/FastAdd.aspx' -O -`
#ce4ea562da94963330f72cbbfba2e7552430f47eb9bc54964f2773efa3c65dd840b6a4aeca85ab037895bf25
=end
		else
			puts "error: don't know how to order from source #{source}"
		end
	end
end

boards = [Board.new("power_converter", "regulator", "Power Regulator"),
	Board.new("gyro_motherboard", "gyro", "Gyro Board", "USB", "Gyro", "Analog and Digital ports", "LDO Power Regulator", "Clock and Microcontroller", "5V switching regulator")]
if ARGV[0]
	boards.reject! do |board|
		board.name != ARGV[0]
	end
	ARGV.shift
end
if ARGV[0]
	case ARGV[0]
	when /^[mM]ouser$/
		source = Mouser
	when /^[dD]igikey$/
		source = Digikey
	else
		puts "error: can't parse source '#{ARGV[0]}'"
		exit
	end
end
boards.each do |board|
	if source
		board.add_to_order(source, ARGV[1].to_i)
	else
		board.check
		board.write_bom
	end
end

__END__
#converter = Board.new("power_converter", "regulator", "Power Regulator")
#converter.check
#converter.write_bom

gyro = Board.new("gyro_motherboard", "gyro", "Gyro Board", "USB", "Gyro", "Analog and Digital ports", "LDO Power Regulator", "Clock and Microcontroller", "5V switching regulator")
gyro.check
gyro.write_bom

