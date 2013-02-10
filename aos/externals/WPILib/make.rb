
$excludes = [/Vision2009/,/CInterfaces/,/NetworkTables/,/SmartDashboard/,/Commands/,/Buttons/,/Preferences/,/swp$/]

files = `find WPILib`.split(/\n/).collect do |file|
	if(file =~ /.*\.cpp/)
		file.gsub!(/cpp\Z/,"o")
		$excludes.each do |exclude|
			if(file && file =~ exclude)
				file = nil
			end
		end
		file
	end
end.compact

make = "make -f ../module.mk"
defines = "DEFINES=-D\'SVN_REV=\\\"2262\\\"\'"
added_includes = "ADDED_INCLUDES=-I./WPILib/"
outfile = "wpilib.out PROJECT_TARGET=wpilib.out"
objects = "PROJECT_OBJECTS=\"#{files.join(" ")}\""

cmd = "#{make} #{outfile} #{added_includes} #{defines} #{objects}"
system cmd
