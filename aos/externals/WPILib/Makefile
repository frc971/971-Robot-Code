broken:
	@echo THIS FILE IS OLD 2>&1
	@echo IT IS ONLY HERE TO HELP MANUALLY DOWNLOAD NEW VERSIONS!!!!! 2>&1
	exit 1

all: make.rb WPILib.a
	#ruby make.rb
clean:
	rm -f *.out WPILib/*.o WPILib/*/*.o *.d

deploy: all
	ncftpput robot /ni-rt/system/ wpilib.out

wpilibupdate.exe:
	wget http://firstforge.wpi.edu/sf/frs/do/downloadFile/projects.wpilib/frs.2012_update_for_c.frc_update_3111/frs1405?dl=1 -O wpilibupdate.exe

WPILib: wpilibupdate.exe
	mkdir wpilibinstaller
	cd wpilibinstaller
	unzip wpilibupdate.exe -d wpilibinstaller/ | tee wpilibinstaller/unziplog.txt
	cat wpilibinstaller/unziplog.txt | grep "inflating: wpilibinstaller/.*\.zip" | sed "s/.*\(wpilibinstaller\/.*\.zip\).*/\1/" | xargs -I '{}' cp '{}' ./
	#cat wpilibinstaller/unziplog.txt | grep "inflating: wpilibinstaller/.*\.zip" | sed "s/.*wpilibinstaller\/\(.*\.zip\).*/\1/" | xargs -I '{}' ln -s '{}' ./WPILib.zip
	unzip -a WPILib*.zip
	rm -r wpilibinstaller
	rm -f wpilib_page1.url wpilib_page2.url wpilib_page3.url

WPILib.a: wpilibupdate.exe
	unzip wpilibupdate.exe
	cp vxworks-6.3/target/lib/WPILib.a ./

