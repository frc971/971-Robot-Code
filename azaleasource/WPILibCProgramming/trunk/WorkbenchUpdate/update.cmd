rmdir /s /q C:\WindRiver\vxworks-6.3\target\h\WPILib
rmdir /s /q C:\WindRiver\vxworks-6.3\target\h\frcvision
rmdir /s /q C:\WindRiver\docs\extensions\WPILib
rmdir /s /q C:\WindRiver\WPILib
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\SimpleRobot
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\IterativeRobot
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\BuiltinDefaultCode
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\CSimpleTemplate
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\DashboardDataExample
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\DashboardDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\DefaultRobot
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\GyroSample
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\IterativeBenchtopDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\IterativeDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\SimpleTemplate
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\SimpleTracker
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\VisionDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\VisionServoDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\2010ImageDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\TwoColorTrackDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\CameraDemo
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\CameraToDashboardExample
rmdir /s /q C:\WindRiver\vxworks-6.3\target\src\demo\LineTrackerSample

del /q C:\WindRiver\workbench-3.0\wrwb\wrworkbench\eclipse\plugins\edu.wpi.*
del /q C:\WindRiver\workbench-3.3\wrwb\wrworkbench\eclipse\plugins\edu.wpi.*
del /q C:\WindRiver\WPILib\*.out
del /q C:\WindRiver\WPILib\vxWorks
del /q C:\WindRiver\WPILib\cRIOFRC_vxWorks
del /q C:\WindRiver\WPILib\cRIOFRCII_vxWorks
del /q c:\WindRiver\docs\extensions\FRC\*.*
del /q c:\WindRiver\docs\extensions\FRC_Vision_API_Specification.*
del /q c:\WindRiver\docs\extensions\NIVisionCVI.*
del /q c:\WindRiver\docs\extensions\Robot*.*
del /q c:\WindRiver\docs\extensions\WPI*.*

xcopy vxworks-6.3 c:\windriver\vxworks-6.3 /s /y /i
xcopy workbench-3.0 c:\windriver\workbench-3.0 /s /y /i
xcopy workbench-3.3 c:\windriver\workbench-3.3 /s /y /i
xcopy WPILib c:\windriver\WPILib /s /y /i
xcopy docs c:\windriver\docs /s /y /i

mkdir c:\windriver\wpilib\cRIO_Images
mkdir c:\windriver\wpilib\cRIO_Images\backup
copy c:\windriver\wpilib\cRIO_Images\*.zip c:\windriver\wpilib\cRIO_Images\backup
del /q c:\windriver\wpilib\cRIO_Images\*.zip
copy frc_20*.zip c:\windriver\wpilib\cRIO_Images

ren C:\WindRiver\gnu\3.4.4-vxworks-6.3\x86-win32\libexec\gcc\powerpc-wrs-vxworks\3.4.4\get_feature.exe get_feature.hangs.exe
