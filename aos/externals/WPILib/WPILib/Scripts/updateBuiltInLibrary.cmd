cd C:\WindRiver\vxworks-6.3\target

mkdir h\WPILib
mkdir h\WPILib\Buttons
mkdir h\WPILib\CAN
mkdir h\WPILib\ChipObject
mkdir h\WPILib\CInterfaces
mkdir h\WPILib\Commands
mkdir h\WPILib\NetworkCommunication
mkdir h\WPILib\NetworkTables
mkdir h\WPILib\SmartDashboard
mkdir h\WPILib\visa
mkdir h\WPILib\Vision
mkdir h\WPILib\Vision2009

del h\WPILib\*.h
del h\WPILib\Buttons\*.h
del h\WPILib\CAN\*.h
del h\WPILib\ChipObject\*.h
del h\WPILib\CInterfaces\*.h
del h\WPILib\Commands\*.h
del h\WPILib\NetworkCommunication\*.h
del h\WPILib\NetworkTables\*.h
del h\WPILib\SmartDashboard\*.h
del h\WPILib\visa\*.h
del h\WPILib\Vision\*.h
del h\WPILib\Vision2009\*.h

copy c:\WindRiver\workspace\WPILib\*.h h\WPILib
copy c:\WindRiver\workspace\WPILib\Buttons\*.h h\WPILib\Buttons
copy c:\WindRiver\workspace\WPILib\CAN\*.h h\WPILib\CAN
copy c:\WindRiver\workspace\WPILib\ChipObject\*.h h\WPILib\ChipObject
copy C:\WindRiver\workspace\WPILib\CInterfaces\*.h h\WPILib\CInterfaces
copy C:\WindRiver\workspace\WPILib\Commands\*.h h\WPILib\Commands
copy C:\WindRiver\workspace\WPILib\NetworkCommunication\*.h h\WPILib\NetworkCommunication
copy C:\WindRiver\workspace\WPILib\NetworkTables\*.h h\WPILib\NetworkTables
copy C:\WindRiver\workspace\WPILib\SmartDashboard\*.h h\WPILib\SmartDashboard
copy c:\WindRiver\workspace\WPILib\visa\*.h h\WPILib\visa
copy c:\WindRiver\workspace\WPILib\Vision\*.h h\WPILib\Vision
copy c:\WindRiver\workspace\WPILib\Vision2009\*.h h\WPILib\Vision2009

copy C:\WindRiver\workspace\WPILib\PPC603gnu\WPILib\Debug\WPILib.a lib

rem copy c:\WindRiver\workspace\WorkbenchUpdate\frc_20*.zip c:\WindRiver\WPILib\cRIO_Images

cd \WindRiver\workspace\WPILib\Scripts
