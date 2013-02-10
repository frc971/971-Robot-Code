
C:

mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\Buttons
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\CAN
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\ChipObject
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\CInterfaces
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\Commands
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\NetworkCommunication
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\NetworkTables
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\SmartDashboard
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\visa
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\Vision
mkdir \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib\Vision2009

del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\Buttons\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\CAN\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\ChipObject\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\CInterfaces\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\Commands\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\NetworkCommunication\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\NetworkTables\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\SmartDashboard\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\visa\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\Vision\*.h
del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPIlib\Vision2009\*.h

del \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\lib\WPILib.a

cd \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\h\WPILib

copy \WindRiver\workspace\WPILib\*.h
copy \WindRiver\workspace\WPILib\Buttons\*.h Buttons
copy \WindRiver\workspace\WPILib\CAN\*.h CAN
copy \WindRiver\workspace\WPILib\ChipObject\*.h ChipObject
copy \WindRiver\workspace\WPILib\CInterfaces\*.h CInterfaces
copy \WindRiver\workspace\WPILib\Commands\*.h Commands
copy \WindRiver\workspace\WPILib\NetworkCommunication\*.h NetworkCommunication
copy \WindRiver\workspace\WPILib\NetworkTables\*.h NetworkTables
copy \WindRiver\workspace\WPILib\SmartDashboard\*.h SmartDashboard
copy \WindRiver\workspace\WPILib\visa\*.h visa
copy \WindRiver\workspace\WPILib\Vision\*.h Vision
copy \WindRiver\workspace\WPILib\Vision2009\*.h Vision2009

copy C:\WindRiver\workspace\WPILib\PPC603gnu\WPILib\Debug\WPILib.a \WindRiver\workspace\WorkbenchUpdate\vxworks-6.3\target\lib

cd \WindRiver\workspace\WPILib\Scripts
