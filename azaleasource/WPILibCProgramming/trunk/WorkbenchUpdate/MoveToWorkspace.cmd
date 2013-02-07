rmdir /s /q c:\WindRiver\workspace\WPILib
rmdir /s /q c:\WindRiver\workspace\WorkbenchUpdate
rmdir /s /q c:\WindRiver\workspace\UnitTestProject
xcopy WPILib c:\WindRiver\workspace\WPILib /s /y /i
xcopy WorkbenchUpdate c:\WindRiver\workspace\WorkbenchUpdate /s /y /i
xcopy UnitTestProject c:\WindRiver\workspace\UnitTestProject /s /y /i

pushd c:\WindRiver\workspace\WPILib\PPC603gnu
C:\WindRiver\wrenv.exe -p vxworks-6.3 make --no-print-directory BUILD_SPEC=PPC603gnu DEBUG_MODE=1 TRACE=1 clean all
cd ..\scripts
pause
cmd /c updateBuiltInLibrary.cmd 
popd

pushd c:\WindRiver\workspace\UnitTestProject
C:\WindRiver\wrenv.exe -p vxworks-6.3 make --no-print-directory BUILD_SPEC=PPC603gnu DEBUG_MODE=1 TRACE=1 clean all
popd
