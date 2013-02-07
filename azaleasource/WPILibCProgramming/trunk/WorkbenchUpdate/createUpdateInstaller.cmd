echo off
set WINRAR="c:\Progra~1\WinRAR\WinRAR.exe"

rem Update the WPILib source dir
svn update ../WPILib/

rem Come up with the name of the update
for /F %%x in ('cscript //NoLogo helpers\updateName.vbs') do set UPDATE_NAME=%%x

rem Check the update name for errors
rem if not %UPDATE_NAME% == BadSVNSync goto continue
rem echo WPILib is not synced properly!
rem svnversion -n ../WPILib
rem goto end
rem :continue

rem Rebuild WPILib
rem pushd ..\WPILib\PPC603gnu
rem C:\WindRiver\wrenv.exe -p vxworks-6.3 make --no-print-directory BUILD_SPEC=PPC603gnu DEBUG_MODE=1 TRACE=1 clean all
rem popd

rem Make the WorkbenchUpdate dir current
pushd ..\WPILib\Scripts
cmd /C CopyWPILibToUpdateDirectory.cmd
popd

rem Export the WPILib Source
rmdir /s /q WPILib\WPILib
del WPILib\WPILibC++Source*
svn export ..\WPILib\ WPILib\WPILib

rem Compress the source
pushd WPILib
%WINRAR% A -r -df WPILibC++Source%UPDATE_NAME%.zip WPILib
popd

rem Get the revision
for /F %%x in ('svnversion -n ../WPILib') do set UPDATE_REV=%%x

rem Create a project number config
echo PROJECT_NUMBER = rev%UPDATE_REV% > Doxyfile.PROJECT_NUMBER

rem Recompile the Doxygen documentation
rmdir /s /q doxygen_output
mkdir doxygen_output
helpers\doxygen.exe helpers\Doxyfile > doxygen_output\build.log 2> doxygen_output\build_errors.log

rem Done with the Doxyfile.PROJECT_NUMBER
del Doxyfile.PROJECT_NUMBER

rem Compile the resultant help output
helpers\hhc doxygen_output\html\index.hhp

rem Move the help to the installer path
copy doxygen_output\html\index.chm "docs\extensions\FRC\WPILib C++ Reference.chm"

rem Add the update files, but exclude the .svn and build output directories
%WINRAR% A -r -x*\.svn\* -x*\PPC603gnu\* WorkbenchUpdate%UPDATE_NAME%.zip docs vxworks-6.3 workbench-3.0 workbench-3.3 WPILib update.cmd

rem Remove the source zip file since it is already in the update.
del WPILib\WPILibC++Source*.*

rem Add the cRIO image with no compression
%WINRAR% A -m0 WorkbenchUpdate%UPDATE_NAME%.zip FRC_20*.zip

rem Start with the boiler-plate comment
copy helpers\sfx_comment.txt sfx_comment.tmp.txt

rem Append the name of this update (ends up after "Title=" in the comment)
echo WorkbenchUpdate%UPDATE_NAME% >> sfx_comment.tmp.txt

rem Add the comment to the archive
%WINRAR% c -zsfx_comment.tmp.txt WorkbenchUpdate%UPDATE_NAME%.zip

rem Done with the comment
del sfx_comment.tmp.txt

rem Make it self extracting
%WINRAR% s -df WorkbenchUpdate%UPDATE_NAME%.zip
echo Success!
:end
