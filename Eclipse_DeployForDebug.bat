:: set working directory to where the compiled lib is
cd %~dp0\Debug

:: move the headers into the WPILIB folder for FRC applications
xcopy /Y /E ..\..\Phoenix-frc-lib\cpp\include\ctre %HOMEPATH%\wpilib\user\cpp\include\ctre

:: move the driver headers into the WPILIB folder for FRC applications
xcopy /Y /E ..\..\Phoenix-frc-lib\libraries\driver\include\ctre %HOMEPATH%\wpilib\user\cpp\include\ctre

:: move the low level static lib so Robot apps use it
echo F|xcopy /Y /E .\libCTRE_Phoenix.a %HOMEPATH%\wpilib\user\cpp\lib\libCTRE_Phoenix.a