REM set AREA_DETECTOR=%SUPPORT%\areaDetector_2_git
REM set EPICS_DISPLAY_PATH=%AREA_DETECTOR%\ADPICam\PICamApp\op\adl;%AREA_DETECTOR%ADCore\ADApp\op\adl
REM echo %EPICS_DISPLAY_PATH%
REM start medm -x -macro "P=13PICAM1:, R=cam1:" PICam.adl
REM start medm -x -macro "P=13PICAM1:, R=cam1:" ADBase.adl
..\..\bin\windows-x64-debug\PICamApp st.cmd

