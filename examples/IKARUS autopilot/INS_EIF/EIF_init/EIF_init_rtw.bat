@echo off
call setup_mssdk71.bat

cd .
nmake -f EIF_init_rtw.mk  GENERATE_REPORT=1 ADD_MDL_NAME_TO_GLOBALS=0
@if errorlevel 1 goto error_exit
exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
An_error_occurred_during_the_call_to_make
