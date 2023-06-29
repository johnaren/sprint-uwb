@echo off
:: record <port> <file>
set port=%1
set file=%2

:: if there is an invalid switch exception, switch to backslash
if exist %file% (
	del %file%
)

putty -serial %port% -sercfg 115200 -sessionlog %file%
