echo off

:: APP.bin is the update firmware
:: --app_start_address is the address of update firmware
:: --app_version is the version of update firmware
:: --serial_port is the serial port number                
set app_bin=.\Image\APP2.bin
set app_start_address=0x1020000
set app_version=0x01020304
set serial_port=COM9

				  
							 
.\NSUtil\NSUtil.exe ius serial %app_bin%                                       ^
                               --app_start_address %app_start_address%         ^
							   --app_version %app_version%                     ^
							   --serial_port %serial_port%


							 
PAUSE
EXIT							 