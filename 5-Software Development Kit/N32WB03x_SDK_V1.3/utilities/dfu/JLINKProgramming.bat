@echo off


set JLink_path=.\JLink\JLink_V632\JLink.exe
set JLink_script_path=.\JLink\JLink_Script\download.jlink
set NSUtil_path=.\NSUtil\NSUtil.exe

::Creating bootsetting file
::bootsetting.bin path
set output_bootsetting=.\Image\bootsetting.bin
::bank1 parameters£¬nonoptional
set bank1_start_address=0x1004000
set bank1_version=0x00000001
set bank1_bin=.\Image\APP1.bin
set bank1_activation=yes
::bank2 parameters£¬optional
set bank2_start_address=0x1020000
set bank2_version=0x00000001
set bank2_bin=.\Image\APP2.bin
set bank2_activation=no
::ImageUpdate parameters£¬optional
set image_update_start_address=0x0103C000
set image_update_version=0x00000001
::set image_update_bin=.\Image\ImageUpdate.bin
set image_update_activation=no
::ImageUpdate parameters£¬optional
set public_key_file=.\keys\public_key.bin


if exist %output_bootsetting% (del %output_bootsetting%)

if defined public_key_file (

	if defined bank2_bin if defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--public_key_file %public_key_file%                           ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%                         ^
												--bank2_start_address %bank2_start_address%                   ^
												--bank2_version %bank2_version%                               ^
												--bank2_bin %bank2_bin%                                       ^
												--bank2_activation %bank2_activation%                         ^
												--image_update_start_address %image_update_start_address%     ^
												--image_update_version %image_update_version%                 ^
												--image_update_bin %image_update_bin%                         ^
												--image_update_activation %image_update_activation%
												
	
	)
	if defined bank2_bin if not defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--public_key_file %public_key_file%                           ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%                         ^
												--bank2_start_address %bank2_start_address%                   ^
												--bank2_version %bank2_version%                               ^
												--bank2_bin %bank2_bin%                                       ^
												--bank2_activation %bank2_activation%
												
												
	)
	if not defined bank2_bin if defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--public_key_file %public_key_file%                           ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%                         ^
												--image_update_start_address %image_update_start_address%     ^
												--image_update_version %image_update_version%                 ^
												--image_update_bin %image_update_bin%                         ^
												--image_update_activation %image_update_activation%

															
	)
	if not defined bank2_bin if not defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--public_key_file %public_key_file%                           ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%
	
		
	)	
	

)

if not defined public_key_file (

	if defined bank2_bin if defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%                         ^
												--bank2_start_address %bank2_start_address%                   ^
												--bank2_version %bank2_version%                               ^
												--bank2_bin %bank2_bin%                                       ^
												--bank2_activation %bank2_activation%                         ^
												--image_update_start_address %image_update_start_address%     ^
												--image_update_version %image_update_version%                 ^
												--image_update_bin %image_update_bin%                         ^
												--image_update_activation %image_update_activation%
	)
	if defined bank2_bin if not defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%                         ^
												--bank2_start_address %bank2_start_address%                   ^
												--bank2_version %bank2_version%                               ^
												--bank2_bin %bank2_bin%                                       ^
												--bank2_activation %bank2_activation%
	)
	if not defined bank2_bin if defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%                         ^
												--image_update_start_address %image_update_start_address%     ^
												--image_update_version %image_update_version%                 ^
												--image_update_bin %image_update_bin%                         ^
												--image_update_activation %image_update_activation%
	)
	if not defined bank2_bin if not defined image_update_bin (
	
		%NSUtil_path% bootsetting generate %output_bootsetting%                                         ^
												--bank1_start_address %bank1_start_address%                   ^
												--bank1_version %bank1_version%                               ^
												--bank1_bin %bank1_bin%                                       ^
												--bank1_activation %bank1_activation%
	)	



)
::Execute Jlink script file to program bin files								 
if exist %output_bootsetting% (
	%JLink_path% -Device N32WB031 -If SWD -Speed 4000 -commanderscript %JLink_script_path%
)          





PAUSE
EXIT

