#!/bin/bash
set -e

if [ "$1" == "release" ]; then
	CFLAGS=-DRELEASE_BUILD
	MODE=release
	echo "Release build"
else
	CFLAGS=-DDEBUG_BUILD
	MODE=debug
	echo "Debug build"
fi

version="v282_${MODE}_25plus_dc30_10_no_power_limit"
settings_date=$(date +%Y%m%d)
settings_time=$(date +%H%M)

release_folder=/mnt/c/Users/Rüdiger/Meine\ Ablage/DriveSyncFiles/ebike
release_folder2=$(pwd)/releases
backup_folder=$(pwd)/releases/backup

cd src
# Clean existing
rm -rf main.hex || true
make clean || true

# Build firmware
echo Build started...
make all CFLAGS=$CFLAGS

# Save new firmware
echo Copying firmware to release folder.
echo $release_folder/TSDZ2-$version-$settings_date.hex
mkdir -p "$release_folder"
mkdir -p "$release_folder2"
yes | cp -rf ../bin/main.hex "$release_folder/TSDZ2-$version-$settings_date.hex"
yes | cp -rf ../bin/main.hex "$release_folder2/TSDZ2-$version-$settings_date.hex"

# ruedbi:
exit 0

backup=no
while true; do
	read -p "Do you want to backup the firmware ? [y/N]" yn
	case $yn in
		y ) backup=yes; break;;
		n ) break;;
		* ) break;;
esac
done

# Backup firmware
if [ "$backup" = "yes" ]; then
	echo Backup current firmware to $backup_folder/TSDZ_orig_opt-$version-$settings_date.bin
	mkdir -p $backup_folder
	make backup
	yes | cp -rf ../bin/TSDZ_orig_opt.bin $backup_folder/TSDZ_orig_opt-$version-$settings_date.bin
	yes | cp -rf ../bin/TSDZ_orig.bin $backup_folder/TSDZ_orig-$version-$settings_date.bin
	yes | cp -rf ../bin/TSDZ_orig_eeprom.bin $backup_folder/TSDZ_orig_eeprom-$version-$settings_date.bin
fi

flash=yes
while true; do
read -p "Do you want to flash the motor ? [Y/n]" yn
case $yn in
	y ) break;;
	n ) flash=no; break;;
	* ) break;;
esac
done

# Flash new firmware
if [ "$flash" = "yes" ]; then
	make clear_eeprom
	make flash
fi

echo All done !
