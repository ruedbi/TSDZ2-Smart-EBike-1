#!/bin/bash
set -e

CFLAGS=-DRELEASE_BUILD
MODE=release
echo "Release build"

version="test"
settings_date=$(date +%Y%m%d)
settings_time=$(date +%H%M)

release_folder=$(pwd)/releases
backup_folder=$(pwd)/releases/backup

cd src
# Clean existing
make clean || true

# Build firmware
echo Build started...
make all CFLAGS=$CFLAGS


echo All done !
