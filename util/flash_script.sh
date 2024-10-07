#!/bin/bash

# Script to automate the process of flashing and programming
# Place this script in root/sw/applications/bme/

# Colors for fancy output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if the PROJECT name is provided as an argument
if [ -z "$1" ]; then
    echo -e "${RED}Usage: $0 <PROJECT_NAME> [plot/serial] [verbose]${NC}"
    exit 1
fi

PROJECT_NAME="$1"

# Check for 'verbose' argument
if [ "$3" == "verbose" ]; then
    VERBOSE=1
else
    VERBOSE=0
fi

# Check for 'verbose' argument
if [ "$2" == "plot" ]; then
    PLOT=1
elif [ "$2" == "verbose" ]; then
    VERBOSE=1
fi


# Navigate to the root directory
echo -e "${BLUE}➤ Navigating to the root directory...${NC}"
cd "$(dirname "$0")"/../ || {
    echo -e "${RED}✖ Failed to change to the root directory.${NC}"
    exit 1
}
sleep 1

# Run 'make flash-readid' and capture the output
echo -e "${BLUE}➤ Reading flash ID...${NC}"
if [ $VERBOSE -eq 1 ]; then
    OUTPUT=$(make flash-readid)
else
    OUTPUT=$(make flash-readid 2>&1)
fi

# Check if the flash ID matches the expected value
if echo "$OUTPUT" | grep -q "flash ID: 0xEF 0x70 0x18 0x00"; then
    echo -e "${GREEN}✔ Flash ID matches the expected value.${NC}"
else
    echo -e "${RED}✖ Flash ID does not match. Expected: 0xEF 0x70 0x18 0x00${NC}"
    echo -e "${RED}Actual output:${NC}"
    echo "$OUTPUT"
    exit 1
fi
sleep 1

# Proceed with building the application
echo -e "${BLUE}➤ Building the application for project '${PROJECT_NAME}'...${NC}"
if [ $VERBOSE -eq 1 ]; then
    make app PROJECT="$PROJECT_NAME" TARGET=pynq-z2 LINKER=flash_load || {
        echo -e "${RED}✖ Failed to build the application.${NC}"
        exit 1
    }
else
    make app PROJECT="$PROJECT_NAME" TARGET=pynq-z2 LINKER=flash_load > /dev/null 2>&1 || {
        echo -e "${RED}✖ Failed to build the application.${NC}"
        exit 1
    }
fi
echo -e "${GREEN}✔ Build completed successfully.${NC}"
sleep 1

# Program the flash
echo -e "${BLUE}➤ Programming the flash...${NC}"
if [ $VERBOSE -eq 1 ]; then
    make flash-prog || {
        echo -e "${RED}✖ Failed to program the flash.${NC}"
        exit 1
    }
else
    make flash-prog > /dev/null 2>&1 || {
        echo -e "${RED}✖ Failed to program the flash.${NC}"
        exit 1
    }
fi
echo -e "${GREEN}✔ Flash programming completed successfully.${NC}"
sleep 1

echo -e "${GREEN}🎉 All steps completed successfully!${NC}"

if [ $PLOT -eq 1 ]; then
    # Opening serial port and plotting data
    echo -e "${BLUE}➤ Starting data plotting...${NC}"

    # Run the Python plotting script
    python3 util/plot_sensor_data_bme.py "$PROJECT_NAME"
else
    # Openning serial port
    echo -e "${BLUE}➤ Openning serial port...${NC}"
    picocom -b 9600 -r -l --imap lfcrlf /dev/serial/by-id/usb-FTDI_Quad_RS232-HS-if02-port0
fi