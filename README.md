# mie438_project
MIE438 Course Project: Remote Controlled Two-Wheeled Balancing Vehicle

# Running the code

To run, you must first set up the Espressif software developent environment on your computer.

Setup instructions can be found here:
- Windows: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html
- MacOS and Linux: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html

## Setup Notes (for anyone working in linux or mac):
- define the following alias in ~/.bashrc
-     alias get_idf='. $HOME/esp/esp-idf/export.sh'

- then reload shell configuration with:
-     source ~/.bashrc

- and now whenever u wanna open a new workspace for ESP-IDF u can run:
-     get_idf

## Flash and Monitor Output:
- set device target:
-     idf.py set-target esp32
- build:
-     idf.py build
- flash - if ur flashing for the first time, find available ports with:
-     ls /dev/ttyUSB*
- specify port like this:
-     idf.py -p PORT flash
- for example:
-     idf.py -p /dev/ttyUSB0 flash
- and now i think it will recognize the port automatically after the first flash, so going forward u can use:
-     idf.py flash
- and to open output log, use:
-     idf.py monitor
- flash and open log simultaneously with:
-     idf.py flash monitor

