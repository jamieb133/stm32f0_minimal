### Overview
A minimal project for the stm32f0 discovery board without BSP or HAL autogenerated code. 

### Build
```
mkdir BUILD
cd BUILD
cmake -DARM_TOOLCHAIN_DIR:PATH=<path to arm gcc bin dir> -DCMAKE_TOOLCHAIN_FILE:PATH=../cmake/toolchain.cmake ..
make
```

### Upload
To flash the program you can use st-link:
```
sudo apt install stlink-tools
```
```
# Write to the base flash address space.
st-flash write stm32f0_minimal.bin 0x08000000
```

### Debug
To debug with gdb you'll need openocd and gdb-multiarch:
```
sudo apt install openocd
sudo apt install gdb-multiarch
```

Run openocd using the config files included in this repo, this will open a gdb server on 3333:
```
openocd -f debug_scripts/stlink.cfg -f debug_scripts/stm32f0x.cfg 
```

You should see something like this:
```
Open On-Chip Debugger 0.12.0+dev-01168-g682f927f8 (2023-05-05-07:40)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 1000 kHz
Info : STLINK V2J25S0 (API v2) VID:PID 0483:3748
Info : Target voltage: 2.913369
Info : [stm32f0x.cpu] Cortex-M0 r0p0 processor detected
Info : [stm32f0x.cpu] target has 4 breakpoints, 2 watchpoints
Info : starting gdb server for stm32f0x.cpu on 3333
Info : Listening on port 3333 for gdb connections
```

Then run gdb-multiarch to launch the debug session:
```
gdb-multiarch stm32f0_minimal
```
Connect to the gdb server, load the binary, and reset the program counter.
```
(gdb) target remote localhost:3333
(gdb) file stm32f0_minimal
(gdb) set $pc=Reset_Handler
```