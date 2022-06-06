Minimal configuration of a core-v-mcu

## Prerequisite

1. [optional] If you wish to install the Conda enviroment with python 3.9:

```bash
$ conda update conda
$ conda env create -f environment.yml
```

Activate the environment with

```bash
$ conda activate core-v-mini-mcu
```
2. Install the required Python tools:

```
$ pip3 install --user -r python-requirements.txt
```

Add '--root user_builds' to set your build foders for the pip packages
and add that folder to the `PATH` variable

3. Install the required apt tools:

```
$ sudo apt install lcov libelf1 libelf-dev libftdi1-2 libftdi1-dev libncurses5 libssl-dev libudev-dev libusb-1.0-0 lsb-release texinfo makeinfo autoconf cmake flex bison libexpat-dev gawk
```

In general, have a look at the [Install required software](https://docs.opentitan.org/doc/ug/install_instructions/#system-preparation) section of the OpenTitan documentation.

4. Install the RISC-V Compiler:

```
$ git clone --branch 2022.01.17 --recursive https://github.com/riscv/riscv-gnu-toolchain
$ cd riscv-gnu-toolchain
$ ./configure --prefix=/home/yourusername/tools/riscv --with-abi=ilp32 --with-arch=rv32imc --with-cmodel=medlow
$ make
```

Then, set the `RISCV` env variable as:

```
$ export RISCV=/home/yourusername/tools/riscv
```

5. Install the Verilator:

```
$ export VERILATOR_VERSION=4.210

$ git clone https://github.com/verilator/verilator.git
$ cd verilator
$ git checkout v$VERILATOR_VERSION

$ autoconf
$ ./configure --prefix=/home/yourusername/tools/verilator/$VERILATOR_VERSION
$ make
$ make install
```
Then, set the `PATH` env variable to as:

```
$ export PATH=/home/yourusername/tools/verilator/$VERILATOR_VERSION/bin:$PATH
```

In general, have a look at the [Install Verilator](https://docs.opentitan.org/doc/ug/install_instructions/#verilator) section of the OpenTitan documentation.

If you want to see the vcd waveforms generated by the Verilator simulation, install GTKWAVE:

```
$ sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
$ sudo apt-get install -y gtkwave
```

## Adding external IPs

This repository relies on [vendor](https://docs.opentitan.org/doc/ug/vendor_hw/) to add new IPs.
In the ./util folder, the vendor.py scripts implements what is describeb above.

## Compiling Software

Don't forget to set the `RISCV` env variable to the compiler folder (without the `/bin` included).

Then go to the `./sw` folder and type:

```
$ make applications/hello_world/hello_world.hex
```

This will create the executable file to be loaded in your target system (ASIC, FPGA, Simulation).

## Simulating

This project supports simulation with Verilator, Synopsys VCS, and Siemens Questasim.

### Compiling for Verilator

To simulate your application with Verilator, first compile the HDL:

```
$ fusesoc --cores-root . run --no-export --target=sim --tool=verilator --setup --build openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildsim.log
```

then, go to your target system built folder

```
$ cd ./build/openhwgroup.org_systems_core-v-mini-mcu_0/sim-verilator
```

and type to run your compiled software:

```
$ ./Vtestharness +firmware=../../../sw/applications/hello_world/hello_world.hex
```

### Compiling for VCS

To simulate your application with VCS, first compile the HDL:

```
$ fusesoc --cores-root . run --no-export --target=sim --tool=vcs --setup --build openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildsim.log
```

then, go to your target system built folder

```
$ cd ./build/openhwgroup.org_systems_core-v-mini-mcu_0/sim-vcs
```

and type to run your compiled software:

```
$ ./openhwgroup.org_systems_core-v-mini-mcu_0 +firmware=../../../sw/applications/hello_world/hello_world.hex
```

### Compiling for Questasim

To simulate your application with Questasim, first set the env variable `MODEL_TECH` to your Questasim bin folder, then compile the HDL:

```
$ fusesoc --cores-root . run --no-export --target=sim --tool=modelsim --setup --build openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildsim.log
```

then, go to your target system built folder

```
$ cd ./build/openhwgroup.org_systems_core-v-mini-mcu_0/sim-modelsim/
```

and type to run your compiled software:

```
$ make run PLUSARGS="c firmware=../../../sw/applications/hello_world/hello_world.hex"
```

You can also use vopt for HDL optimized compilation:

```
$ fusesoc --cores-root . run --no-export --target=sim_opt --setup --build openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildsim.log
```

then go to

```
$ cd ./build/openhwgroup.org_systems_core-v-mini-mcu_0/sim_opt-modelsim/
```
and 

```
$ make run PLUSARGS="c firmware=../../../sw/applications/hello_world/hello_world.hex"
```
Questasim version must be >= Questasim 2019.3

### UART DPI

To simulate the UART, we use the LowRISC OpenTitan [UART DPI](https://github.com/lowRISC/opentitan/tree/master/hw/dv/dpi/uartdpi). 
Read how to interact with it in the Section "Interact with the simulated UART" [here](https://docs.opentitan.org/doc/ug/getting_started_verilator/).
The output of the UART DPI module is printed in the `uart0.log` file in the simulation folder.

For example, to see the "hello world!" output of the Verilator simulation:

```
$ cd ./build/openhwgroup.org_systems_core-v-mini-mcu_0/sim-verilator
$ ./Vtestharness +firmware=../../../sw/applications/hello_world/hello_world.hex
$ cat uart0.log
```
## Debug

Follow the [Debug](./Debug.md) guide to debug core-v-mini-mcu.

## Emulation

This project supports emulation on FPGAs (work in progress).

### Xilinx Nexys-A7 100T Flow

Work In Progress and untested!!!

To build and program the bitstream for your FPGA with vivado, type:

```
$ fusesoc --cores-root . run --no-export --target=nexys-a7-100t --setup --build openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildvivado.log
```

If you only need the synthesis implementation:

```
$ fusesoc --cores-root . run --no-export --target=nexys-a7-100t --setup openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildvivado.log
```

then

```
$ cd ./build/openhwgroup.org_systems_core-v-mini-mcu_0/nexys-a7-100t-vivado/
$ make synth
```

at the end of the synthesis, you can export your netlist by typing:

```
$ vivado -notrace -mode batch -source ../../../hw/fpga/scripts/export_verilog_netlist.tcl
```

Only Vivado 2021.2 has been tried.

## ASIC Implementation

This project can be implemented using standard cells based ASIC flow. (work in progress)

### Synthesis with Synopsys Design Compiler

First, you need to provide technology-dependent implementations of some of the cells which require specific instantiation.

Then, please provide a set_libs.tcl and set_constraints.tcl scripts to set link and target libraries, and constraints as the clock.

To generate and run synthesis scripts with DC, execute:

```
$ fusesoc --cores-root . run --no-export --target=asic_synthesis --setup --build openhwgroup.org:systems:core-v-mini-mcu 2>&1 | tee buildsim.log
```

This relies on a fork of [edalize](https://github.com/davideschiavone/edalize) that contains templates for Design Compiler.

## Change Peripheral Memory Map

```
$ python ./util/mcu_gen.py --cfg mcu_cfg.hjson --outdir ./hw/core-v-mini-mcu/include/ --pkg-sv ./hw/core-v-mini-mcu/include/core_v_mini_mcu_pkg.sv.tpl
```
```
python ./util/mcu_gen.py --cfg mcu_cfg.hjson --outdir ./sw/device/lib/runtime/ --header-c ./sw/device/lib/runtime/core_v_mini_mcu.h.tpl
```

## Files are formatted with Verible

We use version v0.0-1824-ga3b5bedf

See: [Install Verible](https://docs.opentitan.org/doc/ug/install_instructions/)
