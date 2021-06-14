#!/bin/bash

cd ~/Documents/RVfpga/verilatorSIM
make clean
make
cp ~/Documents/RVfpga/verilatorSIM/Vrvfpgasim ~/.platformio/packages/tool-verilator-swervolf/Vswervolf_core_tb
cd ~/Documents/RVfpga/examples/testLw
~/.platformio/penv/bin/pio run --target generate_trace
cd .pio/build/swervolf_nexys
gtkwave trace.vcd
