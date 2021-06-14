cd ~/Documents/RVfpga/verilatorSIM 
make clean 
make


cd ~/Documents/RVfpga/examples/memBench/commandLine
rm bench.S; cp ../src/bench.S ./bench.S 
make clean 

make bench.elf 

make bench.bin 

make bench.vh 

../../../verilatorSIM/Vrvfpgasim +ram_init_file=bench.vh +vcd=1 +timeout=$1

gtkwave trace.vcd -S ~/temp.tcl
