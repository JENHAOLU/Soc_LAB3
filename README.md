# Soc_LAB3

FIR - Verilog implementation
# Interface
data_in stream (Xn)
data_out: stream (Yn)
coef[Tape_Num-1:0] axilite
len: axilite
ap_start: axilite
ap_done: axilite
# Specification
Data width: 32
Number of tap coefficients: 11
Number of data is determined by the testbench

Only use one Multiplier and one Adder
Shift register implemented with SRAM (Shift_RAM, size = 10 DW) – size = 10 DW
Tap coefficient implemented with SRAM (Tap_RAM = 11 DW) and initialized by axilite write
Operation
Set ap_start to initiate FIR engine (ap_start valid for one clock cycle)
Stream in Xn. The input rate is depending on the FIR processing speed. Use AXI-Stream valid/ready for flow control
Stream out Yn. The output rate depends on the FIR processing speed
