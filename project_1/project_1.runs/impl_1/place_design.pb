
Q
Command: %s
53*	vivadotcl2 
place_design2default:defaultZ4-113h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
xc7a100t2default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
xc7a100t2default:defaultZ17-349h px? 
P
Running DRC with %s threads
24*drc2
62default:defaultZ23-27h px? 
V
DRC finished with %s
79*	vivadotcl2
0 Errors2default:defaultZ4-198h px? 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px? 
p
,Running DRC as a precondition to command %s
22*	vivadotcl2 
place_design2default:defaultZ4-22h px? 
P
Running DRC with %s threads
24*drc2
62default:defaultZ23-27h px? 
?
?Bank IO standard internal Vref conflict: Conflicting INTERNAL_VREF constraint in Bank %s.  Some ports in this bank, for example, %s   (SSTL18_II, Vref=0.900V) 
 at site %s conflict with constrained INTERNAL_VREF of 0.750V.%s*DRC2.
 "
342default:default2default:default2D
 ".
ddram_dq[0]ddram_dq[0]2default:default2default:default2@
 "*
	IOB_X1Y54
	IOB_X1Y542default:default2default:default24
 DRC|Pin Planning|IO Standard2default:default8ZBIIVRC-1h px? 
?
YReport rule limit reached: REQP-1839 rule limit reached: 20 violations have been found.%s*DRC29
 !DRC|DRC System|Rule limit reached2default:default8ZCHECK-3h px? 
?
YReport rule limit reached: REQP-1840 rule limit reached: 20 violations have been found.%s*DRC29
 !DRC|DRC System|Rule limit reached2default:default8ZCHECK-3h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[0]"ddr2/ldc/storage_10_reg_0/WEBWE[0]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[0]"ddr2/ldc/storage_10_reg_0/WEBWE[0]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[1]"ddr2/ldc/storage_10_reg_0/WEBWE[1]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[1]"ddr2/ldc/storage_10_reg_0/WEBWE[1]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[2]"ddr2/ldc/storage_10_reg_0/WEBWE[2]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[2]"ddr2/ldc/storage_10_reg_0/WEBWE[2]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[3]"ddr2/ldc/storage_10_reg_0/WEBWE[3]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[3]"ddr2/ldc/storage_10_reg_0/WEBWE[3]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[4]"ddr2/ldc/storage_10_reg_0/WEBWE[4]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[4]"ddr2/ldc/storage_10_reg_0/WEBWE[4]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[5]"ddr2/ldc/storage_10_reg_0/WEBWE[5]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[5]"ddr2/ldc/storage_10_reg_0/WEBWE[5]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[6]"ddr2/ldc/storage_10_reg_0/WEBWE[6]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[6]"ddr2/ldc/storage_10_reg_0/WEBWE[6]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[7]"ddr2/ldc/storage_10_reg_0/WEBWE[7]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_0	ddr2/ldc/storage_10_reg_02default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_0/WEBWE[7]"ddr2/ldc/storage_10_reg_0/WEBWE[7]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2\
 "F
ddr2/ldc/storage_13_reg	ddr2/ldc/storage_13_reg2default:default2default:default2l
 "V
ddr2/ldc/storage_13_reg/ENARDENddr2/ldc/storage_13_reg/ENARDEN2default:default2default:default2z
 "d
&ddr2/ldc/soc_read_r_buffer_syncfifo_re&ddr2/ldc/soc_read_r_buffer_syncfifo_re2default:default2default:default2?
 "?
Ecdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/gen_sync[0].i_sync/reg_q_reg[1]	Ecdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/gen_sync[0].i_sync/reg_q_reg[1]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2\
 "F
ddr2/ldc/storage_13_reg	ddr2/ldc/storage_13_reg2default:default2default:default2l
 "V
ddr2/ldc/storage_13_reg/ENARDENddr2/ldc/storage_13_reg/ENARDEN2default:default2default:default2z
 "d
&ddr2/ldc/soc_read_r_buffer_syncfifo_re&ddr2/ldc/soc_read_r_buffer_syncfifo_re2default:default2default:default2?
 "?
Ecdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/gen_sync[1].i_sync/reg_q_reg[1]	Ecdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/gen_sync[1].i_sync/reg_q_reg[1]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2\
 "F
ddr2/ldc/storage_13_reg	ddr2/ldc/storage_13_reg2default:default2default:default2l
 "V
ddr2/ldc/storage_13_reg/ENARDENddr2/ldc/storage_13_reg/ENARDEN2default:default2default:default2z
 "d
&ddr2/ldc/soc_read_r_buffer_syncfifo_re&ddr2/ldc/soc_read_r_buffer_syncfifo_re2default:default2default:default2?
 "~
3cdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/wptr_q_reg[0]	3cdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/wptr_q_reg[0]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2\
 "F
ddr2/ldc/storage_13_reg	ddr2/ldc/storage_13_reg2default:default2default:default2l
 "V
ddr2/ldc/storage_13_reg/ENARDENddr2/ldc/storage_13_reg/ENARDEN2default:default2default:default2z
 "d
&ddr2/ldc/soc_read_r_buffer_syncfifo_re&ddr2/ldc/soc_read_r_buffer_syncfifo_re2default:default2default:default2?
 "~
3cdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/wptr_q_reg[1]	3cdc/i_axi_cdc/i_cdc_fifo_gray_r/i_src/wptr_q_reg[1]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[0]"ddr2/ldc/storage_10_reg_1/WEBWE[0]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[0]"ddr2/ldc/storage_10_reg_1/WEBWE[0]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[1]"ddr2/ldc/storage_10_reg_1/WEBWE[1]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[1]"ddr2/ldc/storage_10_reg_1/WEBWE[1]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[2]"ddr2/ldc/storage_10_reg_1/WEBWE[2]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[2]"ddr2/ldc/storage_10_reg_1/WEBWE[2]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[3]"ddr2/ldc/storage_10_reg_1/WEBWE[3]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.a_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2`
 "J
ddr2/ldc/storage_10_reg_1	ddr2/ldc/storage_10_reg_12default:default2default:default2r
 "\
"ddr2/ldc/storage_10_reg_1/WEBWE[3]"ddr2/ldc/storage_10_reg_1/WEBWE[3]2default:default2default:default2v
 "`
$ddr2/ldc/vns_roundrobin5_grant_reg_0$ddr2/ldc/vns_roundrobin5_grant_reg_02default:default2default:default2?
 "?
Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg	Qcdc/i_axi_cdc/i_cdc_fifo_gray_w/i_dst/i_spill_register/gen_spill_reg.b_full_q_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Zswervolf/swerv_eh1/mem/dc_mem/dc_data_inst/wb_dout_way_sel_lp[0].dc_rw_adr_ff/dout_reg[10]	Zswervolf/swerv_eh1/mem/dc_mem/dc_data_inst/wb_dout_way_sel_lp[0].dc_rw_adr_ff/dout_reg[10]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?

?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "~
3swervolf/swerv_eh1/swerv/dec/tlu/freeff/dout_reg[0]	3swervolf/swerv_eh1/swerv/dec/tlu/freeff/dout_reg[0]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?

?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "~
3swervolf/swerv_eh1/swerv/dec/tlu/freeff/dout_reg[1]	3swervolf/swerv_eh1/swerv/dec/tlu/freeff/dout_reg[1]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?

?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "~
3swervolf/swerv_eh1/swerv/dec/tlu/freeff/dout_reg[2]	3swervolf/swerv_eh1/swerv/dec/tlu/freeff/dout_reg[2]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Jswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[0]	Jswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[0]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[10]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[10]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[11]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[11]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[12]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[12]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[13]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[13]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[14]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[14]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[17]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[17]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2?
 "?
Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2	Pswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_22default:default2default:default2?
 "?
`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]`swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2/ADDRARDADDR[13]2default:default2default:default2?
 "?
Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]Iswervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ADR[10]2default:default2default:default2?
 "?
Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[18]	Kswervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[18]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
c
DRC finished with %s
79*	vivadotcl2)
0 Errors, 43 Warnings2default:defaultZ4-198h px? 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px? 
U

Starting %s Task
103*constraints2
Placer2default:defaultZ18-103h px? 
}
BMultithreading enabled for place_design using a maximum of %s CPUs12*	placeflow2
62default:defaultZ30-611h px? 
v

Phase %s%s
101*constraints2
1 2default:default2)
Placer Initialization2default:defaultZ18-101h px? 
?

Phase %s%s
101*constraints2
1.1 2default:default29
%Placer Initialization Netlist Sorting2default:defaultZ18-101h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.022default:default2
00:00:00.032default:default2
3876.7272default:default2
0.0002default:default2
41812default:default2
111412default:defaultZ17-722h px? 
Z
EPhase 1.1 Placer Initialization Netlist Sorting | Checksum: 6910fb90
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.03 ; elapsed = 00:00:00.06 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 4181 ; free virtual = 111412default:defaulth px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.022default:default2
00:00:00.022default:default2
3876.7272default:default2
0.0002default:default2
41812default:default2
111412default:defaultZ17-722h px? 
?

Phase %s%s
101*constraints2
1.2 2default:default2F
2IO Placement/ Clock Placement/ Build Placer Device2default:defaultZ18-101h px? 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2-
clk_gen/dout[0]_i_1__42032default:default2
182default:default2?
?	swervolf/swerv_eh1/swerv/lsu/bus_intf/ldst_dual_dc3ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/lsu_byten_dc3ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/lsu_byten_dc3ff/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/lsu_byten_dc3ff/dout_reg[3] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/access_fault_dc3ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2-
clk_gen/dout[0]_i_1__42042default:default2
112default:default2?
?	swervolf/swerv_eh1/swerv/lsu/clkdomain/lsu_freeze_c1_dc2_clkenff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_hit_rdc_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/hitable_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_hit_way_dc23_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_hit_way_dc23_ff/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[0]_i_1__312default:default2
12default:default2V
B	swervolf/swerv_eh1/swerv/exu/mul_e1/low_e2_ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[0]_i_1__322default:default2
12default:default2V
B	swervolf/swerv_eh1/swerv/exu/mul_e1/low_e3_ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2-
clk_gen/dout[0]_i_1__42052default:default2
132default:default2?
?	swervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/dec_nonblock_load_freeze_dc3ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/lsu_full_hit_dc3ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/clkdomain/lsu_freeze_c1_dc3_clkenff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_hit_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/hitable_dc23_ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2*
clk_gen/dout[0]_i_2__12default:default2
52default:default2?
?	swervolf/swerv_eh1/swerv/exu/mul_e1/ld_rs1_byp_e1_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/exu/mul_e1/ld_rs2_byp_e1_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/exu/mul_e1/low_e1_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/exu/mul_e1/rs1_sign_e1_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/exu/mul_e1/rs2_sign_e1_ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5522default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[1].TAG_DIRTY[6].TAG_PACKET[1].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[1].TAG_DIRTY[6].TAG_PACKET[2].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[1].TAG_DIRTY[6].TAG_PACKET[3].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[1].TAG_DIRTY[6].TAG_PACKET[4].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[1].TAG_DIRTY[6].TAG_PACKET[5].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5532default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5542default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5552default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5562default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[2].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5572default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[0].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5582default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[1].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5592default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[2].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5602default:default2
2562default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[0].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[1].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[2].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[3].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_DIRTY[3].TAG_DIRTY[0].TAG_PACKET[4].TAG_DIRTY_WAYS[3].dc_way_tagdirty_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5802default:default2
72default:default2?
?	swervolf/swerv_eh1/swerv/lsu/bus_intf/ldst_dual_dc2ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/access_fault_dc2ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/addr_external_dc2ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/addr_in_dccm_dc2ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/addr_in_pic_dc2ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5812default:default2
32default:default2?
?	swervolf/swerv_eh1/swerv/lsu/clkdomain/lsu_freeze_c1_dc1_clkenff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_vlddc1ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_i0_valid_dc1ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5822default:default2
22default:default2?
?	swervolf/swerv_eh1/swerv/lsu/clkdomain/lsu_freeze_c1_dc4_clkenff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/lsu_i0_valid_dc4ff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__5832default:default2
982default:default2?
?	swervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_errorff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ibuf_timerff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ibuf_timerff/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ibuf_timerff/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/bus_intf/bus_buffer/ld_bus_dataff/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7462default:default2
92default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/ifu_iccm_reg_acc_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/unc_miss_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/mb_rep_wayf2_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/mb_rep_wayf2_ff/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/mb_rep_wayf2_ff/dout_reg[2] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7472default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7482default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7492default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7502default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7512default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].ic_way0_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7522default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].ic_way1_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7532default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].ic_way2_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__7542default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].ic_way3_tagvalid_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8652default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[0].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[10].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[11].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[12].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[13].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8662default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[0].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[10].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[11].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[12].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[13].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8672default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[0].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[10].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[11].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[12].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[13].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8682default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[0].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[10].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[11].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[12].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[3].TAG_WAIT[13].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8692default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[0].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[10].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[11].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[12].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[13].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8702default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[0].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[10].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[11].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[12].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[13].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8712default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[0].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[10].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[11].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[12].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[13].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8722default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[0].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[10].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[11].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[12].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[2].TAG_WAIT[13].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8732default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[0].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[10].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[11].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[12].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[13].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8742default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[0].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[10].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[11].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[12].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[13].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8752default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[0].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[10].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[11].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[12].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[13].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8762default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[0].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[10].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[11].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[12].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[1].TAG_WAIT[13].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8772default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[0].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[10].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[11].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[12].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[13].TAG_WAIT_WAYS[3].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8782default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[0].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[10].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[11].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[12].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[13].TAG_WAIT_WAYS[2].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8792default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[0].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[10].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[11].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[12].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[13].TAG_WAIT_WAYS[1].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8802default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[0].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[10].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[11].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[12].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_WAIT[0].TAG_WAIT[13].TAG_WAIT_WAYS[0].dc_way_tagwait_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8812default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[0].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[10].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[11].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[12].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[13].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8822default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[0].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[10].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[11].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[12].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[13].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8832default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[0].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[10].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[11].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[12].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[13].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8842default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[0].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[10].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[11].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[12].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[3].TAG_VALID[13].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8852default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[0].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[10].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[11].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[12].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[13].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8862default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[0].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[10].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[11].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[12].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[13].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8872default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[0].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[10].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[11].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[12].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[13].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8882default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[0].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[10].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[11].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[12].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[2].TAG_VALID[13].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8892default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8902default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8912default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8922default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[0].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[10].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[11].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[12].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[1].TAG_VALID[13].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8932default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].TAG_VALID_WAYS[3].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8942default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].TAG_VALID_WAYS[2].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8952default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].TAG_VALID_WAYS[1].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[0]_i_2__8962default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[0].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[10].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[11].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[12].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_TAG_VALID[0].TAG_VALID[13].TAG_VALID_WAYS[0].dc_way_tagvalid_dup/dffsc/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1432default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[0].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[0].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[0].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[0].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[0].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1442default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[1].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[1].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[1].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[1].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[1].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1452default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[2].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[2].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[2].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[2].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[2].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1462default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[3].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[3].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[3].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[3].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[3].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1472default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[4].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[4].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[4].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[4].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[4].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1482default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[5].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[5].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[5].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[5].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[5].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1492default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[6].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[6].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[6].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[6].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[6].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__1502default:default2
322default:default2?
?	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[7].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[7].BHT_CLK_GROUP[0].BHT_FLOPS[0].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[7].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[7].BHT_CLK_GROUP[0].BHT_FLOPS[10].bht_bank/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/bp/BANKS[7].BHT_CLK_GROUP[0].BHT_FLOPS[11].bht_bank/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[1]_i_2__172default:default2
22default:default2?
|	swervolf/swerv_eh1/swerv/dbg/axi_bresp_ff/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/dbg/axi_rresp_ff/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__2142default:default2
642default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[3].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[3].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[3].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[3].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[3].WAY_COUNT[11].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__2152default:default2
642default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[2].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[2].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[2].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[2].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[2].WAY_COUNT[11].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__2162default:default2
642default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[1].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[1].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[1].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[1].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[1].WAY_COUNT[11].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[1]_i_2__2172default:default2
642default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[0].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[0].WAY_COUNT[0].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[0].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[0].WAY_COUNT[10].dc_way_count_dup/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/CLK_GRP_WAY_COUNT[0].WAY_COUNT[11].dc_way_count_dup/dffs/dout_reg[0] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__362default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[0].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[0].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[0].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[0].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[0].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__372default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[1].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[1].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[1].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[1].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[1].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__382default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[2].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[2].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[2].WAY_STATUS[1].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[2].WAY_STATUS[2].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[2].WAY_STATUS[2].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__392default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[3].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[3].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[3].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[3].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[3].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__402default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[4].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[4].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[4].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[4].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[4].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__412default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[5].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[5].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[5].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[5].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[5].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__422default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[6].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[6].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[6].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[6].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[6].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2+
clk_gen/dout[2]_i_2__432default:default2
242default:default2?
?	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[7].WAY_STATUS[0].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[7].WAY_STATUS[0].ic_way_status/dffs/dout_reg[1] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[7].WAY_STATUS[0].ic_way_status/dffs/dout_reg[2] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[7].WAY_STATUS[1].ic_way_status/dffs/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/ifu/mem_ctl/CLK_GRP_WAY_STATUS[7].WAY_STATUS[1].ic_way_status/dffs/dout_reg[1] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/dout[31]_i_2__402default:default2
782default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dccm_ctl/Gen_dccm_enable.dccm_data_hi_ff/dout_reg[0] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dccm_ctl/Gen_dccm_enable.dccm_data_hi_ff/dout_reg[10] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dccm_ctl/Gen_dccm_enable.dccm_data_hi_ff/dout_reg[11] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dccm_ctl/Gen_dccm_enable.dccm_data_hi_ff/dout_reg[12] {FDCE}
	swervolf/swerv_eh1/swerv/lsu/dccm_ctl/Gen_dccm_enable.dccm_data_hi_ff/dout_reg[13] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2.
clk_gen/ram_core_reg_0_i_12default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[0].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__02default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[1].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[1].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[1].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__12default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[2].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[2].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[2].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__22default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[3].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[3].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[3].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__32default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[4].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[4].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[4].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__42default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[5].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[5].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[5].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__52default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[6].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[6].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[6].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place21
clk_gen/ram_core_reg_0_i_1__62default:default2
32default:default2?
?	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[7].dccm_bank/ram_core_reg_0 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[7].dccm_bank/ram_core_reg_1 {RAMB36E1}
	swervolf/swerv_eh1/mem/Gen_dccm_enable.dccm/mem_bank[7].dccm_bank/ram_core_reg_2 {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2/
clk_gen/ram_core_reg_i_1__22default:default2
12default:default2t
`	swervolf/swerv_eh1/mem/icm/ic_tag_inst/WAYS[0].ICACHE_SZ_16.ic_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place20
clk_gen/ram_core_reg_i_1__292default:default2
22default:default2?
?	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[0].DC_DEPTH_TAG_MUL[0].dc_way_tag/ram_core_reg {RAMB18E1}
	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[0].DC_DEPTH_TAG_MUL[1].dc_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2/
clk_gen/ram_core_reg_i_1__32default:default2
12default:default2t
`	swervolf/swerv_eh1/mem/icm/ic_tag_inst/WAYS[1].ICACHE_SZ_16.ic_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place20
clk_gen/ram_core_reg_i_1__302default:default2
22default:default2?
?	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[1].DC_DEPTH_TAG_MUL[0].dc_way_tag/ram_core_reg {RAMB18E1}
	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[1].DC_DEPTH_TAG_MUL[1].dc_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place20
clk_gen/ram_core_reg_i_1__312default:default2
22default:default2?
?	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[2].DC_DEPTH_TAG_MUL[0].dc_way_tag/ram_core_reg {RAMB18E1}
	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[2].DC_DEPTH_TAG_MUL[1].dc_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place20
clk_gen/ram_core_reg_i_1__322default:default2
22default:default2?
?	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[3].DC_DEPTH_TAG_MUL[0].dc_way_tag/ram_core_reg {RAMB18E1}
	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[3].DC_DEPTH_TAG_MUL[1].dc_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2/
clk_gen/ram_core_reg_i_1__42default:default2
12default:default2t
`	swervolf/swerv_eh1/mem/icm/ic_tag_inst/WAYS[2].ICACHE_SZ_16.ic_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2/
clk_gen/ram_core_reg_i_1__52default:default2
12default:default2t
`	swervolf/swerv_eh1/mem/icm/ic_tag_inst/WAYS[3].ICACHE_SZ_16.ic_way_tag/ram_core_reg {RAMB18E1}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place2,
clk_gen/rptc_hrc[31]_i_32default:default2
322default:default2?
?	swervolf/timer_ptc/rptc_hrc_reg[0] {FDCE}
	swervolf/timer_ptc/rptc_hrc_reg[10] {FDCE}
	swervolf/timer_ptc/rptc_hrc_reg[11] {FDCE}
	swervolf/timer_ptc/rptc_hrc_reg[12] {FDCE}
	swervolf/timer_ptc/rptc_hrc_reg[13] {FDCE}
2default:defaultZ30-568h px? 
?
?A LUT '%s' is driving clock pin of %s registers. This could lead to large hold time violations. First few involved registers are:
%s524*place28
$swervolf/timer_ptc/rptc_cntr[31]_i_32default:default2
322default:default2?
?	swervolf/timer_ptc/rptc_cntr_reg[0] {FDCE}
	swervolf/timer_ptc/rptc_cntr_reg[10] {FDCE}
	swervolf/timer_ptc/rptc_cntr_reg[11] {FDCE}
	swervolf/timer_ptc/rptc_cntr_reg[12] {FDCE}
	swervolf/timer_ptc/rptc_cntr_reg[13] {FDCE}
2default:defaultZ30-568h px? 
?
?Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2 
Place 30-5682default:default2
1002default:defaultZ17-14h px? 
g
RPhase 1.2 IO Placement/ Clock Placement/ Build Placer Device | Checksum: d8920e2c
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:10 ; elapsed = 00:00:05 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 4059 ; free virtual = 110242default:defaulth px? 
}

Phase %s%s
101*constraints2
1.3 2default:default2.
Build Placer Netlist Model2default:defaultZ18-101h px? 
P
;Phase 1.3 Build Placer Netlist Model | Checksum: 19a473d19
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:29 ; elapsed = 00:00:14 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3386 ; free virtual = 103522default:defaulth px? 
z

Phase %s%s
101*constraints2
1.4 2default:default2+
Constrain Clocks/Macros2default:defaultZ18-101h px? 
M
8Phase 1.4 Constrain Clocks/Macros | Checksum: 19a473d19
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:29 ; elapsed = 00:00:14 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3386 ; free virtual = 103522default:defaulth px? 
I
4Phase 1 Placer Initialization | Checksum: 19a473d19
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:29 ; elapsed = 00:00:14 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3381 ; free virtual = 103472default:defaulth px? 
q

Phase %s%s
101*constraints2
2 2default:default2$
Global Placement2default:defaultZ18-101h px? 
p

Phase %s%s
101*constraints2
2.1 2default:default2!
Floorplanning2default:defaultZ18-101h px? 
C
.Phase 2.1 Floorplanning | Checksum: 1562914d4
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:39 ; elapsed = 00:00:18 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3342 ; free virtual = 103082default:defaulth px? 
x

Phase %s%s
101*constraints2
2.2 2default:default2)
Global Placement Core2default:defaultZ18-101h px? 
?

Phase %s%s
101*constraints2
2.2.1 2default:default20
Physical Synthesis In Placer2default:defaultZ18-101h px? 
v
7Found %s candidate LUT instances to create LUTNM shape
536*physynth2
18612default:defaultZ32-1018h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
12922default:default2!
nets or cells2default:default2
7582default:default2
cells2default:default2
5342default:default2
cells2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
K
)No high fanout nets found in the design.
65*physynthZ32-65h px? 
?
$Optimized %s %s. Created %s new %s.
216*physynth2
02default:default2
net2default:default2
02default:default2
instance2default:defaultZ32-232h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
I
'No nets found for fanout-optimization.
64*physynthZ32-64h px? 
?
$Optimized %s %s. Created %s new %s.
216*physynth2
02default:default2
net2default:default2
02default:default2
instance2default:defaultZ32-232h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[6]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[6]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_19>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_192default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__4	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[8]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[8]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[4]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[4]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[10]Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[10]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[6]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[6]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_24>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_242default:default2?
Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_27	Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_272default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[2]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[2]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__3	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[0]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[0]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__3	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[7]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[9]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[9]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[1]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[1]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__6	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[0]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[0]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__6	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[9]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[9]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[2]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[2]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__6	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[7]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[5]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[5]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[3]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[3]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[8]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[8]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[1]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[1]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__3	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
7swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[10]7swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[10]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[5]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[5]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[4]6swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ADR[4]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__6	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[3]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_18[3]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__3	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[10]Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[10]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_17>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_172default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__3	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[2]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[2]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__2	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[6]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[6]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[8]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[8]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[7]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[1]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[1]2default:default2?
Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11	Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_112default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
=swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_2=swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__0	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[1]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[1]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__2	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[4]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[4]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[0]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[0]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__2	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[5]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[5]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[3]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[3]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[7]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[7]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[9]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_16[9]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__2	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[4]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[4]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_82default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[2]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[2]2default:default2?
Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10	Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_102default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[3]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[3]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_92default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[5]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[5]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_72default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[2]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[2]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__5	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_23>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_232default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__6	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[6]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[6]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[4]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[4]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[0]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[0]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__5	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[0]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[0]2default:default2?
Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12	Cswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_122default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[7]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[10]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[10]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[6]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[6]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[5]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[5]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[1]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[1]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__5	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[3]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[3]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[1]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[1]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__0	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_13>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_132default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__1	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__12default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[0]Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[0]2default:default2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_14	Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_142default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Qswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/DIBDI[0]Qswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/DIBDI[0]2default:default2?
tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_mem/dc_tag_inst/pargen/ram_core_reg_i_18	tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_mem/dc_tag_inst/pargen/ram_core_reg_i_182default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[6]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[6]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_6__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[0]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[0]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__0	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[8]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[8]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[9]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[9]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[4]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[4]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[7]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[3]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[3]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[2]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[2]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__0	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_10__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Wswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/dc_wr_data[30]Wswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/dc_wr_data[30]2default:default2?
\swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/ram_core_reg_i_2__0	\swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/ram_core_reg_i_2__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[5]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[5]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[9]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[9]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_32default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[1]Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[1]2default:default2?
Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_9	Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_92default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dc_rw_tag_addr[7]Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dc_rw_tag_addr[7]2default:default2?
Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_17	Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_172default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[8]@swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_0[8]2default:default2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4	Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[10]Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_12[10]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__0	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[9]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[9]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__4	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_21>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_212default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__5	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[0]Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[0]2default:default2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_10	Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_102default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[4]Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[4]2default:default2?
Yswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_5	Yswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[1]Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[1]2default:default2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_13	Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_132default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[3]Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[3]2default:default2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_11	Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_112default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[5]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[5]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__4	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_7__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[1]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[1]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__4	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_11__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[4]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[4]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__4	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_8__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[6]Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[6]2default:default2?
Pswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_3__2	Pswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_3__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Wswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/dc_wr_data[10]Wswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/dc_wr_data[10]2default:default2?
Yswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/ram_core_reg_i_1	Yswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc2ff/genblock.dff/dffs/ram_core_reg_i_12default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[0]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[0]2default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__4	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_12__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[7]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__4	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[10]Bswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[10]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_2__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Sswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dout_reg[62][36]Sswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dout_reg[62][36]2default:default2?
Vswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_1__0	Vswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_1__02default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[8]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[8]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_4__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[4]Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[4]2default:default2?
Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_6	Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_62default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[5]Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[5]2default:default2?
Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_4	Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dc_rw_tag_addr[9]Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dc_rw_tag_addr[9]2default:default2?
Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_15	Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_152default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Uswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dc_rw_tag_addr[10]Uswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/dc_rw_tag_addr[10]2default:default2?
Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_20	Tswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sadc2ff/genblock.dff/dffs/ram_core_reg_i_202default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[3]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_20[3]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__4	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_9__42default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[9]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_22[9]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__5	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_3__52default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[2]Nswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/dc_rw_tag_addr[2]2default:default2?
Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_8	Mswervolf/swerv_eh1/swerv/lsu/dc_ctl/dc_freeze_originff/dffsc/ram_core_reg_i_82default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_14[7]Aswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_14[7]2default:default2?
Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__1	Eswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_5__12default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_15>swervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/dout_reg[2]_152default:default2?
Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__2	Fswervolf/swerv_eh1/swerv/lsu/stbuf/RdPtrff/dffs/ram_core_reg_0_i_13__22default:default8Z32-117h px? 
?
DNet %s could not be optimized because driver %s could not be cloned
117*physynth2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[2]Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/dc_rw_tag_addr[2]2default:default2?
Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_12	Zswervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsu_pkt_dc1ff/genblock.dff/dffs/ram_core_reg_i_122default:default8Z32-117h px? 
?
?Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2"
Physopt 32-1172default:default2
1002default:defaultZ17-14h px? 
P
.No nets found for critical-cell optimization.
68*physynthZ32-68h px? 
?
$Optimized %s %s. Created %s new %s.
216*physynth2
02default:default2
net2default:default2
02default:default2
instance2default:defaultZ32-232h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
j
FNo candidate cells for DSP register optimization found in the design.
274*physynthZ32-456h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
22default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
h
DNo candidate cells for SRL register optimization found in the design349*physynthZ32-677h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
i
ENo candidate cells for BRAM register optimization found in the design297*physynthZ32-526h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
R
.No candidate nets found for HD net replication521*physynthZ32-949h px? 
?
aEnd %s Pass. Optimized %s %s. Created %s new %s, deleted %s existing %s and moved %s existing %s
415*physynth2
12default:default2
02default:default2
net or cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:default2
02default:default2
cell2default:defaultZ32-775h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.012default:default2
00:00:00.022default:default2
3876.7272default:default2
0.0002default:default2
33412default:default2
103102default:defaultZ17-722h px? 
B
-
Summary of Physical Synthesis Optimizations
*commonh px? 
B
-============================================
*commonh px? 


*commonh px? 


*commonh px? 
?
?-----------------------------------------------------------------------------------------------------------------------------------------------------------
*commonh px? 
?
?|  Optimization                                     |  Added Cells  |  Removed Cells  |  Optimized Cells/Nets  |  Dont Touch  |  Iterations  |  Elapsed   |
-----------------------------------------------------------------------------------------------------------------------------------------------------------
*commonh px? 
?
?|  LUT Combining                                    |          758  |            534  |                  1292  |           0  |           1  |  00:00:16  |
|  Very High Fanout                                 |            0  |              0  |                     0  |           0  |           1  |  00:00:01  |
|  Fanout                                           |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  Critical Cell                                    |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  DSP Register                                     |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  Shift Register to Pipeline                       |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  Shift Register                                   |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  BRAM Register                                    |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  Dynamic/Static Region Interface Net Replication  |            0  |              0  |                     0  |           0  |           1  |  00:00:00  |
|  Total                                            |          758  |            534  |                  1292  |           0  |           9  |  00:00:17  |
-----------------------------------------------------------------------------------------------------------------------------------------------------------
*commonh px? 


*commonh px? 


*commonh px? 
T
?Phase 2.2.1 Physical Synthesis In Placer | Checksum: 149169ac7
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:02:58 ; elapsed = 00:01:30 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3334 ; free virtual = 103032default:defaulth px? 
K
6Phase 2.2 Global Placement Core | Checksum: 15a5b4f7e
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:03:14 ; elapsed = 00:01:36 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3328 ; free virtual = 102982default:defaulth px? 
D
/Phase 2 Global Placement | Checksum: 15a5b4f7e
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:03:14 ; elapsed = 00:01:36 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3369 ; free virtual = 103382default:defaulth px? 
q

Phase %s%s
101*constraints2
3 2default:default2$
Detail Placement2default:defaultZ18-101h px? 
}

Phase %s%s
101*constraints2
3.1 2default:default2.
Commit Multi Column Macros2default:defaultZ18-101h px? 
P
;Phase 3.1 Commit Multi Column Macros | Checksum: 12a0b2ae3
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:03:24 ; elapsed = 00:01:41 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3358 ; free virtual = 103272default:defaulth px? 


Phase %s%s
101*constraints2
3.2 2default:default20
Commit Most Macros & LUTRAMs2default:defaultZ18-101h px? 
R
=Phase 3.2 Commit Most Macros & LUTRAMs | Checksum: 1223880fd
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:03:41 ; elapsed = 00:01:49 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3355 ; free virtual = 103252default:defaulth px? 
y

Phase %s%s
101*constraints2
3.3 2default:default2*
Area Swap Optimization2default:defaultZ18-101h px? 
L
7Phase 3.3 Area Swap Optimization | Checksum: 166c6c8fd
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:03:42 ; elapsed = 00:01:49 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3356 ; free virtual = 103252default:defaulth px? 
?

Phase %s%s
101*constraints2
3.4 2default:default22
Pipeline Register Optimization2default:defaultZ18-101h px? 
S
>Phase 3.4 Pipeline Register Optimization | Checksum: ca1ee00f
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:03:42 ; elapsed = 00:01:49 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3356 ; free virtual = 103252default:defaulth px? 
t

Phase %s%s
101*constraints2
3.5 2default:default2%
Fast Optimization2default:defaultZ18-101h px? 
G
2Phase 3.5 Fast Optimization | Checksum: 11bffabec
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:04:08 ; elapsed = 00:02:04 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3361 ; free virtual = 103302default:defaulth px? 


Phase %s%s
101*constraints2
3.6 2default:default20
Small Shape Detail Placement2default:defaultZ18-101h px? 
R
=Phase 3.6 Small Shape Detail Placement | Checksum: 15fc5df46
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:04:35 ; elapsed = 00:02:30 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3262 ; free virtual = 102362default:defaulth px? 
u

Phase %s%s
101*constraints2
3.7 2default:default2&
Re-assign LUT pins2default:defaultZ18-101h px? 
H
3Phase 3.7 Re-assign LUT pins | Checksum: 1cb4bc0d4
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:04:38 ; elapsed = 00:02:32 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3266 ; free virtual = 102412default:defaulth px? 
?

Phase %s%s
101*constraints2
3.8 2default:default22
Pipeline Register Optimization2default:defaultZ18-101h px? 
S
>Phase 3.8 Pipeline Register Optimization | Checksum: c957d9de
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:04:39 ; elapsed = 00:02:33 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3266 ; free virtual = 102412default:defaulth px? 
t

Phase %s%s
101*constraints2
3.9 2default:default2%
Fast Optimization2default:defaultZ18-101h px? 
F
1Phase 3.9 Fast Optimization | Checksum: dd18a796
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:05:06 ; elapsed = 00:02:46 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3287 ; free virtual = 102572default:defaulth px? 
C
.Phase 3 Detail Placement | Checksum: dd18a796
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:05:06 ; elapsed = 00:02:47 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3287 ; free virtual = 102572default:defaulth px? 
?

Phase %s%s
101*constraints2
4 2default:default2<
(Post Placement Optimization and Clean-Up2default:defaultZ18-101h px? 
{

Phase %s%s
101*constraints2
4.1 2default:default2,
Post Commit Optimization2default:defaultZ18-101h px? 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px? 
?

Phase %s%s
101*constraints2
4.1.1 2default:default2/
Post Placement Optimization2default:defaultZ18-101h px? 
U
@Post Placement Optimization Initialization | Checksum: 7d80cd8e
*commonh px? 
u

Phase %s%s
101*constraints2
4.1.1.1 2default:default2"
BUFG Insertion2default:defaultZ18-101h px? 
?
PProcessed net %s, BUFG insertion was skipped due to placement/routing conflicts.32*	placeflow2,
clk_gen/o_rst_core_reg_12default:defaultZ46-33h px? 
?
PProcessed net %s, BUFG insertion was skipped due to placement/routing conflicts.32*	placeflow2,
clk_gen/o_rst_core_reg_22default:defaultZ46-33h px? 
?
PProcessed net %s, BUFG insertion was skipped due to placement/routing conflicts.32*	placeflow2$
clk_gen/rst_core2default:defaultZ46-33h px? 
?
PProcessed net %s, BUFG insertion was skipped due to placement/routing conflicts.32*	placeflow2%
ddr2/ldc/FDPE_1_02default:defaultZ46-33h px? 
?
?BUFG insertion identified %s candidate nets. Inserted BUFG: %s, Replicated BUFG Driver: %s, Skipped due to Placement/Routing Conflicts: %s, Skipped due to Timing Degradation: %s, Skipped due to Illegal Netlist: %s.43*	placeflow2
42default:default2
02default:default2
02default:default2
42default:default2
02default:default2
02default:defaultZ46-56h px? 
G
2Phase 4.1.1.1 BUFG Insertion | Checksum: 7d80cd8e
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:05:28 ; elapsed = 00:02:55 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3811 ; free virtual = 107812default:defaulth px? 
?
hPost Placement Timing Summary WNS=%s. For the most accurate timing information please run report_timing.610*place2
-6.5812default:defaultZ30-746h px? 
R
=Phase 4.1.1 Post Placement Optimization | Checksum: be867a73
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:25 ; elapsed = 00:03:34 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3807 ; free virtual = 107772default:defaulth px? 
M
8Phase 4.1 Post Commit Optimization | Checksum: be867a73
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:25 ; elapsed = 00:03:34 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3807 ; free virtual = 107772default:defaulth px? 
y

Phase %s%s
101*constraints2
4.2 2default:default2*
Post Placement Cleanup2default:defaultZ18-101h px? 
K
6Phase 4.2 Post Placement Cleanup | Checksum: be867a73
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:26 ; elapsed = 00:03:35 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3808 ; free virtual = 107782default:defaulth px? 
s

Phase %s%s
101*constraints2
4.3 2default:default2$
Placer Reporting2default:defaultZ18-101h px? 
E
0Phase 4.3 Placer Reporting | Checksum: be867a73
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:26 ; elapsed = 00:03:35 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3814 ; free virtual = 107842default:defaulth px? 
z

Phase %s%s
101*constraints2
4.4 2default:default2+
Final Placement Cleanup2default:defaultZ18-101h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.022default:default2
00:00:00.022default:default2
3876.7272default:default2
0.0002default:default2
38142default:default2
107842default:defaultZ17-722h px? 
L
7Phase 4.4 Final Placement Cleanup | Checksum: b9e0671e
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:26 ; elapsed = 00:03:36 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3814 ; free virtual = 107842default:defaulth px? 
[
FPhase 4 Post Placement Optimization and Clean-Up | Checksum: b9e0671e
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:27 ; elapsed = 00:03:36 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3814 ; free virtual = 107842default:defaulth px? 
=
(Ending Placer Task | Checksum: 24f1d66a
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:27 ; elapsed = 00:03:36 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3814 ; free virtual = 107842default:defaulth px? 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1842default:default2
1432default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
place_design2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
place_design: 2default:default2
00:07:312default:default2
00:03:382default:default2
3876.7272default:default2
0.0002default:default2
39052default:default2
108752default:defaultZ17-722h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.022default:default2
00:00:00.022default:default2
3876.7272default:default2
0.0002default:default2
39052default:default2
108762default:defaultZ17-722h px? 
H
&Writing timing data to binary archive.266*timingZ38-480h px? 
D
Writing placer database...
1603*designutilsZ20-1893h px? 
=
Writing XDEF routing.
211*designutilsZ20-211h px? 
J
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px? 
J
#Writing XDEF routing special nets.
210*designutilsZ20-210h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2)
Write XDEF Complete: 2default:default2
00:00:082default:default2
00:00:022default:default2
3876.7272default:default2
0.0002default:default2
37792default:default2
108442default:defaultZ17-722h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2j
V/home/pepitoigrillo/Documents/RVfpga/project_1/project_1.runs/impl_1/rvfpga_placed.dcp2default:defaultZ17-1381h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2&
write_checkpoint: 2default:default2
00:00:242default:default2
00:00:162default:default2
3876.7272default:default2
0.0002default:default2
38682default:default2
108712default:defaultZ17-722h px? 
a
%s4*runtcl2E
1Executing : report_io -file rvfpga_io_placed.rpt
2default:defaulth px? 
?
?report_io: Time (s): cpu = 00:00:00.09 ; elapsed = 00:00:00.13 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3860 ; free virtual = 10862
*commonh px? 
?
%s4*runtcl2x
dExecuting : report_utilization -file rvfpga_utilization_placed.rpt -pb rvfpga_utilization_placed.pb
2default:defaulth px? 
~
%s4*runtcl2b
NExecuting : report_control_sets -verbose -file rvfpga_control_sets_placed.rpt
2default:defaulth px? 
?
?report_control_sets: Time (s): cpu = 00:00:00.14 ; elapsed = 00:00:00.19 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3870 ; free virtual = 10872
*commonh px? 


End Record