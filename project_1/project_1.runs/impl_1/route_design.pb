
Q
Command: %s
53*	vivadotcl2 
route_design2default:defaultZ4-113h px? 
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
route_design2default:defaultZ4-22h px? 
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
b
DRC finished with %s
79*	vivadotcl2(
0 Errors, 1 Warnings2default:defaultZ4-198h px? 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px? 
V

Starting %s Task
103*constraints2
Routing2default:defaultZ18-103h px? 
}
BMultithreading enabled for route_design using a maximum of %s CPUs17*	routeflow2
62default:defaultZ35-254h px? 
p

Phase %s%s
101*constraints2
1 2default:default2#
Build RT Design2default:defaultZ18-101h px? 
B
-Phase 1 Build RT Design | Checksum: a231f3aa
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:23 ; elapsed = 00:00:15 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3617 ; free virtual = 106522default:defaulth px? 
v

Phase %s%s
101*constraints2
2 2default:default2)
Router Initialization2default:defaultZ18-101h px? 
o

Phase %s%s
101*constraints2
2.1 2default:default2 
Create Timer2default:defaultZ18-101h px? 
A
,Phase 2.1 Create Timer | Checksum: a231f3aa
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:23 ; elapsed = 00:00:15 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3617 ; free virtual = 106532default:defaulth px? 
{

Phase %s%s
101*constraints2
2.2 2default:default2,
Fix Topology Constraints2default:defaultZ18-101h px? 
M
8Phase 2.2 Fix Topology Constraints | Checksum: a231f3aa
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:24 ; elapsed = 00:00:15 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3586 ; free virtual = 106232default:defaulth px? 
t

Phase %s%s
101*constraints2
2.3 2default:default2%
Pre Route Cleanup2default:defaultZ18-101h px? 
F
1Phase 2.3 Pre Route Cleanup | Checksum: a231f3aa
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:24 ; elapsed = 00:00:15 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3586 ; free virtual = 106232default:defaulth px? 
p

Phase %s%s
101*constraints2
2.4 2default:default2!
Update Timing2default:defaultZ18-101h px? 
C
.Phase 2.4 Update Timing | Checksum: 1d8f0eeed
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:01:09 ; elapsed = 00:00:33 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3529 ; free virtual = 105652default:defaulth px? 
?
Intermediate Timing Summary %s164*route2O
;| WNS=-5.337 | TNS=-22424.123| WHS=-2.929 | THS=-2227.289|
2default:defaultZ35-416h px? 
I
4Phase 2 Router Initialization | Checksum: 152f7fdde
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:01:31 ; elapsed = 00:00:39 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3530 ; free virtual = 105672default:defaulth px? 
p

Phase %s%s
101*constraints2
3 2default:default2#
Initial Routing2default:defaultZ18-101h px? 
C
.Phase 3 Initial Routing | Checksum: 1894d6286
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:02:06 ; elapsed = 00:00:48 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3507 ; free virtual = 105442default:defaulth px? 
?
>Design has %s pins with tight setup and hold constraints.

%s
244*route2
41762default:default2?
?The top 5 pins with tight setup and hold constraints:

+--------------------------+--------------------------+----------------------------------------------------------------------------------------------------------+
|       Launch Clock       |      Capture Clock       |                                                 Pin                                                      |
+--------------------------+--------------------------+----------------------------------------------------------------------------------------------------------+
|                 clk_core |                 clk_core |                            swervolf/swerv_eh1/swerv/dma_ctrl/GenFifo[3].fifo_done_dff/dffsc/dout_reg[0]/D|
|                 clk_core |                 clk_core |                         swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sddc1ff/genblock.dff/dffs/dout_reg[37]/D|
|                 clk_core |                 clk_core |                            swervolf/swerv_eh1/swerv/dma_ctrl/GenFifo[0].fifo_done_dff/dffsc/dout_reg[0]/D|
|                 clk_core |                 clk_core |                            swervolf/swerv_eh1/swerv/dma_ctrl/GenFifo[2].fifo_done_dff/dffsc/dout_reg[0]/D|
|                 clk_core |                 clk_core |                         swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/sddc1ff/genblock.dff/dffs/dout_reg[34]/D|
+--------------------------+--------------------------+----------------------------------------------------------------------------------------------------------+

File with complete list of pins: tight_setup_hold_pins.txt
2default:defaultZ35-580h px? 
s

Phase %s%s
101*constraints2
4 2default:default2&
Rip-up And Reroute2default:defaultZ18-101h px? 
u

Phase %s%s
101*constraints2
4.1 2default:default2&
Global Iteration 02default:defaultZ18-101h px? 
?
Intermediate Timing Summary %s164*route2M
9| WNS=-6.735 | TNS=-30936.022| WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px? 
H
3Phase 4.1 Global Iteration 0 | Checksum: 1bd064c8e
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:05:23 ; elapsed = 00:02:02 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3474 ; free virtual = 105122default:defaulth px? 
u

Phase %s%s
101*constraints2
4.2 2default:default2&
Global Iteration 12default:defaultZ18-101h px? 
?
Intermediate Timing Summary %s164*route2M
9| WNS=-6.694 | TNS=-29952.369| WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px? 
H
3Phase 4.2 Global Iteration 1 | Checksum: 101a6cfd9
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:04 ; elapsed = 00:02:48 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3458 ; free virtual = 105002default:defaulth px? 
u

Phase %s%s
101*constraints2
4.3 2default:default2&
Global Iteration 22default:defaultZ18-101h px? 
H
3Phase 4.3 Global Iteration 2 | Checksum: 1fe85d9c4
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:45 ; elapsed = 00:03:06 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3480 ; free virtual = 105182default:defaulth px? 
F
1Phase 4 Rip-up And Reroute | Checksum: 1fe85d9c4
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:07:45 ; elapsed = 00:03:06 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3480 ; free virtual = 105182default:defaulth px? 
|

Phase %s%s
101*constraints2
5 2default:default2/
Delay and Skew Optimization2default:defaultZ18-101h px? 
p

Phase %s%s
101*constraints2
5.1 2default:default2!
Delay CleanUp2default:defaultZ18-101h px? 
r

Phase %s%s
101*constraints2
5.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px? 
E
0Phase 5.1.1 Update Timing | Checksum: 2043bb40d
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:02 ; elapsed = 00:03:12 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3492 ; free virtual = 105312default:defaulth px? 
?
Intermediate Timing Summary %s164*route2M
9| WNS=-6.694 | TNS=-29948.749| WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px? 
C
.Phase 5.1 Delay CleanUp | Checksum: 139c4e55a
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:03 ; elapsed = 00:03:12 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3489 ; free virtual = 105282default:defaulth px? 
z

Phase %s%s
101*constraints2
5.2 2default:default2+
Clock Skew Optimization2default:defaultZ18-101h px? 
M
8Phase 5.2 Clock Skew Optimization | Checksum: 139c4e55a
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:03 ; elapsed = 00:03:12 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3489 ; free virtual = 105282default:defaulth px? 
O
:Phase 5 Delay and Skew Optimization | Checksum: 139c4e55a
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:03 ; elapsed = 00:03:13 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3489 ; free virtual = 105282default:defaulth px? 
n

Phase %s%s
101*constraints2
6 2default:default2!
Post Hold Fix2default:defaultZ18-101h px? 
p

Phase %s%s
101*constraints2
6.1 2default:default2!
Hold Fix Iter2default:defaultZ18-101h px? 
r

Phase %s%s
101*constraints2
6.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px? 
E
0Phase 6.1.1 Update Timing | Checksum: 17c3847bd
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:32 ; elapsed = 00:03:23 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3444 ; free virtual = 104822default:defaulth px? 
?
Intermediate Timing Summary %s164*route2M
9| WNS=-6.682 | TNS=-29923.460| WHS=-2.033 | THS=-14.249|
2default:defaultZ35-416h px? 
C
.Phase 6.1 Hold Fix Iter | Checksum: 172bc3422
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:39 ; elapsed = 00:03:29 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3408 ; free virtual = 104462default:defaulth px? 
?
?The router encountered %s pins that are both setup-critical and hold-critical and tried to fix hold violations at the expense of setup slack. Such pins are:
%s201*route2
312default:default2?
?	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__819/I1
	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[2].DC_DEPTH_TAG_MUL[0].dc_tag_c1_cgc/clkhdr/en_ff_reg/D
	swervolf/swerv_eh1/mem/dc_mem/dc_tag_inst/DC_TAG_WAYS[1].DC_DEPTH_TAG_MUL[0].dc_tag_c1_cgc/clkhdr/en_ff_reg/D
	swervolf/swerv_eh1/swerv/lsu/lsu_lsc_ctl/lsadder/dout[11]_i_5/I2
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__4117/I1
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__715/I1
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__1112/I1
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__1633/I1
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__1750/I1
	swervolf/swerv_eh1/swerv/lsu/dc_ctl/rdc_rd_ptrff/dffsc/dout[0]_i_1__1651/I1
	.. and 21 more pins.
2default:defaultZ35-468h px? 
A
,Phase 6 Post Hold Fix | Checksum: 187329218
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:40 ; elapsed = 00:03:29 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3410 ; free virtual = 104482default:defaulth px? 
o

Phase %s%s
101*constraints2
7 2default:default2"
Route finalize2default:defaultZ18-101h px? 
B
-Phase 7 Route finalize | Checksum: 1881628cd
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:40 ; elapsed = 00:03:30 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3410 ; free virtual = 104482default:defaulth px? 
v

Phase %s%s
101*constraints2
8 2default:default2)
Verifying routed nets2default:defaultZ18-101h px? 
I
4Phase 8 Verifying routed nets | Checksum: 1881628cd
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:40 ; elapsed = 00:03:30 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3409 ; free virtual = 104472default:defaulth px? 
r

Phase %s%s
101*constraints2
9 2default:default2%
Depositing Routes2default:defaultZ18-101h px? 
E
0Phase 9 Depositing Routes | Checksum: 1e765e290
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:08:44 ; elapsed = 00:03:34 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3411 ; free virtual = 104502default:defaulth px? 
t

Phase %s%s
101*constraints2
10 2default:default2&
Post Router Timing2default:defaultZ18-101h px? 
q

Phase %s%s
101*constraints2
10.1 2default:default2!
Update Timing2default:defaultZ18-101h px? 
D
/Phase 10.1 Update Timing | Checksum: 131557f86
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:09:14 ; elapsed = 00:03:44 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3415 ; free virtual = 104532default:defaulth px? 
?
Estimated Timing Summary %s
57*route2M
9| WNS=-6.682 | TNS=-29923.460| WHS=0.050  | THS=0.000  |
2default:defaultZ35-57h px? 
B
!Router estimated timing not met.
128*routeZ35-328h px? 
G
2Phase 10 Post Router Timing | Checksum: 131557f86
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:09:14 ; elapsed = 00:03:44 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3415 ; free virtual = 104532default:defaulth px? 
@
Router Completed Successfully
2*	routeflowZ35-16h px? 
?

%s
*constraints2?
?Time (s): cpu = 00:09:14 ; elapsed = 00:03:45 . Memory (MB): peak = 3876.727 ; gain = 0.000 ; free physical = 3551 ; free virtual = 105902default:defaulth px? 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
6112default:default2
1462default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
route_design2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
route_design: 2default:default2
00:09:202default:default2
00:03:482default:default2
3876.7272default:default2
0.0002default:default2
35512default:default2
105902default:defaultZ17-722h px? 
~
4The following parameters have non-default value.
%s
395*common2&
general.maxThreads2default:defaultZ17-600h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.012default:default2
00:00:00.022default:default2
3876.7272default:default2
0.0002default:default2
35512default:default2
105902default:defaultZ17-722h px? 
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
00:00:092default:default2
00:00:032default:default2
3876.7272default:default2
0.0002default:default2
33912default:default2
105522default:defaultZ17-722h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2j
V/home/pepitoigrillo/Documents/RVfpga/project_1/project_1.runs/impl_1/rvfpga_routed.dcp2default:defaultZ17-1381h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2&
write_checkpoint: 2default:default2
00:00:232default:default2
00:00:152default:default2
3876.7272default:default2
0.0002default:default2
34972default:default2
105742default:defaultZ17-722h px? 
?
%s4*runtcl2{
gExecuting : report_drc -file rvfpga_drc_routed.rpt -pb rvfpga_drc_routed.pb -rpx rvfpga_drc_routed.rpx
2default:defaulth px? 
?
Command: %s
53*	vivadotcl2n
Zreport_drc -file rvfpga_drc_routed.rpt -pb rvfpga_drc_routed.pb -rpx rvfpga_drc_routed.rpx2default:defaultZ4-113h px? 
>
IP Catalog is up to date.1232*coregenZ19-1839h px? 
P
Running DRC with %s threads
24*drc2
62default:defaultZ23-27h px? 
?
#The results of DRC are in file %s.
168*coretcl2?
Z/home/pepitoigrillo/Documents/RVfpga/project_1/project_1.runs/impl_1/rvfpga_drc_routed.rptZ/home/pepitoigrillo/Documents/RVfpga/project_1/project_1.runs/impl_1/rvfpga_drc_routed.rpt2default:default8Z2-168h px? 
\
%s completed successfully
29*	vivadotcl2

report_drc2default:defaultZ4-42h px? 
?
%s4*runtcl2?
?Executing : report_methodology -file rvfpga_methodology_drc_routed.rpt -pb rvfpga_methodology_drc_routed.pb -rpx rvfpga_methodology_drc_routed.rpx
2default:defaulth px? 
?
Command: %s
53*	vivadotcl2?
?report_methodology -file rvfpga_methodology_drc_routed.rpt -pb rvfpga_methodology_drc_routed.pb -rpx rvfpga_methodology_drc_routed.rpx2default:defaultZ4-113h px? 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px? 
Y
$Running Methodology with %s threads
74*drc2
62default:defaultZ23-133h px? 
?
2The results of Report Methodology are in file %s.
450*coretcl2?
f/home/pepitoigrillo/Documents/RVfpga/project_1/project_1.runs/impl_1/rvfpga_methodology_drc_routed.rptf/home/pepitoigrillo/Documents/RVfpga/project_1/project_1.runs/impl_1/rvfpga_methodology_drc_routed.rpt2default:default8Z2-1520h px? 
d
%s completed successfully
29*	vivadotcl2&
report_methodology2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2(
report_methodology: 2default:default2
00:04:102default:default2
00:00:562default:default2
7888.5702default:default2
4011.8442default:default2
41722default:default2
101972default:defaultZ17-722h px? 
?
%s4*runtcl2?
wExecuting : report_power -file rvfpga_power_routed.rpt -pb rvfpga_power_summary_routed.pb -rpx rvfpga_power_routed.rpx
2default:defaulth px? 
?
Command: %s
53*	vivadotcl2~
jreport_power -file rvfpga_power_routed.rpt -pb rvfpga_power_summary_routed.pb -rpx rvfpga_power_routed.rpx2default:defaultZ4-113h px? 
q
$Power model is not available for %s
23*power2*
	STARTUPE2		STARTUPE22default:default8Z33-23h px? 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px? 
K
,Running Vector-less Activity Propagation...
51*powerZ33-51h px? 
P
3
Finished Running Vector-less Activity Propagation
1*powerZ33-1h px? 
?
?Detected over-assertion of set/reset/preset/clear net with high fanouts, power estimation might not be accurate. Please run Tool - Power Constraint Wizard to set proper switching activities for control signals.282*powerZ33-332h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
6252default:default2
1472default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
report_power2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
report_power: 2default:default2
00:00:462default:default2
00:00:172default:default2
7912.5822default:default2
24.0122default:default2
41092default:default2
101472default:defaultZ17-722h px? 
?
%s4*runtcl2m
YExecuting : report_route_status -file rvfpga_route_status.rpt -pb rvfpga_route_status.pb
2default:defaulth px? 
?
%s4*runtcl2?
?Executing : report_timing_summary -max_paths 10 -file rvfpga_timing_summary_routed.rpt -pb rvfpga_timing_summary_routed.pb -rpx rvfpga_timing_summary_routed.rpx -warn_on_violation 
2default:defaulth px? 
r
UpdateTimingParams:%s.
91*timing29
% Speed grade: -1, Delay Type: min_max2default:defaultZ38-91h px? 
|
CMultithreading enabled for timing update using a maximum of %s CPUs155*timing2
62default:defaultZ38-191h px? 
?
rThe design failed to meet the timing requirements. Please see the %s report for details on the timing violations.
188*timing2"
timing summary2default:defaultZ38-282h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2+
report_timing_summary: 2default:default2
00:00:522default:default2
00:00:102default:default2
7912.5822default:default2
0.0002default:default2
40892default:default2
101302default:defaultZ17-722h px? 

%s4*runtcl2c
OExecuting : report_incremental_reuse -file rvfpga_incremental_reuse_routed.rpt
2default:defaulth px? 
g
BIncremental flow is disabled. No incremental reuse Info to report.423*	vivadotclZ4-1062h px? 

%s4*runtcl2c
OExecuting : report_clock_utilization -file rvfpga_clock_utilization_routed.rpt
2default:defaulth px? 
?
%s4*runtcl2?
?Executing : report_bus_skew -warn_on_violation -file rvfpga_bus_skew_routed.rpt -pb rvfpga_bus_skew_routed.pb -rpx rvfpga_bus_skew_routed.rpx
2default:defaulth px? 
r
UpdateTimingParams:%s.
91*timing29
% Speed grade: -1, Delay Type: min_max2default:defaultZ38-91h px? 
|
CMultithreading enabled for timing update using a maximum of %s CPUs155*timing2
62default:defaultZ38-191h px? 


End Record