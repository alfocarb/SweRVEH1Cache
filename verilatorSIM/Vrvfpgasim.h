// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Primary design header
//
// This header should be included by all source files instantiating the design.
// The class here is then constructed to instantiate the design.
// See the Verilator manual for examples.

#ifndef _Vrvfpgasim_H_
#define _Vrvfpgasim_H_

#include "verilated_heavy.h"
#include "Vrvfpgasim__Dpi.h"

class Vrvfpgasim__Syms;
class Vrvfpgasim_rvfpgasim;
class VerilatedVcd;

//----------

VL_MODULE(Vrvfpgasim) {
  public:
    // CELLS
    // Public to allow access to /*verilator_public*/ items;
    // otherwise the application code can consider these internals.
    Vrvfpgasim_rvfpgasim* rvfpgasim;
    
    // PORTS
    // The application code writes and reads these signals to
    // propagate new values into/out from the Verilated model.
    // Begin mtask footprint all: 
    VL_IN8(clk,0,0);
    VL_IN8(i_jtag_tck,0,0);
    VL_IN8(i_jtag_trst_n,0,0);
    VL_IN8(rst,0,0);
    VL_IN8(i_jtag_tms,0,0);
    VL_IN8(i_jtag_tdi,0,0);
    VL_OUT8(o_jtag_tdo,0,0);
    VL_OUT8(o_uart_tx,0,0);
    VL_OUT8(o_gpio,0,0);
    
    // LOCAL SIGNALS
    // Internals; generally not touched by application code
    
    // LOCAL VARIABLES
    // Internals; generally not touched by application code
    // Anonymous structures to workaround compiler member-count bugs
    struct {
        // Begin mtask footprint all: 
        VL_SIG8(__VinpClk__TOP__rvfpgasim____Vcellinp__swervolf__rstn,0,0);
        VL_SIG8(__VinpClk__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__core_rst_l,0,0);
        VL_SIG8(__VinpClk__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__dbg__DOT__dbg_dm_rst_l,0,0);
        VL_SIG8(__VinpClk__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__dbg__DOT____Vcellinp__dbg_state_reg__rst_l,0,0);
        VL_SIG8(__Vclklast__TOP____VinpClk__TOP__rvfpgasim____Vcellinp__swervolf__rstn,0,0);
        VL_SIG8(__Vclklast__TOP__clk,0,0);
        VL_SIG8(__Vclklast__TOP____VinpClk__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__core_rst_l,0,0);
        VL_SIG8(__Vclklast__TOP__i_jtag_tck,0,0);
        VL_SIG8(__Vclklast__TOP__i_jtag_trst_n,0,0);
        VL_SIG8(__Vclklast__TOP____VinpClk__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__dbg__DOT__dbg_dm_rst_l,0,0);
        VL_SIG8(__Vclklast__TOP____VinpClk__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__dbg__DOT____Vcellinp__dbg_state_reg__rst_l,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__timer_ptc__DOT__hrc_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__timer_ptc__DOT__lrc_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__timer_ptc__DOT__cntr_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_data_inst__DOT____Vcellinp__WAYS__BRA__0__KET____DOT__SUBBANKS__BRA__0__KET____DOT__ic_bank_sb_way_data__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_data_inst__DOT____Vcellinp__WAYS__BRA__1__KET____DOT__SUBBANKS__BRA__0__KET____DOT__ic_bank_sb_way_data__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_data_inst__DOT____Vcellinp__WAYS__BRA__2__KET____DOT__SUBBANKS__BRA__0__KET____DOT__ic_bank_sb_way_data__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_data_inst__DOT____Vcellinp__WAYS__BRA__3__KET____DOT__SUBBANKS__BRA__0__KET____DOT__ic_bank_sb_way_data__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__exu__DOT__mul_e1__DOT__exu_mul_c1_e2_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__exu__DOT__mul_e1__DOT__exu_mul_c1_e1_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__dbg__DOT__bus_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__exu__DOT__mul_e1__DOT__exu_mul_c1_e3_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT__fetch_f1_f2_c1_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_freeze_c2_dc4_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_freeze_c2_dc1_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_dccm_c1_dc3_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_freeze_c2_dc3_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT__axiclk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_free_c2_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_freeze_c1_dc2_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_freeze_c2_dc2_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf____PVT__swerv_eh1__DOT__swerv__DOT__lsu__DOT__lsu_freeze_c1_dc3_clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__0__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__1__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__2__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__3__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__4__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__5__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__6__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__Gen_dccm_enable__DOT__dccm__DOT____Vcellinp__mem_bank__BRA__7__KET____DOT__dccm_bank__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__0__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__1__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__2__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__3__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__4__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__5__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__6__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_WAY_STATUS__BRA__7__KET____DOT__WAY_STATUS__BRA__0__KET____DOT__ic_way_status__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__0__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__1__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__2__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__3__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__4__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__5__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__6__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__bp__DOT____Vcellinp__BANKS__BRA__7__KET____DOT__BHT_CLK_GROUP__BRA__0__KET____DOT__BHT_FLOPS__BRA__0__KET____DOT__bht_bank__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_tag_inst__DOT____Vcellinp__WAYS__BRA__0__KET____DOT__ICACHE_SZ_16__DOT__ic_way_tag__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_tag_inst__DOT____Vcellinp__WAYS__BRA__1__KET____DOT__ICACHE_SZ_16__DOT__ic_way_tag__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_tag_inst__DOT____Vcellinp__WAYS__BRA__2__KET____DOT__ICACHE_SZ_16__DOT__ic_way_tag__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__mem__DOT__icm__DOT__ic_tag_inst__DOT____Vcellinp__WAYS__BRA__3__KET____DOT__ICACHE_SZ_16__DOT__ic_way_tag__CLK,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__0__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way3_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__1__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way3_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__0__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way2_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__1__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way2_tagvalid_dup__clk,0,0);
    };
    struct {
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__0__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way1_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__1__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way1_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__0__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way0_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vclklast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__ifu__DOT__mem_ctl__DOT____Vcellinp__CLK_GRP_TAG_VALID__BRA__1__KET____DOT__TAG_VALID__BRA__0__KET____DOT__ic_way0_tagvalid_dup__clk,0,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim____Vcellinp__swervolf__rstn,0,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__timer_ptc__DOT__eclk_gate,0,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__core_rst_l,0,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__dbg__DOT__dbg_dm_rst_l,0,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__dbg__DOT____Vcellinp__dbg_state_reg__rst_l,0,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__1__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__1__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__2__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__2__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__3__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__3__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__4__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__0__KET____DOT__COMPARE__BRA__4__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__1__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__1__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__1__KET____DOT__COMPARE__BRA__1__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__1__KET____DOT__COMPARE__BRA__1__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__1__KET____DOT__COMPARE__BRA__2__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__1__KET____DOT__COMPARE__BRA__2__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__2__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__2__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__2__KET____DOT__COMPARE__BRA__1__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__2__KET____DOT__COMPARE__BRA__1__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__3__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_priority,3,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__swerv_eh1__DOT__swerv__DOT__pic_ctrl_inst__DOT____Vcellout__LEVEL__BRA__3__KET____DOT__COMPARE__BRA__0__KET____DOT__cmp_l1__out_id,7,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__data_nodes,20,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__data_nodes,20,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__data_nodes,20,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_b_mux__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__req_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__index_nodes,5,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__gnt_nodes,2,0);
        VL_SIG8(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__req_nodes,2,0);
    };
    struct {
        VL_SIG16(__Vchglast__TOP__rvfpgasim__swervolf__timer_ptc__DOT__rptc_ctrl,8,0);
        VL_SIGW(__Vm_traceActivity,159,0,5);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__mst_reqs_o,650,0,21);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__0__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__data_nodes,215,0,7);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__mst_reqs_o,650,0,21);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__1__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__data_nodes,215,0,7);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__mst_reqs_o,650,0,21);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_slv_port_demux__BRA__2__KET____DOT__i_axi_demux__gen_demux__DOT__i_r_mux__DOT__genblk2__DOT__data_nodes,215,0,7);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__data_nodes,221,0,7);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__0__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__data_nodes,203,0,7);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_aw_arbiter__DOT__genblk2__DOT__data_nodes,221,0,7);
        VL_SIGW(__Vchglast__TOP__rvfpgasim__swervolf__axi_intercon__DOT__axi_xbar__DOT__gen_mst_port_mux__BRA__1__KET____DOT__i_axi_mux__gen_mux__DOT__i_ar_arbiter__DOT__genblk2__DOT__data_nodes,203,0,7);
    };
    
    // INTERNAL VARIABLES
    // Internals; generally not touched by application code
    Vrvfpgasim__Syms* __VlSymsp;  // Symbol table
    
    // PARAMETERS
    // Parameters marked /*verilator public*/ for use by application code
    
    // CONSTRUCTORS
  private:
    VL_UNCOPYABLE(Vrvfpgasim);  ///< Copying not allowed
  public:
    /// Construct the model; called by application code
    /// The special name  may be used to make a wrapper with a
    /// single model invisible with respect to DPI scope names.
    Vrvfpgasim(const char* name="TOP");
    /// Destroy the model; called (often implicitly) by application code
    ~Vrvfpgasim();
    /// Trace signals in the model; called by application code
    void trace(VerilatedVcdC* tfp, int levels, int options=0);
    
    // API METHODS
    /// Evaluate the model.  Application must call when inputs change.
    void eval();
    /// Simulation complete, run final blocks.  Application must call on completion.
    void final();
    
    // INTERNAL METHODS
  private:
    static void _eval_initial_loop(Vrvfpgasim__Syms* __restrict vlSymsp);
  public:
    void __Vconfigure(Vrvfpgasim__Syms* symsp, bool first);
  private:
    static QData _change_request(Vrvfpgasim__Syms* __restrict vlSymsp);
    void _ctor_var_reset() VL_ATTR_COLD;
  public:
    static void _eval(Vrvfpgasim__Syms* __restrict vlSymsp);
  private:
#ifdef VL_DEBUG
    void _eval_debug_assertions();
#endif // VL_DEBUG
  public:
    static void _eval_initial(Vrvfpgasim__Syms* __restrict vlSymsp) VL_ATTR_COLD;
    static void _eval_settle(Vrvfpgasim__Syms* __restrict vlSymsp) VL_ATTR_COLD;
    static void _sequent__TOP__3(Vrvfpgasim__Syms* __restrict vlSymsp);
    static void _settle__TOP__1(Vrvfpgasim__Syms* __restrict vlSymsp) VL_ATTR_COLD;
    static void _settle__TOP__2(Vrvfpgasim__Syms* __restrict vlSymsp);
    static void traceChgThis(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__10(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__100(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__101(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__102(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__103(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__104(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__105(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__106(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__107(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__108(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__109(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__11(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__110(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__111(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__112(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__113(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__114(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__115(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__116(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__117(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__118(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__119(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__12(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__120(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__121(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__122(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__123(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__124(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__125(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__126(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__127(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__128(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__129(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__13(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__130(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__131(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__132(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__133(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__134(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__135(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__136(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__137(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__138(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__139(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__14(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__140(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__141(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__142(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__143(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__144(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__145(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__146(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__147(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__148(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__149(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__15(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__150(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__151(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__152(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__153(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__154(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__155(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__156(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__157(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__158(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__159(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__16(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__160(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__161(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__162(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__163(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__164(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__165(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__166(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__167(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__168(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__169(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__17(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__170(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__171(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__172(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__173(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__174(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__175(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__176(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__177(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__178(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__179(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__18(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__180(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__181(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__182(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__183(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__184(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__185(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__186(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__187(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__188(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__189(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__19(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__190(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__191(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__192(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__193(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__194(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__195(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__196(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__197(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__198(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__199(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__2(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__20(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__200(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__201(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__202(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__203(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__204(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__205(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__206(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__207(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__208(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__209(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__21(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__210(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__211(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__212(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__213(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__214(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__215(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__216(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__217(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__218(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__219(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__22(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__220(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__221(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__222(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__223(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__224(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__225(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__226(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__227(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__228(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__229(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__23(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__230(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__231(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__232(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__233(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__234(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__235(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__236(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__237(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__238(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__239(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__24(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__240(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__241(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__242(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__243(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__244(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__245(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__246(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__247(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__248(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__249(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__25(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__250(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__251(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__252(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__253(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__254(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__255(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__256(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__257(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__258(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__259(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__26(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__260(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__261(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__262(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__263(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__264(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__265(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__266(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__267(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__268(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__269(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__27(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__270(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__271(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__272(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__273(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__274(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__275(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__276(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__277(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__278(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__279(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__28(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__280(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__281(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__282(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__283(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__284(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__285(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__286(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__287(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__288(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__289(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__29(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__290(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__291(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__292(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__293(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__294(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__295(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__296(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__297(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__298(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__299(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__3(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__30(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__300(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__301(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__302(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__303(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__304(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__305(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__306(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__307(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__308(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__309(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__31(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__310(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__311(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__312(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__313(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__314(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__315(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__316(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__317(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__318(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__319(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__32(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__320(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__321(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__322(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__323(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__324(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__325(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__326(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__327(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__328(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__329(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__33(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__330(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__331(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__332(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__333(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__334(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__335(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__336(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__337(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__338(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__339(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__34(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__340(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__341(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__342(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__343(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__344(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__345(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__346(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__347(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__348(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__349(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__35(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__350(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__351(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__352(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__353(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__354(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__355(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__356(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__357(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__358(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__359(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__36(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__360(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__361(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__362(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__363(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__364(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__365(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__366(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__367(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__368(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__369(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__37(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__370(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__371(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__372(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__373(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__374(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__375(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__376(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__377(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__378(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__379(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__38(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__380(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__381(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__382(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__383(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__384(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__385(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__386(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__387(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__388(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__389(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__39(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__390(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__391(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__392(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__393(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__394(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__395(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__396(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__397(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__398(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__399(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__4(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__40(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__400(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__401(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__402(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__403(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__404(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__405(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__406(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__407(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__408(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__409(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__41(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__410(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__411(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__412(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__413(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__414(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__415(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__416(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__417(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__418(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__419(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__42(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__420(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__421(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__422(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__423(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__424(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__425(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__426(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__427(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__428(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__429(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__43(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__430(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__431(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__432(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__433(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__434(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__435(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__436(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__437(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__438(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__439(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__44(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__440(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__441(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__442(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__443(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__444(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__445(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__446(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__447(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__448(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__449(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__45(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__46(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__47(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__48(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__49(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__5(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__50(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__51(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__52(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__53(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__54(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__55(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__56(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__57(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__58(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__59(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__6(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__60(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__61(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__62(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__63(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__64(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__65(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__66(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__67(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__68(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__69(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__7(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__70(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__71(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__72(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__73(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__74(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__75(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__76(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__77(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__78(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__79(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__8(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__80(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__81(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__82(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__83(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__84(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__85(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__86(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__87(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__88(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__89(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__9(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__90(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__91(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__92(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__93(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__94(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__95(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__96(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__97(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__98(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceChgThis__99(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code);
    static void traceFullThis(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code) VL_ATTR_COLD;
    static void traceFullThis__1(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code) VL_ATTR_COLD;
    static void traceInitThis(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code) VL_ATTR_COLD;
    static void traceInitThis__1(Vrvfpgasim__Syms* __restrict vlSymsp, VerilatedVcd* vcdp, uint32_t code) VL_ATTR_COLD;
    static void traceInit(VerilatedVcd* vcdp, void* userthis, uint32_t code);
    static void traceFull(VerilatedVcd* vcdp, void* userthis, uint32_t code);
    static void traceChg(VerilatedVcd* vcdp, void* userthis, uint32_t code);
} VL_ATTR_ALIGNED(128);

#endif // guard
