// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//********************************************************************************
// $Id$
//
//
// Owner:
// Function: Checks the memory map for the address
// Comments:
//
//********************************************************************************

/*   Tag          C     Bank Offset     Bank      Byte offset
 * [31:12]    [11:6]     [5:4]              [3:2]       [1:0]  depth 256
 * 
 *  Tag          C     Bank Offset     Bank      Byte offset
 * [31:13]    [12:6]     [5:4]              [3:2]       [1:0]  depth 512
 */


module lsu_dc_ctl
   import swerv_types::*;
(
    input logic             clk,
    input logic             free_clk,
    input logic             rst_l,
    input logic             scan_mode,
    input logic             lsu_freeze_c2_dc3_clk,      
    input logic             lsu_freeze_c2_dc2_clk,
    input logic             lsu_free_c2_clk,
    input logic             flush_dc2_up,
     
    input logic [31:0]      cache_start_addr_dc1,
    input logic [31:0]      cache_end_addr_dc1,        //not used yet. we are assuming LW
    input logic [31:0]      cache_start_addr_dc2,              
    input logic [31:0]      cache_end_addr_dc2,          //not used yet. we are assuming LW    
    input logic [31:0]      cache_start_addr_dc3,              
    input logic [31:0]      cache_end_addr_dc3,          //not used yet. we are assuming LW
        
    input lsu_pkt_t         lsu_pkt_dc1, 
    input lsu_pkt_t         lsu_pkt_dc2,               
    input lsu_pkt_t         lsu_pkt_dc3,
    input logic             dec_tlu_non_blocking_disable,    // disable the non block
   
    // lsu bus buffer
    input logic             addr_external_dc1,
    input logic             addr_external_dc3,
    input logic             addr_was_external_dc2, 
    output logic            cacheable,
    output logic            rdc_save_nonblock_data,    
    output logic                                lsu_dc_nonblock_load_valid_dc3,     // there is an external load -> put in the cam
    output logic [`RV_LSU_NUM_NBLOAD_WIDTH-1:0] lsu_dc_nonblock_load_tag_dc3,       // the tag of the external non block load
    output logic                                lsu_dc_nonblock_load_inv_dc5,       // invalidate signal for the cam entry for non block loads   // NOT needed
    output logic [`RV_LSU_NUM_NBLOAD_WIDTH-1:0] lsu_dc_nonblock_load_inv_tag_dc5,   // tag of the enrty which needs to be invalidated            // NOT needed
    output logic                                lsu_dc_nonblock_load_data_valid,    // the non block is valid - sending information back to the cam
    output logic                                lsu_dc_nonblock_load_data_error,    // non block load has an error
    output logic [`RV_LSU_NUM_NBLOAD_WIDTH-1:0] lsu_dc_nonblock_load_data_tag,      // the tag of the non block load sending the data/error
    output logic [31:0]                         lsu_dc_nonblock_load_data,          // Data of the non block load
   
    
    // From store
    input logic [63:0]      store_data_dc3,
    input logic [63:0]      store_data_dc2,
   
    // To pipe
    output logic [31:0]     dc_out_data_dc3,   
    
    // To dc_mem - connection
    output logic            dc_mem_data,
    output logic [31:0]     lsu_dc_mem_data_dc3,    
       
    output logic [31:2]                 dc_rw_addr,  
    output logic [31:2]                 dc_rw_tag_addr,  
    output logic                        dc_rd_en,  
    output logic                        dc_wr_word, 
    output logic [`RV_DC_NUM_WAYS-1:0]      dc_wr_en,  
    output logic [`RV_DC_NUM_WAYS-1:0]      lsu_dc_tag_wren,
    output logic [67:0]                 dc_wr_data,         // Data to fill to the Dcache. With Parity
    input  logic [34*`RV_DC_NUM_SUBBANKS-1:0]                dc_rd_data,        // Data read from Dcache. 2x64bits + parity bits. F2 stage. With Parity
    input logic [`RV_DC_NUM_WAYS-1:0]       dc_rd_hit_in,
    output logic [`RV_DC_NUM_WAYS-1:0]      dc_rd_hit_ff, 
    `ifdef RV_DC_WB_POLICY_ENABLE
        input logic [`RV_DC_NUM_WAYS-1:0] [(32-`RV_DC_TAG_HIGH):0] dc_tag_data_raw,
    `endif  
    output logic [`RV_DC_NUM_WAYS-1:0]      dc_tag_valid,
    input logic                         dc_tag_perr_in,
 
    // Hit logic    
    output logic            dc_hit,          
    output logic            dc_hit_dc3, 
   
    // Freeze logic
    input logic             lsu_freeze_dc3,
    input logic             ld_freeze_rst,
    input logic             ld_freeze_en,   
    output logic            lsu_dc_freeze_dc3,
    output logic            lsu_dc_unfreeze_dc3,
    
      // AXI Write Channels
    output logic                            lsu_dc_axi_awvalid,
    input  logic                            lsu_dc_axi_awready,
    output logic [`RV_LSU_DC_BUS_TAG-1:0]       lsu_dc_axi_awid,
    output logic [31:0]                     lsu_dc_axi_awaddr,
    output logic [3:0]                      lsu_dc_axi_awregion,
    output logic [7:0]                      lsu_dc_axi_awlen,
    output logic [2:0]                      lsu_dc_axi_awsize,
    output logic [1:0]                      lsu_dc_axi_awburst,
    output logic                            lsu_dc_axi_awlock,
    output logic [3:0]                      lsu_dc_axi_awcache,
    output logic [2:0]                      lsu_dc_axi_awprot,
    output logic [3:0]                      lsu_dc_axi_awqos,
    
    output logic                            lsu_dc_axi_wvalid,
    input  logic                            lsu_dc_axi_wready,
    output logic [63:0]                     lsu_dc_axi_wdata,
    output logic [7:0]                      lsu_dc_axi_wstrb,
    output logic                            lsu_dc_axi_wlast,

    input  logic                            lsu_dc_axi_bvalid,
    output logic                            lsu_dc_axi_bready,
    input  logic [1:0]                      lsu_dc_axi_bresp,
    input  logic [`RV_LSU_DC_BUS_TAG-1:0]       lsu_dc_axi_bid,

   // AXI Read Channels
    output logic                            lsu_dc_axi_arvalid,
    input  logic                            lsu_dc_axi_arready,
    output logic [`RV_LSU_DC_BUS_TAG-1:0]       lsu_dc_axi_arid,
    output logic [31:0]                     lsu_dc_axi_araddr,
    output logic [3:0]                      lsu_dc_axi_arregion,
    output logic [7:0]                      lsu_dc_axi_arlen,
    output logic [2:0]                      lsu_dc_axi_arsize,
    output logic [1:0]                      lsu_dc_axi_arburst,
    output logic                            lsu_dc_axi_arlock,
    output logic [3:0]                      lsu_dc_axi_arcache,
    output logic [2:0]                      lsu_dc_axi_arprot,
    output logic [3:0]                      lsu_dc_axi_arqos,

    input  logic                            lsu_dc_axi_rvalid,
    output logic                            lsu_dc_axi_rready,
    input  logic [`RV_LSU_DC_BUS_TAG-1:0]       lsu_dc_axi_rid,
    input  logic [63:0]                     lsu_dc_axi_rdata,
    input  logic [1:0]                      lsu_dc_axi_rresp,
    input  logic                            lsu_dc_axi_rlast,

    input logic                             lsu_dc_bus_clk_en   // external drives a clock_en to control bus ratio   
);  
   `include "global.h"
     
   typedef enum logic [1:0] {RDC_IDLE=2'b00, FR_REQ=2'b01, FR_WAIT=2'b10} req_state_t;   // req sm
   logic req_state_en;
   req_state_t req_state, req_nxtstate;
`ifdef RV_DC_WB_POLICY_ENABLE
   typedef enum logic [2:0] {IDLE=3'b000, SND_ADDR=3'b001, SND_DATA=3'b010, W_RESP=3'b011, NO_RESP=3'b100, IS_DIRTY=3'b101} write_state_t;
`endif
   
   
   logic [31:0] dc_rw_addr_debug;
   assign dc_rw_addr_debug = {dc_rw_addr, 2'b0};
   logic [31:0] dc_rw_tag_addr_debug;
   assign dc_rw_tag_addr_debug = {dc_rw_tag_addr, 2'b0};
   
   logic hitable, hitable_dc2, hitable_dc3;
   logic dc_hitff;
   logic dc_hit_rdc_dc2;
   `ifdef RV_DC_WB_POLICY_ENABLE
        logic [DC_NUM_WAYS-1:0]     dc_tag_dirty;
        logic dc_rd_en_rdc, dc_rd_en_rdc_q, dc_rd_en_dc2; //TODO probablemente tenga un loop dc_rd_en_rdc
   `endif
   logic [DC_REQ_LOG2_BUF_DEPTH-1:0]  rdc_wr_ptr_q;
   logic is_nonblocking_load_dc3;
   logic is_nonblocking_load_dc2;
   logic [DC_NUM_WAYS-1:0] dc_rd_hit_q;
   logic [DC_NUM_WAYS-1:0] dc_rdc_hit_way, dc_rdc_hit_way_dc2;
   logic dc_use; //TODO probablemente tenga un loop
   logic dc_tag_use;
   logic lsu_dc_unfreeze_dc3ff;
   logic addr_was_external_dc3;
   logic [31:2] dc_rw_tag_addr_ff;
   logic [DC_NUM_WAYS-1:0] dc_tag_wait;
   
   logic [32*DC_NUM_SUBBANKS-1:0] dc_rd_data_only;
   
   logic [31:0] lsu_dc_mem_data;
      
   logic [63:0] dc_store_data_dc2_shifted;
   logic [63:0] dc_wr_data_noPar;
   
   logic [DC_NUM_WAYS-1:0] dc_st_way_we;

   logic axi_lsu_bus_clk_en;
   logic axiclk_dc;
   logic dc_hit_from_freeze;
   
   // eable
   assign hitable = lsu_pkt_dc1.valid & ~lsu_pkt_dc1.dma & (lsu_pkt_dc1.load | lsu_pkt_dc1.store)  & addr_external_dc1; //If could be a hit. (1 if we use the tag portion of the dc_mem)
   assign cacheable = lsu_pkt_dc3.valid & ~lsu_pkt_dc3.dma & (lsu_pkt_dc3.load | (lsu_pkt_dc3.store & dc_hit_dc3)) & addr_was_external_dc3;
   
   logic dc_tag_perr;
   logic [DC_NUM_WAYS-1:0] dc_rd_hit;
   assign dc_tag_perr = dc_tag_perr_in;
   assign dc_rd_hit = dc_rd_hit_in;
   
   //outs to dc_mem
`ifdef RV_DC_WB_POLICY_ENABLE
   assign dc_use =  ( ( | dc_st_way_we) | (dc_rd_en & ~dc_rd_en_rdc)  | lsu_dc_unfreeze_dc3 );
   assign dc_rd_en = dc_hit & lsu_pkt_dc2.load | dc_hit_from_freeze | dc_store_noWord_readDc | dc_rd_en_rdc; 
`else
    assign dc_rd_en = dc_hit & lsu_pkt_dc2.load |  dc_hit_from_freeze | dc_store_noWord_readDc;
    assign dc_use = ( ( | dc_st_way_we) | dc_rd_en | lsu_dc_unfreeze_dc3 );
`endif     
    assign dc_hit_from_freeze = lsu_pkt_dc3.load & lsu_pkt_dc3.valid & dc_hit_rdc_dc3 & lsu_freeze_dc3 & lsu_dc_unfreeze_dc3;
   assign dc_rw_addr[2] = lsu_dc_unfreeze_dc3 ? cache_start_addr_dc3[2] : cache_start_addr_dc2[2]; 
   assign dc_wr_data = {dc_wr_parity[3:2] , dc_wr_data_noPar[63:32], dc_wr_parity[1:0] , dc_wr_data_noPar[31:0]};
   
   assign dc_tag_use = (hitable & ~lsu_freeze_dc3) | lsu_dc_unfreeze_dc3;
   assign dc_wr_word = ( | dc_st_way_we);
   assign dc_st_way_we = dc_rd_hit & {DC_NUM_WAYS{lsu_pkt_dc2.store & lsu_pkt_dc2.valid & dc_hit}};
   assign dc_wr_en  = rdc_way_we | dc_st_way_we; 
   assign dc_hit = hitable_dc2 & (( | (dc_rd_hit & (dc_tag_valid | dc_tag_wait))) | dc_hit_rdc_dc2) & ~lsu_freeze_dc3; 
   
   assign lsu_dc_tag_wren = dc_wr_en & {DC_NUM_WAYS{dc_first_resp}};
    `ifdef RV_DC_WB_POLICY_ENABLE
        assign dc_rw_tag_addr [31:2] = dc_tag_use ? (lsu_dc_unfreeze_dc3 ? cache_start_addr_dc2[31:2] : cache_start_addr_dc1[31:2]) : 
                                                    (dc_rd_en_rdc ? {dc_start_addr_req[31:DC_TAG_LOW], lsu_dc_axi_arid[2:0], 1'b0} : {dc_start_addr_req[31:DC_TAG_LOW], lsu_dc_axi_rid[2:0], 1'b0});
        assign dc_rw_addr[31:3] = dc_use ? (lsu_dc_unfreeze_dc3 ? cache_start_addr_dc3[31:3] : cache_start_addr_dc2[31:3]) : 
                                            (dc_rd_en_rdc ? {dc_start_addr_req[31:DC_TAG_LOW], lsu_dc_axi_arid[2:0]} : {dc_start_addr_req[31:6], lsu_dc_axi_rid[2:0]});                      

    `else
        assign dc_rw_tag_addr [31:2] = dc_tag_use ? (lsu_dc_unfreeze_dc3 ? cache_start_addr_dc2[31:2] : cache_start_addr_dc1[31:2]): ({dc_start_addr_req[31:6], lsu_dc_axi_rid[2:0], 1'b0});
        assign dc_rw_addr[31:3] = dc_use ? (lsu_dc_unfreeze_dc3 ? cache_start_addr_dc3[31:3] : cache_start_addr_dc2[31:3]) : ({dc_start_addr_req[31:6], lsu_dc_axi_rid[2:0]});  
    `endif

   assign dc_mem_data = hitable_dc3 & ~(dc_hitff & ~lsu_freeze_dc3); 
   assign lsu_dc_mem_data = cache_start_addr_dc3[2] ? lsu_dc_axi_rdata[63:32] : lsu_dc_axi_rdata[31:0]; // cogemos segun alineamiento
   logic [63:0] store_data_dc2_noWord_shift;
   assign store_data_dc2_noWord_shift = (store_data_dc2 << dc_noWord_shift);
    assign dc_store_data_dc2_shifted = dc_store_bit_valid_dc2_shifted & store_data_dc2_noWord_shift; 
    assign dc_wr_data_noPar =  (lsu_pkt_dc2.store & lsu_pkt_dc2.valid & ~lsu_pkt_dc3.dma & ~lsu_freeze_dc3) ? (dc_store_noWord_dc2 ? dc_store_noWord_data_dc2_shifted : dc_store_data_dc2_shifted) : lsu_dc_axi_rdata;
    
    //No word store logic.
    logic [7:0] dc_store_byte_valid_dc2, dc_store_byte_valid_dc2_shifted;
    logic [63:0] dc_store_bit_valid_dc2, dc_store_bit_valid_dc2_shifted;
    logic [3+3:0] dc_noWord_shift;
    logic [63:0]dc_store_noWord_data_dc2_shifted;
    logic dc_store_noWord_dc1;
    logic dc_store_noWord_dc2;
    logic dc_store_noWord_readDc, dc_store_noWord_readDc_q;
    logic [63:0] dc_store_noWord_dataDc;

    assign dc_store_byte_valid_dc2 = lsu_pkt_dc2.word ? 8'h0F : (lsu_pkt_dc2.half ? 8'h03 : 8'h01 );
    assign dc_store_byte_valid_dc2_shifted = dc_store_byte_valid_dc2 << cache_start_addr_dc2[2:0];
    assign dc_store_bit_valid_dc2 = lsu_pkt_dc2.word ? 64'h00000000FFFFFFFF : (lsu_pkt_dc2.half ? 64'h000000000000FFFF : 64'h00000000000000FF );
    assign dc_noWord_shift = (cache_start_addr_dc2[2:0] << 3);
    assign dc_store_bit_valid_dc2_shifted = (dc_store_bit_valid_dc2 << dc_noWord_shift);
    
    assign dc_store_noWord_data_dc2_shifted = (dc_store_noWord_dataDc & ~dc_store_bit_valid_dc2_shifted) | dc_store_data_dc2_shifted;
    
    assign dc_store_noWord_dc1 = lsu_pkt_dc1.valid & lsu_pkt_dc1.store & ~lsu_pkt_dc1.dma & (lsu_pkt_dc1.half | lsu_pkt_dc1.by);
    assign dc_store_noWord_dc2 = lsu_pkt_dc2.valid & lsu_pkt_dc2.store & ~lsu_pkt_dc2.dma & (lsu_pkt_dc2.half | lsu_pkt_dc2.by);
    
    assign dc_store_noWord_readDc = dc_freeze_origin[6] & ~dc_unfreeze_origin[6];
    rvdff #(1) dc_store_noWord_readDcff (.*, .clk(free_clk), .din(dc_store_noWord_readDc), .dout(dc_store_noWord_readDc_q));
    rvdffsc #(64) dc_store_noWord_dataDcff (.*, .clear(1'b0), .clk(free_clk), 
                .en(dc_store_noWord_readDc_q | (( | dc_wr_en) & dc_rw_addr[31:3] == cache_start_addr_dc2[31:3] & rdc_isin_buff)), //we will take any write to the addr to mem
                .din( dc_store_noWord_readDc_q ? (cache_start_addr_dc2[3] ? dc_rd_data_only[DC_NUM_SUBBANKS*32-1:(DC_NUM_SUBBANKS*32)/2] : dc_rd_data_only[(DC_NUM_SUBBANKS*32)/2-1:0]) : dc_wr_data_noPar), 
                .dout(dc_store_noWord_dataDc)
    );   
    
    assign is_nonblocking_load_dc3 = lsu_pkt_dc3.valid & lsu_pkt_dc3.load & ~dc_hit_dc3 & ~dec_tlu_non_blocking_disable & ~lsu_freeze_dc3 & 1'b0;
    assign is_nonblocking_load_dc2 = lsu_pkt_dc2.valid & lsu_pkt_dc2.load & ~dc_hit & ~dec_tlu_non_blocking_disable & ~lsu_freeze_dc3 & 1'b0;     //non blocking loads not working
    assign rdc_save_nonblock_data = lsu_dc_nonblock_load_data_valid;
    assign lsu_dc_nonblock_load_valid_dc3 = is_nonblocking_load_dc3;
    assign lsu_dc_nonblock_load_tag_dc3 = {`RV_LSU_NUM_NBLOAD_WIDTH{rdc_wr_ptr_q}};
    
    // parity generation
    logic [4:0] dc_wr_parity;
    genvar z ;
    for (z=0 ; z < 4 ; z++) begin : DC_DATA_PGEN
           rveven_paritygen #(16) pardc  (.data_in   (dc_wr_data_noPar[((16*z)+15):(16*z)]),
                                          .parity_out(dc_wr_parity[z]));
    end

    // out to pipe
    logic [4+3:0]dc_shift_dc3;
    assign dc_shift_dc3 = (cache_start_addr_dc3[3:0] << 3);
    assign dc_out_data_dc3[31:0] = dc_rd_data_only >> dc_shift_dc3;
    assign dc_rd_data_only = {dc_rd_data [133:102], dc_rd_data [99:68], dc_rd_data [65:34], dc_rd_data [31:0]} ;
    
    // ffs    
    logic dc_hit_rdc_dc3;
    logic [DC_NUM_WAYS-1:0] rdc_way_ff_ff;
    rvdff #(DC_REQ_LOG2_BUF_DEPTH) rdc_wr_ptr_ff (.*, .clk(lsu_freeze_c2_dc3_clk), .din(rdc_wr_ptr), .dout(rdc_wr_ptr_q)); 
    rvdff #(DC_NUM_WAYS) rdc_hit_way_dc23_ff (.*, .clk(lsu_freeze_c2_dc2_clk), .din(dc_rdc_hit_way), .dout(dc_rdc_hit_way_dc2));
    rvdff #(1) dc_hit_rdc_dc23_ff (.*, .clk(free_clk), .din(dc_hit_rdc_dc2), .dout(dc_hit_rdc_dc3));
    rvdffsc #(DC_NUM_WAYS) dc_way2_tagvalid_dup (.*, .clear(1'b0), .clk(lsu_free_c2_clk), .en(~lsu_freeze_dc3), .din(dc_hit_rdc_dc2 ? dc_rdc_hit_way_dc2 : dc_rd_hit), .dout(dc_rd_hit_q));
    
    logic ld_freeze_en_q, ld_freeze_rst_q;
    logic dc_tag_perr_freeze_q;
    logic [DC_NUM_WAYS-1:0] dc_rd_hit_freeze_q;
    rvdff #(1) ld_freeze_enff (.*, .clk(free_clk), .din(ld_freeze_en), .dout(ld_freeze_en_q));
    rvdff #(1) ld_freeze_rstff (.*, .clk(free_clk), .din(ld_freeze_rst), .dout(ld_freeze_rst_q));
    rvdffsc #(DC_NUM_WAYS) dc_rd_hit_inff (.*, .clear(1'b0), .clk(free_clk), .en(ld_freeze_en_q), .din(dc_rd_hit_in), .dout(dc_rd_hit_freeze_q));
    rvdffsc #(1) dc_tag_perr_inff (.*, .clear(1'b0), .clk(free_clk), .en(ld_freeze_en_q), .din(dc_tag_perr_in), .dout(dc_tag_perr_freeze_q));


    `ifdef RV_DC_WB_POLICY_ENABLE
        assign dc_rd_hit_ff = dc_rd_en_rdc_q ? rdc_way_ff : dc_rd_hit_q;    
    `else
        assign dc_rd_hit_ff = dc_rd_hit_q;    
    `endif    
    
    rvdff #(1) was_externalff (.*, .clk(lsu_freeze_c2_dc3_clk), .din(addr_was_external_dc2), .dout(addr_was_external_dc3));
    rvdff #(1) hitable_ff (.*, .clk(lsu_freeze_c2_dc2_clk), .din(hitable), .dout(hitable_dc2));
    rvdff #(1) hitable_dc23_ff (.*, .clk(lsu_freeze_c2_dc3_clk), .din(hitable_dc2), .dout(hitable_dc3));
    rvdffsc #(.WIDTH(32)) dc_bus_dataff (.din(lsu_dc_mem_data[31:0]), .dout(lsu_dc_mem_data_dc3[31:0]), .clk(lsu_free_c2_clk), .clear(1'b0), 
        .en((dc_id_start_req[2:0] == lsu_dc_axi_rid[2:0]) & (lsu_dc_data_recv) ) , .*
    ); 
    rvdff #(1) dc_hit_rdc_ff (.*, .clk(lsu_freeze_c2_dc2_clk), .din(rdc_isin_buff), .dout(dc_hit_rdc_dc2));
    rvdff #(1) dc_hit_ff (.*, .clk(lsu_freeze_c2_dc3_clk), .din(dc_hit), .dout(dc_hitff));
    assign dc_hit_dc3 = (dc_hitff | dc_hit_rdc_dc3 ) & ~lsu_freeze_dc3 & lsu_pkt_dc3.valid;
    rvdffe #(.WIDTH(30)) dc_rw_addrff (.din(dc_rw_tag_addr[31:2]), .dout(dc_rw_tag_addr_ff[31:2]), .en(dc_tag_use), .clk(lsu_free_c2_clk), .*); 
    
    //count hits
    logic [31:0] count_hit_debug, count_hit_debug_plus;
    assign count_hit_debug_plus = count_hit_debug + 1;
    rvdffe #(.WIDTH(32)) count_hit_debugff (.din(count_hit_debug_plus), .dout(count_hit_debug), .en(dc_hit_dc3), .clk(free_clk), .*); 
    
    //clock generation
   assign axi_lsu_bus_clk_en =  lsu_dc_bus_clk_en ;
   
   rvclkhdr axi_clk(.en(axi_lsu_bus_clk_en), .l1clk(axiclk_dc), .*);

   // freeze/unfreeze logic
   // freeze origins:
   //   6: The store in dc2 is half or byte and is in cache. 
   //   5: The load addres in dc1 is in the store buffer
   //   4: The load/store addres in dc1, is in the request buffer
   //   3: The load/store has a hit, but the block of cache is being load in dc
   //   2: The load/store in dc2 has no hit, we have to go to mem to get data
   //   1: Req buff is full
   //   0: Write buff is full
   localparam FREEZE_SIZE_ORIGIN = 7;
   logic [FREEZE_SIZE_ORIGIN-1:0] dc_freeze_origin;   
   logic [FREEZE_SIZE_ORIGIN-1:0] dc_unfreeze_origin;  
   logic [FREEZE_SIZE_ORIGIN-1:0] dc_oneUnfreeze_origin; 
   logic dc_oneUnfreeze;
   assign dc_oneUnfreeze = (lsu_freeze_dc3 & 
                            ((dc_freeze_origin[6] & dc_store_noWord_readDc) |
                             (dc_freeze_origin[5] & ~wdc_isin_buff) |
                             (dc_freeze_origin[4] & ~rdc_isin_buff) | 
                             (dc_freeze_origin[3] & dc_last_resp)   | 
                             (dc_freeze_origin[2] & dc_first_resp& (cache_start_addr_dc3[31:DC_TAG_LOW] == lsu_dc_axi_araddr[31:DC_TAG_LOW]))  | 
                             (dc_freeze_origin[1] & (~rdc_full))    | 
                             (dc_freeze_origin[0] & (~wdc_full)))    );
                              
   assign dc_oneUnfreeze_origin = {(dc_freeze_origin[6] & dc_store_noWord_readDc),
                                                       (dc_freeze_origin[5] & ~wdc_isin_buff),
                                                       (dc_freeze_origin[4] & ~rdc_isin_buff), 
                                                       (dc_freeze_origin[3] & dc_last_resp)  ,
                                                       (dc_freeze_origin[2] & dc_first_resp & (cache_start_addr_dc3[31:DC_TAG_LOW] == lsu_dc_axi_araddr[31:DC_TAG_LOW])), 
                                                       (dc_freeze_origin[1] & (~rdc_full))   , 
                                                       (dc_freeze_origin[0] & (~wdc_full))  };
   
    rvdffsc #(.WIDTH(FREEZE_SIZE_ORIGIN)) dc_freeze_originff (.din({dc_store_noWord_dc1, 
                                                                    wdc_isin_buff,
                                                                    rdc_isin_buff,
                                                                    dc_hit & ( | (dc_rd_hit & dc_tag_wait)),
                                                                    (hitable_dc2 & lsu_pkt_dc2.load & ~is_nonblocking_load_dc2 & ~dc_hit),
                                                                    rdc_full, 
                                                                    wdc_full     }),
        .dout(dc_freeze_origin[FREEZE_SIZE_ORIGIN-1:0]), .en(lsu_dc_freeze_dc3), .clk(free_clk), .clear(ld_freeze_rst) , .*
    ); 
    rvdffsc #(.WIDTH(FREEZE_SIZE_ORIGIN)) dc_unfreeze_originff (.din(lsu_dc_freeze_dc3 ? {FREEZE_SIZE_ORIGIN{1'b0}} : (dc_unfreeze_origin | dc_oneUnfreeze_origin)),
        .dout(dc_unfreeze_origin[FREEZE_SIZE_ORIGIN-1:0]), .en(lsu_dc_freeze_dc3 | dc_oneUnfreeze), .clk(free_clk), .clear(lsu_dc_unfreeze_dc3) , .*
    ); 
   assign lsu_dc_freeze_dc3 = ( dc_store_noWord_dc1 |
                                wdc_isin_buff |
                                rdc_isin_buff |
                                (dc_hit & ( | (dc_rd_hit & dc_tag_wait))) |
                                (hitable_dc2 & lsu_pkt_dc2.load &  ~is_nonblocking_load_dc2 &  ~dc_hit) |
                                rdc_full |
                                wdc_full    ) 
                            & ~lsu_freeze_dc3 & ~flush_dc2_up; //is generated in dc2 because afects the freeze flag ld_freeze_en 
                            
   assign lsu_dc_unfreeze_dc3 = lsu_freeze_dc3 & ((dc_freeze_origin == dc_unfreeze_origin));
   
   ///////////////////////////////////////////////////// 
   /////////////////// TAG valid /////////////////////// 
   /////////////////////////////////////////////////////
   
   //TAG valid    
   logic [DC_NUM_WAYS-1:0] lsu_dc_tag_clear;      
    
   logic reset_all_tags; 
   logic [DC_NUM_WAYS-1:0] lsu_dc_tag_valid_wren;
   
   logic [DC_NUM_WAYS-1:0] dc_tag_valid_rdc;
   
   assign reset_all_tags = 1'b0;                        
    
   assign lsu_dc_tag_clear = 4'b0;
   assign lsu_dc_tag_valid_wren = dc_wr_en & {DC_NUM_WAYS{dc_first_resp}}; 
   
    genvar tv;
    for (tv=0 ; tv< DC_NUM_WAYS ; tv++) begin : READ_TAG_VALID
        assign dc_tag_valid[tv] = dc_tag_valid_out[tv][dc_rw_tag_addr_ff[DC_TAG_HIGH-1:DC_TAG_LOW]];
        assign dc_tag_valid_rdc[tv] = dc_tag_valid_out[tv][reqAddr_in[DC_TAG_HIGH-1:DC_TAG_LOW]];
    end
        
   logic [DC_NUM_WAYS-1:0] [DC_TAG_DEPTH-1:0] dc_tag_valid_out ;
    
   logic [(DC_TAG_DEPTH/32)-1:0][DC_NUM_WAYS-1:0]  tag_valid_w_clken ;
   logic [(DC_TAG_DEPTH/32)-1:0][DC_NUM_WAYS-1:0]  tag_valid_w_clk   ;
        
    genvar  i;
    for (i=0 ; i<DC_TAG_DEPTH/32 ; i++) begin : CLK_GRP_TAG_VALID
       genvar  c;
       for (c=0 ; c<DC_NUM_WAYS ; c++) begin : CLK_GRP_TAG_VALID_GEN
          assign tag_valid_w_clken[i][c] = (((dc_start_addr_req[DC_TAG_HIGH-1:DC_TAG_LOW+5] == i ) &  (lsu_dc_tag_valid_wren[c] | lsu_dc_tag_clear[c])  ));
          rvclkhdr dirty_way_status_cgc ( .en(tag_valid_w_clken[i][c]),   .l1clk(tag_valid_w_clk[i][c]), .* );  
       end
    
       genvar  j;
       for (j=0 ; j<32 ; j++) begin : TAG_VALID
           genvar  k;
           for (k=0 ; k<DC_NUM_WAYS ; k++) begin : TAG_VALID_WAYS
               rvdffsc #(1) dc_way_tagvalid_dup (.*,
                       .clear(lsu_dc_tag_clear[k] & (dc_start_addr_req[DC_TAG_LOW+4:DC_TAG_LOW] == j) | reset_all_tags),  
                       .clk(tag_valid_w_clk[i][k]),
                       .en(((dc_start_addr_req[DC_TAG_LOW+4:DC_TAG_LOW] == j) & (dc_start_addr_req[DC_TAG_HIGH-1:DC_TAG_LOW+5] == i ) & lsu_dc_tag_valid_wren[k]) | reset_all_tags), 
                       .din(1'b1),
                       .dout(dc_tag_valid_out[k][32*i+j]));
           end
                       
        end //TAG_VALID
    end  // CLK_GRP_TAG_VALID
    
   ///////////// end TAG valid
   
   
   ///////////////////////////////////////////////////// 
   //////////////////// TAG wait ///////////////////////
   ///////////////////////////////////////////////////// 
   
   //TAG wait   
   logic [DC_NUM_WAYS-1:0] lsu_dc_wait_wren;  
   logic [DC_NUM_WAYS-1:0] lsu_dc_wait_clean;    
    
   logic reset_all_wait; 
      
   assign reset_all_wait = reset_all_tags;
    
   assign lsu_dc_wait_clean = dc_wr_en & {DC_NUM_WAYS{dc_last_resp}};
   assign lsu_dc_wait_wren =  dc_wr_en & {DC_NUM_WAYS{dc_first_resp}};

    genvar tw;
    for (tw=0 ; tw< DC_NUM_WAYS ; tw++) begin : READ_TAG_WAIT
        assign dc_tag_wait[tw] = dc_tag_wait_out[tw][cache_start_addr_dc2[DC_TAG_HIGH-1:DC_TAG_LOW]]; //en funcion de la dir de dc2, para ver si hay hit
    end
        
   logic [DC_NUM_WAYS-1:0] [DC_TAG_DEPTH-1:0] dc_tag_wait_out ;
      
   logic [(DC_TAG_DEPTH/32)-1:0][DC_NUM_WAYS-1:0]  tag_wait_w_clken ;
   logic [(DC_TAG_DEPTH/32)-1:0][DC_NUM_WAYS-1:0]  tag_wait_w_clk   ;
   
    genvar  tw_i;
    for (tw_i=0 ; tw_i<DC_TAG_DEPTH/32 ; tw_i++) begin : CLK_GRP_TAG_WAIT
       genvar  tw_c;
       for (tw_c=0 ; tw_c<DC_NUM_WAYS ; tw_c++) begin : CLK_GRP_TAG_WAIT_GEN
          assign tag_wait_w_clken[tw_i][tw_c] = (((dc_start_addr_req[DC_TAG_HIGH-1:DC_TAG_LOW+5] == tw_i ) &  (lsu_dc_wait_wren[tw_c] | lsu_dc_wait_clean[tw_c])  ));
          rvclkhdr dirty_way_status_cgc ( .en(tag_wait_w_clken[tw_i][tw_c]),   .l1clk(tag_wait_w_clk[tw_i][tw_c]), .* );  
       end

       genvar  tw_j;
       for (tw_j=0 ; tw_j<32 ; tw_j++) begin : TAG_WAIT
           genvar tw_k;
           for (tw_k=0 ; tw_k<DC_NUM_WAYS ; tw_k++) begin : TAG_WAIT_WAYS
               rvdffsc #(1) dc_way_tagwait_dup (.*,
                       .clear(lsu_dc_wait_clean[tw_k] & (dc_start_addr_req[DC_TAG_LOW+4:DC_TAG_LOW] == tw_j) | reset_all_wait), 
                       .clk(tag_wait_w_clk[tw_i][tw_k]),
                       .en(((dc_start_addr_req[DC_TAG_LOW+4:DC_TAG_LOW] == tw_j) & (dc_start_addr_req[DC_TAG_HIGH-1:DC_TAG_LOW+5] == tw_i ) & lsu_dc_wait_wren[tw_k])),
                       .din(1'b1),
                       .dout(dc_tag_wait_out[tw_k][32*tw_i+tw_j]));
           end
                       
        end //TAG_WAIT
    end  // CLK_GRP_TAG_WAIT
     
   ///////////// end TAG wait
    
`ifdef RV_DC_WB_POLICY_ENABLE

   ///////////////////////////////////////////////////// 
   //////////////////// TAG dirty //////////////////////
   ///////////////////////////////////////////////////// 
   
   //TAG dirty
   //reused signals are commented
   logic [DC_TAG_HIGH-1:0] lsu_dc_rw_int_addr;
   logic [DC_TAG_HIGH-1:0] rdc_dirty_rd_addr; 
   
   //tag dirty connection
   assign lsu_dc_rw_int_addr [DC_TAG_HIGH-1:0] = {dc_rw_addr[DC_TAG_HIGH-1:2], 2'b0}; 
   assign rdc_dirty_rd_addr [DC_TAG_HIGH-1:0] = wdc_wr_addr_in[DC_TAG_HIGH-1:0];  
   
   logic [DC_NUM_WAYS-1:0] lsu_dc_dirty_wren;  
   logic [DC_NUM_WAYS-1:0] lsu_dc_dirty_clean;    
    
   logic reset_all_dirty; 
   
   assign reset_all_dirty = reset_all_tags;                        
    
   assign lsu_dc_dirty_clean = lsu_dc_tag_wren;
   assign lsu_dc_dirty_wren = dc_st_way_we;

    genvar td;
    for (td=0 ; td< DC_NUM_WAYS ; td++) begin : READ_TAG_DIRTY
        assign dc_tag_dirty[td] = dc_tag_dirty_out[td][rdc_dirty_rd_addr[DC_TAG_HIGH-1:DC_TAG_LOW]][rdc_dirty_rd_addr[DC_BANKOFF_HIGH-1:NUM_LOG2_PACKETS_BLOCK]]; 
    end
        
   logic [DC_NUM_WAYS-1:0] [DC_TAG_DEPTH-1:0][NUM_PACKETS_BLOCK-1:0] dc_tag_dirty_out ;
    
   logic [(DC_TAG_DEPTH/32)-1:0][DC_NUM_WAYS-1:0]  tag_dirty_w_clken ;
   logic [(DC_TAG_DEPTH/32)-1:0][DC_NUM_WAYS-1:0]  tag_dirty_w_clk   ;      
   
   localparam NUM_PACKETS_BLOCK = DC_BLOCK_SIZE/2;
   localparam NUM_LOG2_PACKETS_BLOCK = $clog2(NUM_PACKETS_BLOCK);;

    genvar  td_i;
    for (td_i=0 ; td_i<DC_TAG_DEPTH/32 ; td_i++) begin : CLK_GRP_TAG_DIRTY
          genvar  td_c;
          for (td_c=0 ; td_c<DC_NUM_WAYS ; td_c++) begin : CLK_GRP_TAG_DIRTY_GEN
              assign tag_dirty_w_clken[td_i][td_c] = (((lsu_dc_rw_int_addr[DC_TAG_HIGH-1:DC_TAG_LOW+5] == td_i ) &  (lsu_dc_dirty_wren[td_c] | lsu_dc_dirty_clean[td_c])  ));
              rvclkhdr dirty_way_status_cgc ( .en(tag_dirty_w_clken[td_i][td_c]),   .l1clk(tag_dirty_w_clk[td_i][td_c]), .* );  
          end
    
       genvar  td_j;
       for (td_j=0 ; td_j<32 ; td_j++) begin : TAG_DIRTY //Usamos la dir que va directa a la cache (y no la del tag) ya que queremos usar la del store
            genvar  td_p;
            for (td_p=0 ; td_p<NUM_PACKETS_BLOCK ; td_p++) begin : TAG_PACKET
                genvar  td_k;
               for (td_k=0 ; td_k<DC_NUM_WAYS ;td_k++) begin : TAG_DIRTY_WAYS
                   rvdffsc #(1) dc_way_tagdirty_dup (.*,
                           .clear((lsu_dc_dirty_clean[td_k] & (dc_start_addr_req[DC_TAG_LOW+4:DC_TAG_LOW] == td_j) 
                                   & (lsu_dc_rw_int_addr[DC_TAG_HIGH-1:DC_TAG_LOW+5] == td_i )
                                   & (lsu_dc_rw_int_addr[DC_BANKOFF_HIGH-1:NUM_LOG2_PACKETS_BLOCK] == td_p)) | reset_all_dirty
                                  ), 
                           .clk(tag_dirty_w_clk[td_i][td_k]),
                           .en((((lsu_dc_dirty_clean[td_k] ? dc_start_addr_req[DC_TAG_LOW+4:DC_TAG_LOW] : lsu_dc_rw_int_addr[DC_TAG_LOW+4:DC_TAG_LOW] ) == td_j) 
                                   & ((lsu_dc_dirty_clean[td_k] ? dc_start_addr_req[DC_TAG_HIGH-1:DC_TAG_LOW+5] : lsu_dc_rw_int_addr[DC_TAG_HIGH-1:DC_TAG_LOW+5] ) == td_i) 
                                   & (lsu_dc_rw_int_addr[DC_BANKOFF_HIGH-1:NUM_LOG2_PACKETS_BLOCK] == td_p) & lsu_dc_dirty_wren[td_k]
                                  )), //| ((perr_ic_index_ff     [DC_TAG_LOW+4:DC_TAG_LOW] == td_j) & perr_err_inv_way[1])),
                           .din(1'b1),// & ~perr_sel_invalidate),
                           .dout(dc_tag_dirty_out[td_k][32*td_i+td_j][td_p]));
               end    
             end // TAG_PACKET                       
        end //TAG_DIRTY
    end  // CLK_GRP_TAG_DIRTY
     
   ///////////// end TAG dirty
`endif

   
   ///////////////////////////////////////////////////// 
   ///////////// Replace algorithm (FIFO) //////////////
   /////////////////////////////////////////////////////    
   //inputs
   logic [DC_TAG_HIGH-1: DC_TAG_LOW] radc_nblock_wr;
   logic radc_en;  // when we are going to write a new blok in cache
   
   //outputs
   logic [DC_NUM_WAYS-1:0] radc_way;
   
   // Replace algorithm connection
   
   assign radc_en = rdc_newReq; 
   assign radc_nblock_wr = cache_start_addr_dc2[DC_TAG_HIGH-1: DC_TAG_LOW];             // Numero de bloque sobre el que queremos escribir

`ifdef RV_DC_DIRECT_EMPLACEMENT
    assign radc_way = 1'b1; 
`else   
   localparam DC_LOG2_NUM_WAYS = $clog2(DC_NUM_WAYS); 
   //signals 
   logic [DC_LOG2_NUM_WAYS-1:0] radc_way_towr;
   logic [DC_LOG2_NUM_WAYS-1:0] radc_way_towr_plus;
   logic [DC_TAG_DEPTH-1:0] [DC_LOG2_NUM_WAYS-1:0] radc_way_count_out;
   logic [(DC_TAG_DEPTH/32)-1:0]  way_clken ;
   logic [(DC_TAG_DEPTH/32)-1:0]  way_clk   ;
   
       genvar  radc_i;
    for (radc_i=0 ; radc_i<DC_TAG_DEPTH/32 ; radc_i++) begin : CLK_GRP_WAY_COUNT
        assign way_clken[radc_i] = (cache_start_addr_dc2[DC_TAG_HIGH-1:DC_TAG_LOW+5] == radc_i);
        
        rvclkhdr way0_status_cgc ( .en(way_clken[radc_i]),   .l1clk(way_clk[radc_i]), .* );
          
       genvar  radc_j;
        for (radc_j=0 ; radc_j<32 ; radc_j++) begin : WAY_COUNT
             rvdffs #(DC_LOG2_NUM_WAYS) dc_way_count_dup (.*,
                       .clk(way_clk[radc_i]),
                       .en(radc_en & (cache_start_addr_dc2[DC_TAG_LOW+4:DC_TAG_LOW] == radc_j) & (cache_start_addr_dc2[DC_TAG_HIGH-1:DC_TAG_LOW+5] == radc_i)),
                       .din(radc_way_towr_plus), 
                       .dout(radc_way_count_out[32*radc_i+radc_j]));
                       
        end //WAY_COUNT
    end  // CLK_GRP_WAY_COUNT
    
    assign radc_way_towr = radc_way_count_out[radc_nblock_wr];
    assign radc_way_towr_plus = radc_way_towr + 1;
    
    assign radc_way = {{DC_NUM_WAYS-1{1'b0}}, 1'b1} << radc_way_towr; 
`endif 
   ///////////// end Request buffer/no_buffer
   
   
   ///////////////////////////////////////////////////// 
   ///////////// Request buffer/no_buffer //////////////
   ///////////////////////////////////////////////////// 
   //interface
        //in
   logic [31:0] reqAddr_in;
   logic [31:0] rdc_addr_in_buff;
   logic rdc_newReq;     
   logic [DC_NUM_WAYS-1:0] rdc_way_in; 
        //out
   logic rdc_isin_buff;
   logic rdc_full;
   logic [DC_NUM_WAYS-1:0] rdc_way_we;
   
       //Request Buff/no_buff conection
   `ifdef RV_DC_WRITE_ALLOCATE_ENABLE
      assign rdc_newReq = (~dc_hit & lsu_pkt_dc2.valid & (lsu_pkt_dc2.load | lsu_pkt_dc2.store) & ~lsu_freeze_dc3) & hitable_dc2; 
   `else
      assign rdc_newReq = (~dc_hit & lsu_pkt_dc2.valid & lsu_pkt_dc2.load & ~lsu_freeze_dc3) & hitable_dc2;
   `endif
   assign reqAddr_in = cache_start_addr_dc2; 
   assign rdc_way_in = radc_way;
   assign rdc_addr_in_buff = lsu_freeze_dc3 ? cache_start_addr_dc2 : cache_start_addr_dc1;
   //conexion buffer (si separamos en otro archivo)
   
   //logics
   logic rdc_write_data_recv;
   logic lsu_axi_dc_req_ff_in; 
   logic lsu_axi_dc_req_ff2;
   logic [31:0] dc_start_addr_req;
   logic lsu_dc_data_recv;
   logic dc_last_req;
   logic dc_first_req;
   logic dc_last_resp;
   logic dc_first_resp;
   logic [DC_NUM_WAYS-1:0] rdc_way; 
   logic rdc_newElm_req;
   logic [DC_REQ_BUF_DEPTH-1:0] rdc_addr_in_buff_hit;
   logic [DC_REQ_BUF_DEPTH-1:0] [DC_NUM_WAYS-1:0] rdc_way_in_buff_hit;

   logic rdc_empty, rdc_empty_ff;
   logic [DC_REQ_LOG2_BUF_DEPTH-1:0] rdc_rd_ptr, rdc_rd_ptr_plus;
   logic [DC_REQ_LOG2_BUF_DEPTH-1:0] rdc_wr_ptr, rdc_wr_ptr_plus;
   
   logic [DC_REQ_BUF_DEPTH-1:0] rdc_newReq_en;
   logic [DC_REQ_BUF_DEPTH-1:0] dc_last_req_clear;
   logic [DC_REQ_BUF_DEPTH-1:0] dc_last_resp_clear;
   
   logic [DC_REQ_BUF_DEPTH-1:0] lsu_axi_dc_req_ff2_q;
   logic [DC_REQ_BUF_DEPTH-1:0][31:0] dc_start_addr_req_q;
   logic [DC_REQ_BUF_DEPTH-1:0][DC_NUM_WAYS-1:0] rdc_way_q;
   logic [DC_REQ_BUF_DEPTH-1:0] rdc_valid_q;
    logic [DC_NUM_WAYS-1:0] rdc_way_ff;

   
   
        //counter
   logic [LSU_DC_BUS_TAG-1:0] axi_dc_rd_count, axi_dc_rd_count_plus;
   logic [2:0] dc_id_fin_req, dc_id_start_req;      
      
    //basic assign logic 
   assign rdc_way_we = rdc_way & {DC_NUM_WAYS{rdc_write_data_recv}};
   assign rdc_write_data_recv = lsu_dc_data_recv & ~(req_state == RDC_IDLE);
   assign rdc_full = ( & rdc_valid_q) | (rdc_newReq & ( (rdc_wr_ptr + 1) == rdc_rd_ptr));
   assign lsu_axi_dc_req_ff_in  = 1'b1;
   assign lsu_dc_data_recv = lsu_dc_axi_rvalid & lsu_dc_axi_rready;
   assign dc_last_req = (lsu_dc_axi_arvalid & lsu_dc_axi_arready & (dc_id_fin_req[2:0] == axi_dc_rd_count[2:0]));
   assign dc_first_req = (lsu_dc_axi_arvalid & lsu_dc_axi_arready & (dc_id_start_req[2:0] == axi_dc_rd_count[2:0]));
   assign dc_last_resp = (lsu_dc_axi_rvalid & lsu_dc_axi_rready & (dc_id_fin_req[2:0] == lsu_dc_axi_rid[2:0]));
   assign dc_first_resp = (lsu_dc_axi_rvalid & lsu_dc_axi_rready & (dc_id_start_req[2:0] == lsu_dc_axi_rid[2:0]));
   assign rdc_empty = rdc_empty_ff & ~rdc_newReq;
     
    logic [31:0] dc_rdaddr_rdc_q;
`ifdef RV_DC_WB_POLICY_ENABLE
    logic [DC_NUM_WAYS-1:0] rdc_replace;
    logic [DC_REQ_BUF_DEPTH-1:0][DC_NUM_WAYS-1:0] rdc_replace_q;
    logic [DC_NUM_WAYS-1:0][DC_TAG_LEN-1:0] dc_tag_data_raw_sel;
    logic [DC_TAG_LEN-1:0] rdc_waddr;
    rvdff #(32) dc_rdaddr_rdcff (.*, .clk(free_clk), .din({rdc_waddr[DC_TAG_LEN-1:0], lsu_dc_axi_araddr[DC_TAG_HIGH-1: DC_TAG_LOW], axi_dc_rd_count[2:0], 3'b0} ), .dout(dc_rdaddr_rdc_q)); 
       
    for (genvar rdc_wb=0; rdc_wb<DC_NUM_WAYS; rdc_wb++) begin: wb_tag_sel 
        assign dc_tag_data_raw_sel[rdc_wb] = dc_tag_data_raw[rdc_wb][DC_TAG_LEN-1:0] & {DC_TAG_LEN{rdc_way[rdc_wb]}};
    end
    always_comb begin
      rdc_waddr = '0;
      for (int rdc_or_wb=0; rdc_or_wb<DC_NUM_WAYS; rdc_or_wb++) begin
         rdc_waddr[DC_TAG_LEN-1:0] |= dc_tag_data_raw_sel[rdc_or_wb][DC_TAG_LEN-1:0];
      end
    end
  
    assign dc_rd_en_rdc = (lsu_dc_axi_arvalid & lsu_dc_axi_arready) & ( | (rdc_way & rdc_replace));
    rvdff #(1) dc_rd_en_rdcff (.*, .clk(free_clk), .din(dc_rd_en_rdc), .dout(dc_rd_en_rdc_q)); 
`endif
    rvdff #(DC_NUM_WAYS) rdc_wayff (.*, .clk(free_clk), .din(rdc_way), .dout(rdc_way_ff));
   
   assign lsu_dc_nonblock_load_data_valid = dc_first_resp;
   assign lsu_dc_nonblock_load_data_error = 1'b0;
   assign lsu_dc_nonblock_load_data_tag = rdc_rd_ptr;
   assign lsu_dc_nonblock_load_data = dc_start_addr_req_q[rdc_rd_ptr][2] ? dc_wr_data_noPar[63:32] :  dc_wr_data_noPar[31:0];
   
   for (genvar rdc_i=0; rdc_i<DC_REQ_BUF_DEPTH; rdc_i++) begin: GenReqBuf 
       
       assign rdc_addr_in_buff_hit[rdc_i] = (rdc_addr_in_buff[31: DC_TAG_LOW] == dc_start_addr_req_q[rdc_i][31: DC_TAG_LOW]) & rdc_valid_q[rdc_i];
       assign rdc_way_in_buff_hit[rdc_i] = rdc_way_q[rdc_i] & {DC_NUM_WAYS{rdc_addr_in_buff_hit[rdc_i]}};
       assign rdc_newReq_en[rdc_i] = rdc_newReq & (rdc_wr_ptr == rdc_i);
       assign dc_last_req_clear[rdc_i] = dc_last_req & (rdc_rd_ptr == rdc_i);
       assign dc_last_resp_clear[rdc_i] = dc_last_resp & (rdc_rd_ptr == rdc_i);
       
        // ffs
       rvdffsc #(1) axi_dc_req_ff2(.*, .clk(free_clk), .din(lsu_axi_dc_req_ff_in), .dout(lsu_axi_dc_req_ff2_q[rdc_i]), .en(rdc_newReq_en[rdc_i]), .clear(dc_last_req_clear[rdc_i]));
       rvdffe #(.WIDTH(32)) dc_addr_reqff (.din(reqAddr_in[31:0]), .dout(dc_start_addr_req_q[rdc_i][31:0]), .en(rdc_newReq_en[rdc_i]), .clk(free_clk), .*); 
       rvdffsc #(1) valid_req_ff(.*, .clk(free_clk), .din(1'b1), .dout(rdc_valid_q[rdc_i]), .en(rdc_newReq_en[rdc_i]), .clear(dc_last_resp_clear[rdc_i]));
       rvdffsc #(DC_NUM_WAYS) way_ff(.*, .clk(free_clk), .din(rdc_way_in), .dout(rdc_way_q[rdc_i]), .en(rdc_newReq_en[rdc_i]), .clear(1'b0)); 

        `ifdef RV_DC_WB_POLICY_ENABLE
            rvdffsc #(DC_NUM_WAYS) rdc_replace_ff(.*, .clk(free_clk), .din(dc_tag_valid_rdc), .dout(rdc_replace_q[rdc_i]), .en(rdc_newReq_en[rdc_i]), .clear(dc_last_resp_clear[rdc_i]));
        `endif
   end
   
   assign lsu_axi_dc_req_ff2 = lsu_axi_dc_req_ff2_q[rdc_rd_ptr];
   assign dc_start_addr_req[31:0] = dc_start_addr_req_q[rdc_rd_ptr][31:0];
   assign rdc_way = rdc_way_q[rdc_rd_ptr];
   `ifdef RV_DC_WB_POLICY_ENABLE
        assign rdc_replace = rdc_replace_q[rdc_rd_ptr];
   `endif
   assign rdc_isin_buff = (( | rdc_addr_in_buff_hit) | ((reqAddr_in[31: DC_TAG_LOW] == rdc_addr_in_buff[31: DC_TAG_LOW]) & rdc_newReq)) & (lsu_freeze_dc3 ? lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma & (lsu_pkt_dc2.store | lsu_pkt_dc2.load): lsu_pkt_dc1.valid & ~lsu_pkt_dc2.dma & (lsu_pkt_dc1.store | lsu_pkt_dc1.load));
   logic [DC_NUM_WAYS-1:0] rdc_way_in_buff_hit_or;
   logic [DC_NUM_WAYS-1:0][DC_REQ_BUF_DEPTH-1:0] rdc_way_in_buff_hit_or_depth;
   
   for(genvar way_hit_or=0; way_hit_or<DC_NUM_WAYS; way_hit_or++) begin: way_hit_orLoop 
       assign rdc_way_in_buff_hit_or[way_hit_or] = (| rdc_way_in_buff_hit_or_depth[way_hit_or]);
       for(genvar way_hit_depth_or=0; way_hit_depth_or<DC_REQ_BUF_DEPTH; way_hit_depth_or++) begin: way_hit_orLoop_depth
            assign rdc_way_in_buff_hit_or_depth[way_hit_or][way_hit_depth_or] = rdc_way_in_buff_hit[way_hit_depth_or][way_hit_or];
       end 
   end
   assign dc_rdc_hit_way = ( | rdc_addr_in_buff_hit) ? rdc_way_in_buff_hit_or[DC_NUM_WAYS-1:0] : rdc_way_in;
   
   assign rdc_empty_ff = ~( | rdc_valid_q); 
   assign rdc_newElm_req = (rdc_newReq & rdc_empty) | (~rdc_empty & (req_state == RDC_IDLE));
   
   //pointer logic
   assign rdc_rd_ptr_plus[DC_REQ_LOG2_BUF_DEPTH-1:0] = (rdc_rd_ptr[DC_REQ_LOG2_BUF_DEPTH-1:0] + 1'b1);
   rvdffsc #(.WIDTH(DC_REQ_LOG2_BUF_DEPTH)) rdc_rd_ptrff (.din(rdc_rd_ptr_plus[DC_REQ_LOG2_BUF_DEPTH-1:0]), .dout(rdc_rd_ptr[DC_REQ_LOG2_BUF_DEPTH-1:0]), .en(dc_last_resp & ~(req_state == RDC_IDLE)), .clear(1'b0), .*);
   assign rdc_wr_ptr_plus[DC_REQ_LOG2_BUF_DEPTH-1:0] = (rdc_wr_ptr[DC_REQ_LOG2_BUF_DEPTH-1:0] + 1'b1);
   rvdffsc #(.WIDTH(DC_REQ_LOG2_BUF_DEPTH)) rdc_wr_ptrff (.din(rdc_wr_ptr_plus[DC_REQ_LOG2_BUF_DEPTH-1:0]), .dout(rdc_wr_ptr[DC_REQ_LOG2_BUF_DEPTH-1:0]), .en(rdc_newReq), .clear(1'b0), .*);
        
        // counter logic
  
      assign axi_dc_rd_count_plus = rdc_newElm_req ? (rdc_empty_ff ? {1'b0, reqAddr_in[5:3]} : {1'b0, dc_start_addr_req[5:3]}) : axi_dc_rd_count +1;
      rvdffsc #(3) rd_count_finff(.*, .clk(free_clk), .din(axi_dc_rd_count_plus + 3'b111), .dout(dc_id_fin_req), .en(rdc_newElm_req), .clear(1'b0));
   
   rvdffsc #(3) rd_count_startff(.*, .clk(free_clk), .din(axi_dc_rd_count_plus), .dout(dc_id_start_req), .en(rdc_newElm_req), .clear(1'b0));
   
   rvdffsc #(3) rd_countff(.*, .clk(free_clk), .din(axi_dc_rd_count_plus[2:0]), .dout(axi_dc_rd_count[2:0]), .en((lsu_dc_axi_arvalid & lsu_dc_axi_arready) | rdc_newElm_req), .clear(1'b0));
   assign axi_dc_rd_count[3] = rdc_rd_ptr[0]; 
   
   // FIFO state machine
   always_comb begin : REQ_SM
      req_nxtstate   = RDC_IDLE;
      req_state_en   = 1'b1;
      case (req_state)
         RDC_IDLE: begin : rdc_idle
                  req_nxtstate = dc_last_req ? FR_WAIT : FR_REQ;
                  req_state_en = ~rdc_empty;
         end
         FR_REQ: begin : fr_req
                  req_nxtstate = dc_last_resp ? RDC_IDLE : FR_WAIT;
                  req_state_en =  dc_last_req;
         end
         FR_WAIT: begin : fr_wait
                  req_nxtstate =  RDC_IDLE;
                  req_state_en =  dc_last_resp;
         end
         default: begin : def_case
                  req_nxtstate   = RDC_IDLE;
                  req_state_en   = 1'b1;
         end
      endcase
  end
  
   rvdffs #(($bits(req_state_t))) req_state_ff (.clk(free_clk), .din(req_nxtstate), .dout({req_state}), .en(req_state_en),   .*);
      
   //buffer conection
   
   `ifdef RV_DC_WB_POLICY_ENABLE
   assign    lsu_dc_axi_arvalid                 = lsu_axi_dc_req_ff2 & ~(req_state == RDC_IDLE) & ~wdc_rdc_isinbuff;
   assign    lsu_dc_axi_rready                  = ~dc_use & ~dc_tag_use & ~dc_rd_en_rdc;
   `else
   assign    lsu_dc_axi_arvalid                 = lsu_axi_dc_req_ff2 & ~(req_state == RDC_IDLE) & ~wdc_rdc_isinbuff;
   assign    lsu_dc_axi_rready                  = ~dc_use & ~dc_tag_use;
   `endif
   assign    lsu_dc_axi_arid[LSU_DC_BUS_TAG-1:0]= LSU_DC_BUS_TAG'(axi_dc_rd_count[3:0]);
   assign    lsu_dc_axi_araddr[31:0]            = {dc_start_addr_req[31:6], axi_dc_rd_count[2:0], 3'b0} ;
   assign    lsu_dc_axi_arsize[2:0]             = 3'b011;
   assign    lsu_dc_axi_arcache[3:0]            = 4'b1111;
   assign    lsu_dc_axi_arprot[2:0]             = 3'b100;
   assign    lsu_dc_axi_arregion[3:0]           = dc_start_addr_req[31:28];
   assign    lsu_dc_axi_arlen[7:0]              = '0;
   assign    lsu_dc_axi_arburst[1:0]            = 2'b01;
   assign    lsu_dc_axi_arqos[3:0]              = '0;
   assign    lsu_dc_axi_arlock                  = '0;
  
   ///////////// end Request buffer/no_buffer
`ifdef RV_DC_WB_POLICY_ENABLE
        
        
   ///////////////////////////////////////////////////// 
   ///////////// Store buffer/no_buffer [WB] /////////// 32 bits data
   ///////////////////////////////////////////////////// 
   //interface
        //in
   logic wdc_wr_en;
   logic wdc_data_dirty_in;
   logic [31:0] wdc_wr_addr_in;  
   logic [31:0] wdc_addr_in_buff;
   logic [31:0] wdc_addr_rdc_in_buff;
   logic [63:0] wdc_wr_data_in;   
   logic [7:0] wdc_wr_byteValid_in;
   logic wdc_single_store;
        //out
   logic wdc_full;
   logic wdc_isin_buff;
   logic wdc_rdc_isinbuff;   
   
    logic [63:0] rdc_to_wdc_half_data;
    assign wdc_addr_rdc_in_buff = lsu_dc_axi_araddr;
    assign rdc_to_wdc_half_data = dc_rdaddr_rdc_q[3] ? dc_rd_data_only[DC_NUM_SUBBANKS*32-1:(DC_NUM_SUBBANKS*32)/2] : dc_rd_data_only[(DC_NUM_SUBBANKS*32)/2-1:0];
    assign wdc_wr_en = dc_rd_en_rdc_q | wdc_single_store;
    assign wdc_wr_addr_in = wdc_single_store ? cache_start_addr_dc2 : dc_rdaddr_rdc_q;
    assign wdc_wr_data_in = wdc_single_store ? dc_store_noWord_data_dc2_shifted : rdc_to_wdc_half_data;
    assign wdc_wr_byteValid_in = wdc_single_store ? dc_store_byte_valid_dc2_shifted : 8'hFF;
    assign wdc_data_dirty_in = wdc_single_store ? 1'b1 : ( | (rdc_way & dc_tag_dirty));
    assign wdc_addr_in_buff = lsu_freeze_dc3 ? cache_start_addr_dc2 : cache_start_addr_dc1;
    assign wdc_single_store = lsu_pkt_dc2.valid & lsu_pkt_dc2.store & hitable_dc2 & ~lsu_freeze_dc3 & ~dc_hit;
   //seniales del bus si lo externalizamos... 
   localparam DC_ST_BUF_WAYS = 4;
   localparam DC_ST_BUF_DEPTH_BLOCK = DC_BLOCK_SIZE/2;
   localparam DC_ST_LOG2_BUF_WAYS = $clog2(DC_ST_BUF_WAYS);
   localparam DC_ST_LOG2_BUF_DEPTH_BLOCK = $clog2(DC_ST_BUF_DEPTH_BLOCK);
   localparam DC_WDC_PTR_SIZE = DC_ST_LOG2_BUF_WAYS + DC_ST_LOG2_BUF_DEPTH_BLOCK + 1; // +1 becasuse we are saving data in 64 ff
   
      //write state machine
   logic   write_state_en;
   //typedef enum logic [2:0] {IDLE=3'b000, SND_ADDR=3'b001, SND_DATA=3'b010, W_RESP=3'b011, NO_RESP=3'b100, IS_DIRTY=3'b101} write_state_t;
   write_state_t write_state, write_nxtstate;
   
   //internal signals
   logic end_wr_bus;
   logic wdc_wr_valid;
   logic wdc_wr_dirty;
   logic [31:0] wdc_wr_addr;   
   logic [63:0] wdc_wr_data;   
   logic wdc_empty;
   logic wdc_waitResp_timeout, wdc_waitResp_err;
   logic end_wr_block;
   logic wdc_last_wr;
   logic wdc_wr_newBlock;
   logic wdc_wr_last_elmBlock;
   
   logic [DC_WDC_PTR_SIZE-1:0] wdc_rd_ptr, wdc_rd_ptr_plus;
   logic [DC_WDC_PTR_SIZE-1:0] wdc_wr_ptr, wdc_wr_ptr_plus;
   logic [DC_ST_LOG2_BUF_WAYS-1:0] wdc_rd_way_ptr, wdc_wr_way_ptr;
   logic [DC_ST_LOG2_BUF_DEPTH_BLOCK-1:0] wdc_rd_block_ptr, wdc_wr_block_ptr;
   logic wdc_rd_word_ptr;
   logic wdc_wr_word_ptr;
   
   logic wdc_wr_req_sent;
   logic wdc_wr_req_addr_sent;
   
   logic [LSU_DC_BUS_TAG-1:0] wdc_count, wdc_count_plus;
   
   assign wdc_wr_newBlock = wdc_wr_en & (wdc_wr_block_ptr == {DC_ST_LOG2_BUF_DEPTH_BLOCK{1'b0}});
   assign wdc_wr_last_elmBlock = (wdc_wr_en & (wdc_wr_block_ptr == {DC_ST_LOG2_BUF_DEPTH_BLOCK{1'b1}})) | wdc_single;
   assign end_wr_block = (wdc_wr_req_sent | ~wdc_wr_dirty) & (wdc_rd_block_ptr == {DC_ST_LOG2_BUF_DEPTH_BLOCK{1'b1}}) & (wdc_rd_word_ptr == 1'b1) | (wdc_wr_req_sent & wdc_single_store);
   assign wdc_last_wr = wdc_single ? ((wdc_rd_ptr+DC_BLOCK_SIZE) == wdc_wr_ptr) : ((wdc_rd_ptr+1) == wdc_wr_ptr);
 
   // full/empty logic
   assign wdc_full = ( & wdc_wr_valid_q) | (wdc_wr_en & ( (wdc_wr_ptr + 1) == wdc_rd_ptr));
   assign wdc_empty = (wdc_rd_ptr == wdc_wr_ptr) & ~wdc_wr_en;  

   // error logic, not in use yet
   assign wdc_waitResp_timeout = 1'b0; 
   assign wdc_waitResp_err = lsu_dc_axi_bvalid & (lsu_dc_axi_bresp != 2'b00);
   
   // pointer logic 
   assign wdc_rd_ptr_plus = wdc_single ? (wdc_rd_ptr + DC_BLOCK_SIZE) : (wdc_rd_ptr + 1'b1);
   rvdffsc #(DC_WDC_PTR_SIZE) wdc_rd_ptrff(.*, .clk(free_clk), .din(wdc_rd_ptr_plus), 
       .dout(wdc_rd_ptr), .en(end_wr_bus | (~wdc_wr_dirty & write_state == IS_DIRTY)), .clear(1'b0));
   assign wdc_wr_ptr_plus = wdc_single_store ? (wdc_wr_ptr + DC_BLOCK_SIZE) : (wdc_wr_ptr + 2'b10);
   rvdffsc #(DC_WDC_PTR_SIZE) wdc_wr_ptrff(.*, .clk(free_clk), .din(wdc_wr_ptr_plus), .dout(wdc_wr_ptr), .en(wdc_wr_en), .clear(1'b0));
   assign wdc_rd_way_ptr = wdc_rd_ptr [DC_WDC_PTR_SIZE-1:DC_ST_LOG2_BUF_DEPTH_BLOCK+1];
   assign wdc_wr_way_ptr = wdc_wr_ptr [DC_WDC_PTR_SIZE-1:DC_ST_LOG2_BUF_DEPTH_BLOCK+1];
   assign wdc_rd_block_ptr = wdc_rd_ptr [DC_ST_LOG2_BUF_DEPTH_BLOCK:1];
   assign wdc_wr_block_ptr = wdc_wr_ptr [DC_ST_LOG2_BUF_DEPTH_BLOCK:1];
   assign wdc_rd_word_ptr = wdc_rd_ptr [0];
   assign wdc_wr_word_ptr = wdc_wr_ptr [0]; // not needed
   
   // id generation 
   assign wdc_count_plus = wdc_count + 1'b1;
   rvdffsc #(LSU_DC_BUS_TAG) wdc_countff(.*, .clk(free_clk), .din(wdc_count_plus), .dout(wdc_count), .en(wdc_wr_req_sent | ~wdc_wr_dirty), .clear(1'b0));
   
   logic [2+DC_ST_LOG2_BUF_DEPTH_BLOCK:0] wdc_wr_addr_addition;
   logic [DC_ST_BUF_WAYS-1:0] [31:0] wdc_wr_addr_q;
   logic [DC_ST_BUF_WAYS-1:0] wdc_addr_in_buff_hit;
   logic [DC_ST_BUF_WAYS-1:0] wdc_addr_rdc_in_buff_hit;
   logic [DC_ST_BUF_WAYS-1:0] wdc_wr_en_way_in;
   logic [DC_ST_BUF_WAYS-1:0] last_elm_block_cl;
   logic [DC_ST_BUF_WAYS-1:0] end_wr_block_cl;
   logic [DC_ST_BUF_WAYS-1:0] wdc_wr_valid_q;
   logic [DC_ST_BUF_WAYS-1:0] wdc_wr_single_q;
   logic [DC_ST_BUF_WAYS-1:0][7:0] wdc_wr_bytevalid_q;
   logic [DC_ST_BUF_WAYS-1:0] wdc_wr_wait_q;
   logic [DC_ST_BUF_WAYS-1:0][DC_ST_BUF_DEPTH_BLOCK-1:0] wdc_wr_en_block_in;
   logic [DC_ST_BUF_WAYS-1:0][DC_ST_BUF_DEPTH_BLOCK-1:0] wdc_wr_dirty_q;
   logic [DC_ST_BUF_WAYS-1:0][DC_ST_BUF_DEPTH_BLOCK-1:0][63:0] wdc_wr_data_q;
   logic wdc_single;
   logic [7:0]wdc_wr_bytevalid;
   
   for (genvar wdc_i=0; wdc_i<DC_ST_BUF_WAYS; wdc_i++) begin: GenStBufWay
       
        assign wdc_wr_en_way_in[wdc_i] = wdc_wr_newBlock & (wdc_wr_way_ptr == wdc_i); //guardamos info bloque cuando primer dato nuevo bloque entra
        assign last_elm_block_cl[wdc_i] = wdc_wr_last_elmBlock & (wdc_wr_way_ptr == wdc_i); //limpiamos wait cuando el bloque se guarda entero.
        assign end_wr_block_cl[wdc_i] = (end_wr_bus | (~wdc_wr_dirty & write_state == IS_DIRTY)) 
                                    & (wdc_rd_way_ptr == wdc_i) & ((wdc_rd_block_ptr == {DC_ST_LOG2_BUF_DEPTH_BLOCK{1'b1}}) | wdc_single);     //limpiamos cuando acabamos de escribir ese bloque

        rvdffe #(.WIDTH(32)) wdc_addr_writeff (.din(wdc_wr_addr_in), .dout(wdc_wr_addr_q[wdc_i]), .en(wdc_wr_en_way_in[wdc_i]), .clk(free_clk), .*); 
        rvdffsc #(1) wdc_wr_validff(.*, .clk(free_clk), .din(1'b1), .dout(wdc_wr_valid_q[wdc_i]), .en(wdc_wr_en_way_in[wdc_i]), .clear(end_wr_block_cl[wdc_i]));
        rvdffsc #(1) wdc_wr_waitff(.*, .clk(free_clk), .din(1'b1), .dout(wdc_wr_wait_q[wdc_i]), .en(wdc_wr_en_way_in[wdc_i]), .clear(last_elm_block_cl[wdc_i]));
        rvdffsc #(1) wdc_wr_singleff(.*, .clk(free_clk), .din(wdc_single_store), .dout(wdc_wr_single_q[wdc_i]), .en(wdc_wr_en_way_in[wdc_i]), .clear(end_wr_block_cl[wdc_i]));
        rvdffsc #(8) wdc_wr_bytevalidff(.*, .clk(free_clk), .din(wdc_wr_byteValid_in), .dout(wdc_wr_bytevalid_q[wdc_i]), .en(wdc_wr_en_way_in[wdc_i]), .clear(end_wr_block_cl[wdc_i]));

       assign wdc_addr_rdc_in_buff_hit[wdc_i] = (wdc_addr_rdc_in_buff[31: DC_TAG_LOW] == wdc_wr_addr_q[wdc_i][31: DC_TAG_LOW]) & wdc_wr_valid_q[wdc_i];
       assign wdc_addr_in_buff_hit[wdc_i] = (wdc_addr_in_buff[31: DC_TAG_LOW] == wdc_wr_addr_q[wdc_i][31: DC_TAG_LOW]) & wdc_wr_valid_q[wdc_i];

        for (genvar wdc_j=0; wdc_j<DC_ST_BUF_DEPTH_BLOCK; wdc_j++) begin: GenStBufDepthBlock            
           assign wdc_wr_en_block_in[wdc_i][wdc_j] = wdc_wr_en & (wdc_wr_block_ptr == wdc_j) & (wdc_wr_way_ptr == wdc_i);
            
           rvdffsc #(1) wdc_wr_dirtyff(.*, .clk(free_clk), .din(wdc_data_dirty_in), 
                .dout(wdc_wr_dirty_q[wdc_i][wdc_j]), .en(wdc_wr_en_block_in[wdc_i][wdc_j]), .clear(1'b0)
           );               
           rvdffe #(.WIDTH(64)) wdc_data_writeff (.din(wdc_wr_data_in), .dout(wdc_wr_data_q[wdc_i][wdc_j]), .en(wdc_wr_en_block_in[wdc_i][wdc_j]), .clk(free_clk), .*);

        end
   end
              
   assign wdc_wr_addr_addition = {wdc_rd_block_ptr, wdc_rd_word_ptr} << 2; 
   logic [DC_BANKOFF_HIGH-1:0] wdc_wr_addr_add_sel;
   assign wdc_wr_addr_add_sel = wdc_wr_addr_q[wdc_rd_way_ptr][DC_BANKOFF_HIGH-1:0] + wdc_wr_addr_addition;
   assign wdc_wr_addr = {wdc_wr_addr_q[wdc_rd_way_ptr][31:DC_TAG_LOW], wdc_wr_addr_add_sel};
   assign wdc_wr_valid = wdc_wr_valid_q[wdc_rd_way_ptr];
   assign wdc_single = wdc_wr_single_q[wdc_rd_way_ptr];
   assign wdc_wr_bytevalid = wdc_wr_bytevalid_q[wdc_rd_way_ptr];

   assign wdc_wr_data = wdc_wr_data_q[wdc_rd_way_ptr][wdc_rd_block_ptr];
   assign wdc_wr_dirty = wdc_wr_dirty_q[wdc_rd_way_ptr][wdc_rd_block_ptr];
   
   assign wdc_isin_buff = (( | wdc_addr_in_buff_hit) | ((wdc_wr_addr_in[31: DC_TAG_LOW] == wdc_addr_in_buff[31: DC_TAG_LOW]) & wdc_wr_en)) & (lsu_freeze_dc3 ? lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma & lsu_pkt_dc2.load : lsu_pkt_dc1.valid & ~lsu_pkt_dc2.dma & lsu_pkt_dc1.load); 
   assign wdc_rdc_isinbuff = ( | wdc_addr_rdc_in_buff_hit) | ((wdc_addr_rdc_in_buff[31: DC_TAG_LOW] == wdc_addr_in_buff[31: DC_TAG_LOW]) & wdc_wr_en);
   
   assign wdc_wr_req_addr_sent = lsu_dc_axi_awready & lsu_dc_axi_awvalid & lsu_dc_axi_wvalid;
   assign wdc_wr_req_sent = lsu_dc_axi_wready & lsu_dc_axi_wvalid;
      
   assign end_wr_bus = (wdc_waitResp_timeout | wdc_waitResp_err) | ((lsu_dc_axi_bresp == 2'b00) & lsu_dc_axi_bvalid);  


   // FIFO state machine
   always_comb begin : WRITE_SM
      write_nxtstate   = IDLE;
      write_state_en   = 1'b1;
      case (write_state)
         IDLE: begin : idle
                  write_nxtstate = IS_DIRTY;
                  write_state_en = ~wdc_empty;
         end
         IS_DIRTY: begin : is_dirty
                  write_nxtstate = (wdc_wr_dirty & wdc_wr_valid) ? SND_ADDR : IDLE;
                  write_state_en = (wdc_wr_dirty & wdc_wr_valid) | wdc_empty | wdc_last_wr;
         end
         SND_ADDR: begin : snd_addr
                  write_nxtstate = (lsu_dc_axi_wready) ? W_RESP : SND_DATA;
                  write_state_en = wdc_wr_req_addr_sent;
         end
         SND_DATA: begin : snd_data
                  write_nxtstate =  W_RESP;
                  write_state_en =  wdc_wr_req_sent;
         end
         W_RESP: begin : w_resp
                  write_nxtstate =  (wdc_waitResp_timeout | wdc_waitResp_err) ? NO_RESP : ((wdc_empty | wdc_last_wr)? IDLE : IS_DIRTY);
                  write_state_en = end_wr_bus;
         end
         NO_RESP: begin : no_resp //no tratamos el error.
                  write_nxtstate =  wdc_empty ? IDLE : IS_DIRTY;
                  write_state_en =  1'b1;
         end
         default: begin : def_case
                  write_nxtstate   = IDLE;
                  write_state_en   = 1'b1;
         end
      endcase
  end
  
   rvdffs #(($bits(write_state_t))) write_state_ff (.clk(free_clk), .din(write_nxtstate), .dout({write_state}), .en(write_state_en),   .*);
   
   assign lsu_dc_axi_awvalid = (write_state == SND_ADDR);
   assign lsu_dc_axi_wvalid = ((write_state == SND_ADDR) | (write_state == SND_DATA));
   assign lsu_dc_axi_awaddr = wdc_wr_addr;
   assign lsu_dc_axi_wdata = wdc_wr_data;
   assign lsu_dc_axi_wstrb = wdc_single ? wdc_wr_bytevalid : (wdc_rd_word_ptr ? 8'hF0 : 8'h0F);
   assign lsu_dc_axi_bready = 1'b1;
   assign lsu_dc_axi_awid = wdc_count;
   assign lsu_dc_axi_awsize = 3'b011;
`endif
        

   //store buffer conection    
`ifndef RV_DC_WB_POLICY_ENABLE
        assign wdc_wr_en = ((lsu_pkt_dc2.valid & lsu_pkt_dc2.store) & hitable_dc2 & ~lsu_freeze_dc3);   
        assign wdc_wr_addr_in = cache_start_addr_dc2;
        assign wdc_wr_data_in = dc_store_noWord_data_dc2_shifted;//store_data_dc2[31:0];
        assign wdc_wr_byteValid_in = dc_store_byte_valid_dc2_shifted;
        assign wdc_addr_in_buff = lsu_freeze_dc3 ? cache_start_addr_dc2 : cache_start_addr_dc1;
        assign wdc_addr_rdc_in_buff = lsu_dc_axi_araddr;

   ///////////////////////////////////////////////////// 
   ///////////// Store buffer/no_buffer //////////////// 32 bits data
   ///////////////////////////////////////////////////// 
   //interface
        //in
   logic wdc_wr_en;
   logic [31:0] wdc_wr_addr_in;  
   logic [31:0] wdc_addr_in_buff;
   logic [31:0] wdc_addr_rdc_in_buff;
   logic [63:0] wdc_wr_data_in;   
   logic [7:0] wdc_wr_byteValid_in;
        //out
   logic wdc_isin_buff;
   logic wdc_rdc_isinbuff;      
   logic wdc_full;
   
   //seniales del bus si lo externalizamos... 
   
   //internal signals
   logic wdc_wr_valid;
   logic [31:0] wdc_wr_addr;   
   logic [63:0] wdc_wr_data;   
   logic [7:0] wdc_wr_byteValid;
   logic wdc_empty, wdc_empty_ff;
   logic wdc_waitResp_timeout, wdc_waitResp_err;
   logic [3:0] wdc_count_plus, wdc_count;
   logic end_wr_bus;
   logic wdc_last_wr;
   logic wdc_doubleWord;
   
   logic [DC_REQ_LOG2_BUF_DEPTH-1:0] wdc_rd_ptr, wdc_rd_ptr_plus;
   logic [DC_REQ_LOG2_BUF_DEPTH-1:0] wdc_wr_ptr, wdc_wr_ptr_plus;
   
   logic [DC_REQ_BUF_DEPTH-1:0] wdc_wr_en_in;
   logic [DC_REQ_BUF_DEPTH-1:0] end_wr_bus_in;
   
   logic [DC_REQ_BUF_DEPTH-1:0] [31:0] wdc_wr_addr_q;
   logic [DC_REQ_BUF_DEPTH-1:0] [63:0] wdc_wr_data_q;
   logic [DC_REQ_BUF_DEPTH-1:0] [(DC_BLOCK_SIZE/2)-1:0] wdc_wr_byteValid_q;
   logic [DC_REQ_BUF_DEPTH-1:0] wdc_wr_valid_q;
   logic [DC_REQ_BUF_DEPTH-1:0] wdc_addr_in_buff_hit;
   logic [DC_REQ_BUF_DEPTH-1:0] wdc_addr_rdc_in_buff_hit;
   
   assign wdc_last_wr = (wdc_rd_ptr+1 == wdc_wr_ptr);  
   assign end_wr_bus = (wdc_waitResp_timeout | wdc_waitResp_err) | ((lsu_dc_axi_bresp == 2'b00) & lsu_dc_axi_bvalid);
   assign wdc_waitResp_timeout = 1'b0;
   assign wdc_waitResp_err = lsu_dc_axi_bvalid & (lsu_dc_axi_bresp != 2'b00);
   assign wdc_full = ( & wdc_wr_valid_q) | (wdc_wr_en & ( (wdc_wr_ptr + 1) == wdc_rd_ptr));
   assign wdc_empty = wdc_empty_ff & ~wdc_wr_en;
   
   // id generation 
   assign wdc_count_plus = wdc_count + 1'b1;
   rvdffsc #(4) wdc_countff(.*, .clk(free_clk), .din(wdc_count_plus[3:0]), .dout(wdc_count[3:0]), .en((lsu_dc_axi_awvalid & lsu_dc_axi_awready)), .clear(1'b0)); 
   
   
   for (genvar wdc_i=0; wdc_i<DC_ST_BUF_DEPTH; wdc_i++) begin: GenStBuf        
              
       assign wdc_addr_in_buff_hit[wdc_i] = (wdc_addr_in_buff[31: DC_TAG_LOW] == wdc_wr_addr_q[wdc_i][31: DC_TAG_LOW]) & wdc_wr_valid_q[wdc_i];
       assign wdc_addr_rdc_in_buff_hit[wdc_i] = (wdc_addr_rdc_in_buff[31: DC_TAG_LOW] == wdc_wr_addr_q[wdc_i][31: DC_TAG_LOW]) & wdc_wr_valid_q[wdc_i];
       assign wdc_wr_en_in[wdc_i] = wdc_wr_en & (wdc_wr_ptr == wdc_i);
       assign end_wr_bus_in[wdc_i] = end_wr_bus & (wdc_rd_ptr == wdc_i);
       
       //ffs
       rvdffe #(.WIDTH(32)) wdc_addr_writeff (.din(wdc_wr_addr_in), .dout(wdc_wr_addr_q[wdc_i]), .en(wdc_wr_en_in[wdc_i]), .clk(free_clk), .*); 
       rvdffe #(.WIDTH(64)) wdc_data_writeff (.din(wdc_wr_data_in), .dout(wdc_wr_data_q[wdc_i]), .en(wdc_wr_en_in[wdc_i]), .clk(free_clk), .*); 
       rvdffe #((DC_BLOCK_SIZE/2)) wdc_wr_byteValidff(.*, .clk(free_clk), .din(wdc_wr_byteValid_in), .dout(wdc_wr_byteValid_q[wdc_i]), .en(wdc_wr_en_in[wdc_i]));
       rvdffsc #(1) wdc_wr_validff(.*, .clk(free_clk), .din(1'b1), .dout(wdc_wr_valid_q[wdc_i]), .en(wdc_wr_en_in[wdc_i]), .clear(end_wr_bus_in[wdc_i]));
       
   end
   
   assign wdc_wr_addr = wdc_wr_addr_q[wdc_rd_ptr];
   assign wdc_wr_data = wdc_wr_data_q[wdc_rd_ptr];
   assign wdc_wr_byteValid = wdc_wr_byteValid_q[wdc_rd_ptr];
   assign wdc_wr_valid = wdc_wr_valid_q[wdc_rd_ptr];
  
   assign wdc_empty_ff = ~( | wdc_wr_valid_q); 
   
   assign wdc_isin_buff = (( | wdc_addr_in_buff_hit) | ((wdc_wr_addr_in[31: DC_TAG_LOW] == wdc_addr_in_buff[31: DC_TAG_LOW]) & wdc_wr_en)) & (lsu_freeze_dc3 ? lsu_pkt_dc2.valid & ~lsu_pkt_dc2.dma & lsu_pkt_dc2.load : lsu_pkt_dc1.valid & ~lsu_pkt_dc2.dma & lsu_pkt_dc1.load); 
   assign wdc_rdc_isinbuff = ( | wdc_addr_rdc_in_buff_hit) | ((wdc_addr_rdc_in_buff[31: DC_TAG_LOW] == wdc_addr_in_buff[31: DC_TAG_LOW]) & wdc_wr_en);
   
      //pointer logic
   assign wdc_rd_ptr_plus[DC_ST_LOG2_BUF_DEPTH-1:0] = (wdc_rd_ptr[DC_ST_LOG2_BUF_DEPTH-1:0] + 1'b1);
   rvdffsc #(.WIDTH(DC_ST_LOG2_BUF_DEPTH)) wdc_rd_ptrff (.din(wdc_rd_ptr_plus[DC_ST_LOG2_BUF_DEPTH-1:0]), .dout(wdc_rd_ptr[DC_ST_LOG2_BUF_DEPTH-1:0]), .en(end_wr_bus), .clear(1'b0), .*);
   assign wdc_wr_ptr_plus[DC_ST_LOG2_BUF_DEPTH-1:0] = (wdc_wr_ptr[DC_ST_LOG2_BUF_DEPTH-1:0] + 1'b1);
   rvdffsc #(.WIDTH(DC_ST_LOG2_BUF_DEPTH)) wdc_wr_ptrff (.din(wdc_wr_ptr_plus[DC_ST_LOG2_BUF_DEPTH-1:0]), .dout(wdc_wr_ptr[DC_ST_LOG2_BUF_DEPTH-1:0]), .en(wdc_wr_en), .clear(1'b0), .*);
        
   
   //write state machine
   logic   write_state_en;
   typedef enum logic [2:0] {IDLE=3'b000, SND_ADDR=3'b001, SND_DATA=3'b010, W_RESP=3'b011, NO_RESP=3'b100} write_state_t;
   write_state_t write_state, write_nxtstate;

   // FIFO state machine
   always_comb begin : WRITE_SM
      write_nxtstate   = IDLE;
      write_state_en   = 1'b1;
      case (write_state)
         IDLE: begin : idle
                  write_nxtstate = SND_ADDR;
                  write_state_en = ~wdc_empty;
         end
         SND_ADDR: begin : snd_addr
                  write_nxtstate = (lsu_dc_axi_wready) ? W_RESP : SND_DATA;
                  write_state_en = lsu_dc_axi_awready & lsu_dc_axi_awvalid & lsu_dc_axi_wvalid;
         end
         SND_DATA: begin : snd_data
                  write_nxtstate =  W_RESP;
                  write_state_en =  lsu_dc_axi_wready & lsu_dc_axi_wvalid;
         end
         W_RESP: begin : w_resp
                  write_nxtstate =  (wdc_waitResp_timeout | wdc_waitResp_err) ? NO_RESP : ((wdc_empty | wdc_last_wr)? IDLE : SND_ADDR);
                  write_state_en = end_wr_bus;
         end
         NO_RESP: begin : no_resp 
                  write_nxtstate =  wdc_empty ? IDLE : SND_ADDR;
                  write_state_en =  1'b1;
         end
         default: begin : def_case
                  write_nxtstate   = IDLE;
                  write_state_en   = 1'b1;
         end
      endcase
  end
  
   rvdffs #(($bits(write_state_t))) write_state_ff (.clk(free_clk), .din(write_nxtstate), .dout({write_state}), .en(write_state_en),   .*);
   
   assign lsu_dc_axi_awvalid = (write_state == SND_ADDR);
   assign lsu_dc_axi_wvalid = ((write_state == SND_ADDR) | (write_state == SND_DATA));
   assign lsu_dc_axi_awaddr = wdc_wr_addr;
   assign lsu_dc_axi_wdata = wdc_wr_data;
   assign lsu_dc_axi_wstrb = wdc_wr_byteValid;
   assign lsu_dc_axi_bready = 1'b1;
   assign lsu_dc_axi_awid = wdc_count;
   assign lsu_dc_axi_awsize = 3'b011;
   
`endif

   assign lsu_dc_axi_awprot[2:0]           = '0;
   assign lsu_dc_axi_awcache[3:0]          = 4'b1111;
   assign lsu_dc_axi_awregion[3:0]         = wdc_wr_addr[31:28];
   assign lsu_dc_axi_awlen[7:0]            = '0;
   assign lsu_dc_axi_awburst[1:0]          = 2'b01;
   assign lsu_dc_axi_awqos[3:0]            = '0;
   assign lsu_dc_axi_awlock                = '0;
   assign lsu_dc_axi_wlast                 = '1;
   
   ///////////// end Store buffer/no_buffer


endmodule