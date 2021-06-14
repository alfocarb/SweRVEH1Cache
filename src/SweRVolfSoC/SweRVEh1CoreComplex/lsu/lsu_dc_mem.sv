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
// $$
//
//
// Owner:
// Function: 
// Comments:
//
//********************************************************************************

module lsu_dc_mem
    (
    input logic         clk,
    input logic         rst_l,                          // reset
    input logic         scan_mode,                      //QUE ES??   
    input logic         clk_override,

    input logic [31:2]   dc_rw_addr,  // addres de escritura //ojo, porque creo que necesitamos poder escribir y leer el mismo tiempo en direcciones diferentes por lo que puede que no funcione 
    input logic [31:2]   dc_rw_tag_addr,  
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_wr_en,    // activa las ways en las que vamos a escribir.
    input logic [`RV_DC_NUM_WAYS-1:0]      lsu_dc_tag_wren,
    input logic         dc_rd_en,                       // read enable   
    input logic         dc_wr_word,   
    //premux data, aun no se muy bien para que sirve
    //select premux data
    
/*`ifdef RV_ICACHE_ECC                                    //uso el de ICACHE temporalmente, hay que definir uno nosotros
      input  logic [83:0]               dc_wr_data,     // Data to fill to the Icache. With ECC
      output logic [167:0]              dc_rd_data ,    // Data read from Icache. 2x64bits + parity bits. F2 stage. With ECC
`else*/
      input  logic [67:0]               dc_wr_data,     // Data to fill to the dcache. With Parity escritura
      output logic [135:0]              dc_rd_data ,    // Data read from Icache. 2x64bits + parity bits. F2 stage. With Parity lecutra
//`endif 
  
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_tag_valid,   // Si el tag de cada way es valido o no. viene desde el ctl
    `ifdef RV_DC_WB_POLICY_ENABLE
        output logic [`RV_DC_NUM_WAYS-1:0] [(32-`RV_DC_TAG_HIGH):0] dc_tag_data_raw,
    `endif 

    input logic [`RV_DC_NUM_WAYS-1:0]     dc_rd_hit_ff,  
    output logic [`RV_DC_NUM_WAYS-1:0]    dc_rd_hit,      // indica hit en la lectura de cada una de las ways
    output logic         dc_tag_perr                    // Tag Parity error    
);

 
`include "global.h"
//instaciacion modulo del addres (tipo IC_TAG)

//instanciacion modulo de datos (tipo IC_DATA)   

//resto de ff que tenemos ahora creo que deberian ir en el controller.
   DC_TAG  dc_tag_inst
          (
           .*,
           .dc_wr_en     (dc_wr_en[`RV_DC_NUM_WAYS-1:0]),
           //.dc_debug_addr(dc_debug_addr[ICACHE_TAG_HIGH-1:2]),
           .dc_rw_addr   (dc_rw_tag_addr[31:3])
           ) ;

   DC_DATA  dc_data_inst
          (
           .*,
           .dc_wr_en     (dc_wr_en[`RV_DC_NUM_WAYS-1:0]),
           //.dc_debug_addr(ic_debug_addr[ICACHE_TAG_HIGH-1:2]),
           .dc_rw_addr   (dc_rw_addr[`RV_DC_TAG_HIGH-1:2])
           ) ;

endmodule 


module DC_TAG 
     (
    input logic         clk,
    input logic         rst_l,
    input  logic        scan_mode,
    input logic clk_override,

    input logic [31:3]   dc_rw_addr,  // addres de escritura //ojo, porque creo que necesitamos poder escribir y leer el mismo tiempo en direcciones diferentes por lo que puede que no funcione 
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_wr_en,    // activa las ways en las que vamos a escribir.
    input logic [`RV_DC_NUM_WAYS-1:0]      lsu_dc_tag_wren,
    input logic         dc_rd_en,  
    
    `ifdef RV_DC_WB_POLICY_ENABLE
        output logic [`RV_DC_NUM_WAYS-1:0][(32-`RV_DC_TAG_HIGH):0] dc_tag_data_raw,    
    `endif 
    
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_tag_valid,

      output logic [`RV_DC_NUM_WAYS-1:0]   dc_rd_hit,               // indica hit en la lectura de cada una de las ways
      output logic         dc_tag_perr              // Tag Parity error    
      ) ;

`include "global.h"
    logic [DC_NUM_WAYS-1:0]     dc_tag_valid_q;
    `ifndef RV_DC_NONE_MUL
        logic [DC_MUL_DEPTH-1:0] [DC_NUM_WAYS-1:0]         dc_tag_clken ;
        logic [DC_NUM_WAYS-1:0][DC_MUL_DEPTH-1:0]          dc_tag_clk ;
    `else
        logic [DC_NUM_WAYS-1:0]          dc_tag_clken ;
        logic [DC_NUM_WAYS-1:0]          dc_tag_clk ;        
    `endif
    `ifndef RV_DC_WB_POLICY_ENABLE
        logic [DC_NUM_WAYS-1:0][DC_TAG_LEN:0] dc_tag_data_raw;        
    `endif 
    `ifndef RV_DC_NONE_MUL
        logic  [DC_NUM_WAYS-1:0][DC_MUL_DEPTH-1:0] [DC_TAG_LEN:0] dc_tag_data_raw_o;   //TODO ponemos un bit de mas para testing //TODO
        logic  [DC_NUM_WAYS-1:0][DC_MUL_DEPTH-1:0] [DC_TAG_LEN:0] dc_tag_data_raw_o_ext;  //TODO
        for (genvar dc_cpr=0; dc_cpr<DC_NUM_WAYS; dc_cpr++) begin: DC_CP_TAG_RAW
            assign dc_tag_data_raw[dc_cpr] = dc_tag_data_raw_o[dc_cpr][dc_rw_addr[DC_TAG_HIGH-1:DC_TAG_MUL_LOW]];
        end
        logic [DC_NUM_WAYS-1:0] [32:DC_TAG_HIGH] dc_tag_data_comp;
    `endif 
    logic [DC_NUM_WAYS-1:0] [32:DC_TAG_HIGH] w_tout;
    logic [DC_NUM_WAYS-1:0] [32:DC_TAG_HIGH] w_toutff_debug;
    logic [DC_TAG_LEN:0] dc_tag_wr_data ;
    logic [DC_NUM_WAYS-1:0]  dc_tag_way_perr ;
    logic   dc_tag_parity ;
    logic [DC_TAG_HIGH-1:6]   dc_rw_addr_q;
    logic [31:DC_TAG_LOW]   dc_rw_addr_ff;
       
    `ifndef RV_DC_NONE_MUL    
        for (genvar dc_ctm=0; dc_ctm<DC_MUL_DEPTH; dc_ctm++) begin: DC_CLK_TAG_MUL
            assign  dc_tag_clken[dc_ctm] [DC_NUM_WAYS-1:0] = ({DC_NUM_WAYS{dc_rd_en | clk_override}} | lsu_dc_tag_wren[DC_NUM_WAYS-1:0]);
        end
    `else
        assign  dc_tag_clken[DC_NUM_WAYS-1:0]  = ({DC_NUM_WAYS{dc_rd_en | clk_override}} | dc_wr_en[DC_NUM_WAYS-1:0]);
    `endif
    
    rvdff #(32-DC_TAG_LOW) dc_adr_ff (.*, .din ({dc_rw_addr[31:DC_TAG_LOW]}), .dout({dc_rw_addr_ff[31:DC_TAG_LOW]}));
      
    rveven_paritygen #(32-DC_TAG_HIGH) pargen (.data_in(dc_rw_addr[31:DC_TAG_HIGH]), .parity_out(dc_tag_parity));
    assign  dc_tag_wr_data[DC_TAG_LEN:0] = {dc_tag_parity, dc_rw_addr[31:DC_TAG_HIGH]};
     
    logic[DC_MUL_DEPTH-1:0] sb_tag_wren_sel;  
    
    logic [DC_TAG_HIGH-1:DC_TAG_LOW] dc_rw_block_debug, dc_rw_blockff_debug;
    logic [31:DC_TAG_HIGH] dc_rw_tag_debug, dc_rw_tagff_debug;
     
    for (genvar i=0; i<DC_NUM_WAYS; i++) begin: DC_TAG_WAYS
         
        for (genvar dc_tm=0; dc_tm<DC_MUL_DEPTH; dc_tm++) begin: DC_DEPTH_TAG_MUL
            `ifndef RV_DC_NONE_MUL
                assign sb_tag_wren_sel[dc_tm] = (dc_rw_addr[DC_TAG_HIGH-1: DC_TAG_MUL_LOW] == dc_tm);
                rvclkhdr dc_tag_c1_cgc  ( .en(dc_tag_clken[dc_tm][i]), .l1clk(dc_tag_clk[i][dc_tm]), .* );
                ram_64x21  dc_way_tag (
                                     .CLK(dc_tag_clk[i][dc_tm]),
                                     .WE (lsu_dc_tag_wren[i] & sb_tag_wren_sel[dc_tm]),
                                     .D  (dc_tag_wr_data[DC_TAG_LEN:0]), //{{21-DC_TAG_LEN-1{1'b0}}, 
                                     .ADR(dc_rw_addr[DC_TAG_MUL_LOW-1:DC_TAG_LOW]), 
                                     .Q  (dc_tag_data_raw_o_ext[i][dc_tm][DC_TAG_LEN:0])
                                    );
                assign dc_tag_data_raw_o[i][dc_tm][DC_TAG_LEN:0] = dc_tag_data_raw_o_ext[i][dc_tm][DC_TAG_LEN:0];
                assign dc_rw_block_debug = dc_rw_addr[DC_TAG_HIGH-1:DC_TAG_LOW];
                assign dc_rw_blockff_debug = dc_rw_addr_ff[DC_TAG_HIGH-1:DC_TAG_LOW];
                assign dc_rw_tag_debug = dc_rw_addr[31:DC_TAG_HIGH];
                assign dc_rw_tagff_debug = dc_rw_addr_ff[31:DC_TAG_HIGH];
                assign w_toutff_debug[i][31:DC_TAG_HIGH] = dc_tag_data_raw_o[i][dc_rw_addr_ff[DC_TAG_HIGH-1:DC_TAG_MUL_LOW]][DC_TAG_LEN-1:0] ;
                
                assign w_tout[i][31:DC_TAG_HIGH] = dc_tag_data_raw_o[i][dc_rw_addr_ff[DC_TAG_HIGH-1:DC_TAG_MUL_LOW]][DC_TAG_LEN-1:0] ;
                assign w_tout[i][32]                 = dc_tag_data_raw_o[i][dc_rw_addr_ff [DC_TAG_HIGH-1:DC_TAG_MUL_LOW]][DC_TAG_LEN] ;
                assign dc_tag_data_comp[i][31:DC_TAG_HIGH] = dc_tag_data_raw_o[i][dc_rw_addr_ff[DC_TAG_HIGH-1:DC_TAG_MUL_LOW]][DC_TAG_LEN-1:0] ;
                assign dc_rd_hit[i] = (dc_tag_data_comp[i][31:DC_TAG_HIGH] == dc_rw_addr_ff[31:DC_TAG_HIGH]) & dc_tag_valid[i];
            `else
                rvclkhdr dc_tag_c1_cgc  ( .en(dc_tag_clken[i]), .l1clk(dc_tag_clk[i]), .* );
                ram_64x21  dc_way_tag (
                                     .CLK(dc_tag_clk[i]),
                                     .WE (lsu_dc_tag_wren[i]),
                                     .D  (dc_tag_wr_data[DC_TAG_LEN:0]),
                                     .ADR(dc_rw_addr[DC_TAG_MUL_LOW-1:DC_TAG_LOW]), 
                                     .Q  (dc_tag_data_raw[i][DC_TAG_LEN:0])
                                    );
                assign w_tout[i][31:DC_TAG_HIGH] = dc_tag_data_raw[i][31-DC_TAG_HIGH:0] ;
                assign w_tout[i][32]                 = dc_tag_data_raw[i][DC_TAG_LEN] ;
                assign dc_rd_hit[i] = (dc_tag_data_raw[i][DC_TAG_LEN-1:0] == dc_rw_addr_ff[31:DC_TAG_HIGH]) & dc_tag_valid[i];
            `endif        
            rveven_paritycheck #(32-DC_TAG_HIGH) parcheck(.data_in   (w_tout[i][31:DC_TAG_HIGH]),
                                                   .parity_in (w_tout[i][32]),
                                                   .parity_err(dc_tag_way_perr[i]));
        end
    end
    
     assign  dc_tag_perr  = | (dc_tag_way_perr[DC_NUM_WAYS-1:0] & dc_tag_valid[DC_NUM_WAYS-1:0]);

endmodule


module DC_DATA
     (
    input logic         clk,
    input logic         rst_l,
    input logic         scan_mode,
      input logic clk_override,

    input logic [`RV_DC_TAG_HIGH-1:2]   dc_rw_addr,  // addres de escritura //ojo, porque creo que necesitamos poder escribir y leer el mismo tiempo en direcciones diferentes por lo que puede que no funcione 
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_wr_en,    // activa las ways en las que vamos a escribir.
    input logic         dc_rd_en,                    // read enable  
    input logic         dc_wr_word,
    
    //premux to add
    
/* `ifdef RV_ICACHE_ECC
     input  logic [83:0]                dc_wr_data,  // Data to fill to the Icache. With ECC
     output logic [167:0]               dc_rd_data , // Data read from Icache. 2x64bits + parity bits. F2 stage. With ECC
 `else */
     input  logic [67:0]                dc_wr_data,  // Data to fill to the Icache. With Parity
     output logic [135:0]               dc_rd_data , // Data read from Icache. 2x64bits + parity bits. F2 stage. With Parity
//`endif
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_rd_hit_ff,    
    input logic [`RV_DC_NUM_WAYS-1:0]     dc_rd_hit    // indica hit en la lectura de cada una de las ways
      ) ;

`include "global.h"
logic [DC_NUM_SUBBANKS-1:0][33:0] dc_sb_wr_data;
logic [DC_NUM_WAYS-1:0] [135:0]     wb_dout_q;
logic [DC_NUM_SUBBANKS-1:0][DC_NUM_WAYS-1:0]   dc_b_sb_wren;
`ifndef RV_DC_NONE_MUL
    logic [DC_TAG_HIGH-1:2] dc_rw_addr_ff;
    logic [DC_NUM_WAYS-1:0][DC_MUL_DEPTH-1:0][135:0]     wb_dout;//TODO
    logic [DC_MUL_DEPTH-1:0][DC_NUM_WAYS-1:0]            dc_bank_way_clken;//TODO
    logic [DC_MUL_DEPTH-1:0][DC_NUM_WAYS-1:0]            dc_bank_way_clk;//TODO
`else
    logic [DC_NUM_WAYS-1:0][135:0]    wb_dout;
    logic [DC_NUM_WAYS-1:0]           dc_bank_way_clken;
    logic [DC_NUM_WAYS-1:0]           dc_bank_way_clk;
`endif
   assign  dc_sb_wr_data[0][33:0]   =  dc_wr_data[33:0] ;
   assign  dc_sb_wr_data[1][33:0]   =  dc_wr_data[67:34] ;
   assign  dc_sb_wr_data[2][33:0]   =  dc_wr_data[33:0] ;
   assign  dc_sb_wr_data[3][33:0]   =  dc_wr_data[67:34] ;
   
   assign  dc_b_sb_wren[0][DC_NUM_WAYS-1:0]  = dc_wr_en[DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{~dc_rw_addr[3]}} & {DC_NUM_WAYS{(~dc_wr_word | ~dc_rw_addr[2])}};
   assign  dc_b_sb_wren[1][DC_NUM_WAYS-1:0]  = dc_wr_en[DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{~dc_rw_addr[3]}} & {DC_NUM_WAYS{(~dc_wr_word |  dc_rw_addr[2])}};
   assign  dc_b_sb_wren[2][DC_NUM_WAYS-1:0]  = dc_wr_en[DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{ dc_rw_addr[3]}} & {DC_NUM_WAYS{(~dc_wr_word | ~dc_rw_addr[2])}};
   assign  dc_b_sb_wren[3][DC_NUM_WAYS-1:0]  = dc_wr_en[DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{ dc_rw_addr[3]}} & {DC_NUM_WAYS{(~dc_wr_word |  dc_rw_addr[2])}};
  
  logic[DC_MUL_DEPTH-1:0] sb_wren_sel;//TODO
`ifndef RV_DC_NONE_MUL
  for (genvar dc_cm=0; dc_cm<DC_MUL_DEPTH; dc_cm++) begin: DC_CLK_MUL
     assign sb_wren_sel[dc_cm] = (dc_rw_addr[DC_TAG_HIGH-1: DC_TAG_MUL_LOW] == dc_cm);
     assign  dc_bank_way_clken[dc_cm][DC_NUM_WAYS-1:0]  = (({DC_NUM_WAYS{dc_rd_en | clk_override }}) |
                                      (dc_b_sb_wren[0][DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{sb_wren_sel[dc_cm]}}) |
                                      (dc_b_sb_wren[1][DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{sb_wren_sel[dc_cm]}}) |
                                      (dc_b_sb_wren[2][DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{sb_wren_sel[dc_cm]}}) |
                                      (dc_b_sb_wren[3][DC_NUM_WAYS-1:0] & {DC_NUM_WAYS{sb_wren_sel[dc_cm]}}) );
  end
`else
    assign  dc_bank_way_clken[DC_NUM_WAYS-1:0]  = (({DC_NUM_WAYS{dc_rd_en | clk_override }}) |
                                      dc_b_sb_wren[0][DC_NUM_WAYS-1:0] |
                                      dc_b_sb_wren[1][DC_NUM_WAYS-1:0] |
                                      dc_b_sb_wren[2][DC_NUM_WAYS-1:0] |
                                      dc_b_sb_wren[3][DC_NUM_WAYS-1:0] );
`endif
   
    for (genvar i=0; i<DC_NUM_WAYS; i++) begin: DC_WAYS
         
         for (genvar dc_m=0; dc_m<DC_MUL_DEPTH; dc_m++) begin: DC_DEPTH_MUL
            `ifndef RV_DC_NONE_MUL
                rvoclkhdr bank_way_c1_cgc  ( .en(dc_bank_way_clken[dc_m][i]), .l1clk(dc_bank_way_clk[dc_m][i]), .* );
            `else  
                rvoclkhdr bank_way_c1_cgc  ( .en(dc_bank_way_clken[i]), .l1clk(dc_bank_way_clk[i]), .* );
            `endif
            for (genvar k=0; k<DC_NUM_SUBBANKS; k++) begin: DC_SUBBANKS   // 16B subbank
                `ifndef RV_DC_NONE_MUL
                    `RV_DC_DATA_CELL  dc_bank_sb_way_data (
                                             .CLK(dc_bank_way_clk[dc_m][i]),
                                             .WE (dc_b_sb_wren[k][i] & sb_wren_sel[dc_m]), 
                                             .D  (dc_sb_wr_data[k][33:0]),
                                             .ADR(dc_rw_addr[DC_TAG_MUL_LOW-1:DC_BANKOFF_LOW]),
                                             .Q  (wb_dout[i][dc_m][(k+1)*34-1:k*34])
                                            );
                `else
                    `RV_DC_DATA_CELL  dc_bank_sb_way_data (
                                             .CLK(dc_bank_way_clk[i]),
                                             .WE (dc_b_sb_wren[k][i]), 
                                             .D  (dc_sb_wr_data[k][33:0]),
                                             .ADR(dc_rw_addr[DC_TAG_MUL_LOW-1:DC_BANKOFF_LOW]),
                                             .Q  (wb_dout[i][(k+1)*34-1:k*34])
                                            );
                `endif
            end // block: SUBBANKS
        end    
    end

    logic [DC_NUM_WAYS-1:0][135:0] wb_dout_way;
    logic [DC_NUM_WAYS-1:0] [135:0] wb_dout_way_sel;


    for (genvar dc_out=0; dc_out<DC_NUM_WAYS; dc_out++) begin: wb_dout_way_sel_lp 
        `ifndef RV_DC_NONE_MUL
            rvdff #(DC_TAG_HIGH-2 ) dc_rw_adr_ff (.*, .din ({dc_rw_addr[DC_TAG_HIGH-1:2] }), .dout({dc_rw_addr_ff[DC_TAG_HIGH-1:2] }));
            assign wb_dout_way[dc_out][135:0] = wb_dout[dc_out][dc_rw_addr_ff[DC_TAG_HIGH-1: DC_TAG_MUL_LOW]][135:0]; 
        `else
            assign wb_dout_way[dc_out][135:0] = wb_dout[dc_out][135:0];          
        `endif
        assign wb_dout_way_sel[dc_out][135:0]  = ({136{dc_rd_hit_ff[dc_out]}} &  wb_dout_way[dc_out][135:0]);
    end
    
    always_comb begin
      dc_rd_data = '0;
      for (int dc_or_out=0; dc_or_out<DC_NUM_WAYS; dc_or_out++) begin
         dc_rd_data[135:0] |= wb_dout_way_sel[dc_or_out][135:0];
      end
   end
    
endmodule 













