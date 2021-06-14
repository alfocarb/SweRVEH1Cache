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

localparam TOTAL_INT        = `RV_PIC_TOTAL_INT_PLUS1;

localparam DCCM_BITS        = `RV_DCCM_BITS;
localparam DCCM_BANK_BITS   = `RV_DCCM_BANK_BITS;
localparam DCCM_NUM_BANKS   = `RV_DCCM_NUM_BANKS;
localparam DCCM_DATA_WIDTH  = `RV_DCCM_DATA_WIDTH;
localparam DCCM_FDATA_WIDTH = `RV_DCCM_FDATA_WIDTH;
localparam DCCM_BYTE_WIDTH  = `RV_DCCM_BYTE_WIDTH;
localparam DCCM_ECC_WIDTH   = `RV_DCCM_ECC_WIDTH;

localparam LSU_RDBUF_DEPTH  = `RV_LSU_NUM_NBLOAD;
localparam DMA_BUF_DEPTH    = `RV_DMA_BUF_DEPTH;
localparam LSU_STBUF_DEPTH  = `RV_LSU_STBUF_DEPTH;
localparam LSU_SB_BITS      = `RV_LSU_SB_BITS;

localparam DEC_INSTBUF_DEPTH = `RV_DEC_INSTBUF_DEPTH;

localparam ICCM_SIZE         = `RV_ICCM_SIZE;
localparam ICCM_BITS         = `RV_ICCM_BITS;
localparam ICCM_NUM_BANKS    = `RV_ICCM_NUM_BANKS;
localparam ICCM_BANK_BITS    = `RV_ICCM_BANK_BITS;
localparam ICCM_INDEX_BITS   = `RV_ICCM_INDEX_BITS;
localparam ICCM_BANK_HI      = 4 + (`RV_ICCM_BANK_BITS/4);

localparam ICACHE_TAG_HIGH  = `RV_ICACHE_TAG_HIGH;
localparam ICACHE_TAG_LOW   = `RV_ICACHE_TAG_LOW;
localparam ICACHE_IC_DEPTH  = `RV_ICACHE_IC_DEPTH;
localparam ICACHE_TAG_DEPTH = `RV_ICACHE_TAG_DEPTH;

localparam LSU_BUS_TAG     = `RV_LSU_BUS_TAG;
localparam DMA_BUS_TAG     = `RV_DMA_BUS_TAG;
localparam SB_BUS_TAG      = `RV_SB_BUS_TAG;

localparam IFU_BUS_TAG     = `RV_IFU_BUS_TAG;

//DATA CACHE

/*   Tag          C     Bank Offset     Bank      Byte offset
 * [31:12]    [11:6]     [5:4]              [3:2]       [1:0]        ===     Para memoria con depth 256 y bloques 64 bits. [n vias]
 */

localparam DC_BLOCK_SIZE = `RV_DC_BLOCK_SIZE; // in words.
`ifdef RV_DC_NONE_MUL
    localparam DC_DEPTH = 256;
`else
    localparam DC_DEPTH = `RV_DC_DEPTH;
`endif 
`ifdef RV_DC_DIRECT_EMPLACEMENT
    localparam DC_NUM_WAYS = 1;
`else
    localparam DC_NUM_WAYS = `RV_DC_NUM_WAYS;
`endif   
localparam NUM_BYTES_WORD = `RV_NUM_BYTES_WORD;
localparam LSU_DC_BUS_TAG     = `RV_LSU_DC_BUS_TAG; //4
localparam DC_NUM_SUBBANKS = `RV_DC_NUM_SUBBANKS ;
localparam DC_ST_BUF_DEPTH = `RV_DC_ST_BUF_DEPTH;
localparam DC_REQ_BUF_DEPTH = `RV_DC_REQ_BUF_DEPTH;

localparam DC_NUM_LINES_BLOCK = `RV_DC_NUM_LINES_BLOCK;
localparam DC_MUL_DEPTH = `RV_DC_MUL_DEPTH;
localparam DC_LOG2_MUL_DEPTH = `RV_DC_LOG2_MUL_DEPTH;
localparam DC_NUM_SET_DEPTH = `RV_DC_NUM_SET_DEPTH;


localparam DC_REQ_LOG2_BUF_DEPTH = `RV_DC_REQ_LOG2_BUF_DEPTH;

localparam DC_TAG_DEPTH = `RV_DC_TAG_DEPTH;
localparam DC_TAG_LOW   = `RV_DC_TAG_LOW;
localparam DC_TAG_HIGH  = `RV_DC_TAG_HIGH;
localparam DC_TAG_LEN = `RV_DC_TAG_LEN;
localparam DC_TAG_MUL_LOW = `RV_DC_TAG_MUL_LOW;

localparam DC_BANKOFF_HIGH  = `RV_DC_BANKOFF_HIGH;
localparam DC_BANKOFF_LOW   = `RV_DC_BANKOFF_LOW;

localparam DC_ST_LOG2_BUF_DEPTH = `RV_DC_ST_LOG2_BUF_DEPTH;

localparam  DC_FULL_WORD_SIZE = DC_DEPTH*DC_NUM_SUBBANKS;




