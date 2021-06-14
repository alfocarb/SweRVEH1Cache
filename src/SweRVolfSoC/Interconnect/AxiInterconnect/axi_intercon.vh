// THIS FILE IS AUTOGENERATED BY axi_intercon_gen
// ANY MANUAL CHANGES WILL BE LOST
wire  [2:0] ifu_arid;
wire [31:0] ifu_araddr;
wire  [7:0] ifu_arlen;
wire  [2:0] ifu_arsize;
wire  [1:0] ifu_arburst;
wire        ifu_arlock;
wire  [3:0] ifu_arcache;
wire  [2:0] ifu_arprot;
wire  [3:0] ifu_arregion;
wire  [3:0] ifu_arqos;
wire        ifu_arvalid;
wire        ifu_arready;
wire  [2:0] ifu_rid;
wire [63:0] ifu_rdata;
wire  [1:0] ifu_rresp;
wire        ifu_rlast;
wire        ifu_rvalid;
wire        ifu_rready;
wire  [3:0] lsu_awid;
wire [31:0] lsu_awaddr;
wire  [7:0] lsu_awlen;
wire  [2:0] lsu_awsize;
wire  [1:0] lsu_awburst;
wire        lsu_awlock;
wire  [3:0] lsu_awcache;
wire  [2:0] lsu_awprot;
wire  [3:0] lsu_awregion;
wire  [3:0] lsu_awqos;
wire        lsu_awvalid;
wire        lsu_awready;
wire  [3:0] lsu_arid;
wire [31:0] lsu_araddr;
wire  [7:0] lsu_arlen;
wire  [2:0] lsu_arsize;
wire  [1:0] lsu_arburst;
wire        lsu_arlock;
wire  [3:0] lsu_arcache;
wire  [2:0] lsu_arprot;
wire  [3:0] lsu_arregion;
wire  [3:0] lsu_arqos;
wire        lsu_arvalid;
wire        lsu_arready;
wire [63:0] lsu_wdata;
wire  [7:0] lsu_wstrb;
wire        lsu_wlast;
wire        lsu_wvalid;
wire        lsu_wready;
wire  [3:0] lsu_bid;
wire  [1:0] lsu_bresp;
wire        lsu_bvalid;
wire        lsu_bready;
wire  [3:0] lsu_rid;
wire [63:0] lsu_rdata;
wire  [1:0] lsu_rresp;
wire        lsu_rlast;
wire        lsu_rvalid;
wire        lsu_rready;
`ifdef RV_DC_ENABLE
    wire  [3:0] lsu_dc_awid;
    wire [31:0] lsu_dc_awaddr;
    wire  [7:0] lsu_dc_awlen;
    wire  [2:0] lsu_dc_awsize;
    wire  [1:0] lsu_dc_awburst;
    wire        lsu_dc_awlock;
    wire  [3:0] lsu_dc_awcache;
    wire  [2:0] lsu_dc_awprot;
    wire  [3:0] lsu_dc_awregion;
    wire  [3:0] lsu_dc_awqos;
    wire        lsu_dc_awvalid;
    wire        lsu_dc_awready;
    wire  [3:0] lsu_dc_arid;
    wire [31:0] lsu_dc_araddr;
    wire  [7:0] lsu_dc_arlen;
    wire  [2:0] lsu_dc_arsize;
    wire  [1:0] lsu_dc_arburst;
    wire        lsu_dc_arlock;
    wire  [3:0] lsu_dc_arcache;
    wire  [2:0] lsu_dc_arprot;
    wire  [3:0] lsu_dc_arregion;
    wire  [3:0] lsu_dc_arqos;
    wire        lsu_dc_arvalid;
    wire        lsu_dc_arready;
    wire [63:0] lsu_dc_wdata;
    wire  [7:0] lsu_dc_wstrb;
    wire        lsu_dc_wlast;
    wire        lsu_dc_wvalid;
    wire        lsu_dc_wready;
    wire  [3:0] lsu_dc_bid;
    wire  [1:0] lsu_dc_bresp;
    wire        lsu_dc_bvalid;
    wire        lsu_dc_bready;
    wire  [3:0] lsu_dc_rid;
    wire [63:0] lsu_dc_rdata;
    wire  [1:0] lsu_dc_rresp;
    wire        lsu_dc_rlast;
    wire        lsu_dc_rvalid;
    wire        lsu_dc_rready;
`endif


wire  [0:0] sb_awid;
wire [31:0] sb_awaddr;
wire  [7:0] sb_awlen;
wire  [2:0] sb_awsize;
wire  [1:0] sb_awburst;
wire        sb_awlock;
wire  [3:0] sb_awcache;
wire  [2:0] sb_awprot;
wire  [3:0] sb_awregion;
wire  [3:0] sb_awqos;
wire        sb_awvalid;
wire        sb_awready;
wire  [0:0] sb_arid;
wire [31:0] sb_araddr;
wire  [7:0] sb_arlen;
wire  [2:0] sb_arsize;
wire  [1:0] sb_arburst;
wire        sb_arlock;
wire  [3:0] sb_arcache;
wire  [2:0] sb_arprot;
wire  [3:0] sb_arregion;
wire  [3:0] sb_arqos;
wire        sb_arvalid;
wire        sb_arready;
wire [63:0] sb_wdata;
wire  [7:0] sb_wstrb;
wire        sb_wlast;
wire        sb_wvalid;
wire        sb_wready;
wire  [0:0] sb_bid;
wire  [1:0] sb_bresp;
wire        sb_bvalid;
wire        sb_bready;
wire  [0:0] sb_rid;
wire [63:0] sb_rdata;
wire  [1:0] sb_rresp;
wire        sb_rlast;
wire        sb_rvalid;
wire        sb_rready;
wire  [5:0] io_awid;
wire [31:0] io_awaddr;
wire  [7:0] io_awlen;
wire  [2:0] io_awsize;
wire  [1:0] io_awburst;
wire        io_awlock;
wire  [3:0] io_awcache;
wire  [2:0] io_awprot;
wire  [3:0] io_awregion;
wire  [3:0] io_awqos;
wire        io_awvalid;
wire        io_awready;
wire  [5:0] io_arid;
wire [31:0] io_araddr;
wire  [7:0] io_arlen;
wire  [2:0] io_arsize;
wire  [1:0] io_arburst;
wire        io_arlock;
wire  [3:0] io_arcache;
wire  [2:0] io_arprot;
wire  [3:0] io_arregion;
wire  [3:0] io_arqos;
wire        io_arvalid;
wire        io_arready;
wire [63:0] io_wdata;
wire  [7:0] io_wstrb;
wire        io_wlast;
wire        io_wvalid;
wire        io_wready;
wire  [5:0] io_bid;
wire  [1:0] io_bresp;
wire        io_bvalid;
wire        io_bready;
wire  [5:0] io_rid;
wire [63:0] io_rdata;
wire  [1:0] io_rresp;
wire        io_rlast;
wire        io_rvalid;
wire        io_rready;
wire  [5:0] ram_awid;
wire [31:0] ram_awaddr;
wire  [7:0] ram_awlen;
wire  [2:0] ram_awsize;
wire  [1:0] ram_awburst;
wire        ram_awlock;
wire  [3:0] ram_awcache;
wire  [2:0] ram_awprot;
wire  [3:0] ram_awregion;
wire  [3:0] ram_awqos;
wire        ram_awvalid;
wire        ram_awready;
wire  [5:0] ram_arid;
wire [31:0] ram_araddr;
wire  [7:0] ram_arlen;
wire  [2:0] ram_arsize;
wire  [1:0] ram_arburst;
wire        ram_arlock;
wire  [3:0] ram_arcache;
wire  [2:0] ram_arprot;
wire  [3:0] ram_arregion;
wire  [3:0] ram_arqos;
wire        ram_arvalid;
wire        ram_arready;
wire [63:0] ram_wdata;
wire  [7:0] ram_wstrb;
wire        ram_wlast;
wire        ram_wvalid;
wire        ram_wready;
wire  [5:0] ram_bid;
wire  [1:0] ram_bresp;
wire        ram_bvalid;
wire        ram_bready;
wire  [5:0] ram_rid;
wire [63:0] ram_rdata;
wire  [1:0] ram_rresp;
wire        ram_rlast;
wire        ram_rvalid;
wire        ram_rready;

axi_intercon axi_intercon
   (.clk_i          (clk),
    .rst_ni         (rst_n),
    .i_ifu_arid     (ifu_arid),
    .i_ifu_araddr   (ifu_araddr),
    .i_ifu_arlen    (ifu_arlen),
    .i_ifu_arsize   (ifu_arsize),
    .i_ifu_arburst  (ifu_arburst),
    .i_ifu_arlock   (ifu_arlock),
    .i_ifu_arcache  (ifu_arcache),
    .i_ifu_arprot   (ifu_arprot),
    .i_ifu_arregion (ifu_arregion),
    .i_ifu_arqos    (ifu_arqos),
    .i_ifu_arvalid  (ifu_arvalid),
    .o_ifu_arready  (ifu_arready),
    .o_ifu_rid      (ifu_rid),
    .o_ifu_rdata    (ifu_rdata),
    .o_ifu_rresp    (ifu_rresp),
    .o_ifu_rlast    (ifu_rlast),
    .o_ifu_rvalid   (ifu_rvalid),
    .i_ifu_rready   (ifu_rready),
    .i_lsu_awid     (lsu_awid),
    .i_lsu_awaddr   (lsu_awaddr),
    .i_lsu_awlen    (lsu_awlen),
    .i_lsu_awsize   (lsu_awsize),
    .i_lsu_awburst  (lsu_awburst),
    .i_lsu_awlock   (lsu_awlock),
    .i_lsu_awcache  (lsu_awcache),
    .i_lsu_awprot   (lsu_awprot),
    .i_lsu_awregion (lsu_awregion),
    .i_lsu_awqos    (lsu_awqos),
    .i_lsu_awvalid  (lsu_awvalid),
    .o_lsu_awready  (lsu_awready),
    .i_lsu_arid     (lsu_arid),
    .i_lsu_araddr   (lsu_araddr),
    .i_lsu_arlen    (lsu_arlen),
    .i_lsu_arsize   (lsu_arsize),
    .i_lsu_arburst  (lsu_arburst),
    .i_lsu_arlock   (lsu_arlock),
    .i_lsu_arcache  (lsu_arcache),
    .i_lsu_arprot   (lsu_arprot),
    .i_lsu_arregion (lsu_arregion),
    .i_lsu_arqos    (lsu_arqos),
    .i_lsu_arvalid  (lsu_arvalid),
    .o_lsu_arready  (lsu_arready),
    .i_lsu_wdata    (lsu_wdata),
    .i_lsu_wstrb    (lsu_wstrb),
    .i_lsu_wlast    (lsu_wlast),
    .i_lsu_wvalid   (lsu_wvalid),
    .o_lsu_wready   (lsu_wready),
    .o_lsu_bid      (lsu_bid),
    .o_lsu_bresp    (lsu_bresp),
    .o_lsu_bvalid   (lsu_bvalid),
    .i_lsu_bready   (lsu_bready),
    .o_lsu_rid      (lsu_rid),
    .o_lsu_rdata    (lsu_rdata),
    .o_lsu_rresp    (lsu_rresp),
    .o_lsu_rlast    (lsu_rlast),
    .o_lsu_rvalid   (lsu_rvalid),
    .i_lsu_rready   (lsu_rready),
    
    
`ifdef RV_DC_ENABLE
    .i_lsu_dc_awid     (lsu_dc_awid),
    .i_lsu_dc_awaddr   (lsu_dc_awaddr),
    .i_lsu_dc_awlen    (lsu_dc_awlen),
    .i_lsu_dc_awsize   (lsu_dc_awsize),
    .i_lsu_dc_awburst  (lsu_dc_awburst),
    .i_lsu_dc_awlock   (lsu_dc_awlock),
    .i_lsu_dc_awcache  (lsu_dc_awcache),
    .i_lsu_dc_awprot   (lsu_dc_awprot),
    .i_lsu_dc_awregion (lsu_dc_awregion),
    .i_lsu_dc_awqos    (lsu_dc_awqos),
    .i_lsu_dc_awvalid  (lsu_dc_awvalid),
    .o_lsu_dc_awready  (lsu_dc_awready),
    .i_lsu_dc_arid     (lsu_dc_arid),
    .i_lsu_dc_araddr   (lsu_dc_araddr),
    .i_lsu_dc_arlen    (lsu_dc_arlen),
    .i_lsu_dc_arsize   (lsu_dc_arsize),
    .i_lsu_dc_arburst  (lsu_dc_arburst),
    .i_lsu_dc_arlock   (lsu_dc_arlock),
    .i_lsu_dc_arcache  (lsu_dc_arcache),
    .i_lsu_dc_arprot   (lsu_dc_arprot),
    .i_lsu_dc_arregion (lsu_dc_arregion),
    .i_lsu_dc_arqos    (lsu_dc_arqos),
    .i_lsu_dc_arvalid  (lsu_dc_arvalid),
    .o_lsu_dc_arready  (lsu_dc_arready),
    .i_lsu_dc_wdata    (lsu_dc_wdata),
    .i_lsu_dc_wstrb    (lsu_dc_wstrb),
    .i_lsu_dc_wlast    (lsu_dc_wlast),
    .i_lsu_dc_wvalid   (lsu_dc_wvalid),
    .o_lsu_dc_wready   (lsu_dc_wready),
    .o_lsu_dc_bid      (lsu_dc_bid),
    .o_lsu_dc_bresp    (lsu_dc_bresp),
    .o_lsu_dc_bvalid   (lsu_dc_bvalid),
    .i_lsu_dc_bready   (lsu_dc_bready),
    .o_lsu_dc_rid      (lsu_dc_rid),
    .o_lsu_dc_rdata    (lsu_dc_rdata),
    .o_lsu_dc_rresp    (lsu_dc_rresp),
    .o_lsu_dc_rlast    (lsu_dc_rlast),
    .o_lsu_dc_rvalid   (lsu_dc_rvalid),
    .i_lsu_dc_rready   (lsu_dc_rready),
`endif
  
    .i_sb_awid      (sb_awid),
    .i_sb_awaddr    (sb_awaddr),
    .i_sb_awlen     (sb_awlen),
    .i_sb_awsize    (sb_awsize),
    .i_sb_awburst   (sb_awburst),
    .i_sb_awlock    (sb_awlock),
    .i_sb_awcache   (sb_awcache),
    .i_sb_awprot    (sb_awprot),
    .i_sb_awregion  (sb_awregion),
    .i_sb_awqos     (sb_awqos),
    .i_sb_awvalid   (sb_awvalid),
    .o_sb_awready   (sb_awready),
    .i_sb_arid      (sb_arid),
    .i_sb_araddr    (sb_araddr),
    .i_sb_arlen     (sb_arlen),
    .i_sb_arsize    (sb_arsize),
    .i_sb_arburst   (sb_arburst),
    .i_sb_arlock    (sb_arlock),
    .i_sb_arcache   (sb_arcache),
    .i_sb_arprot    (sb_arprot),
    .i_sb_arregion  (sb_arregion),
    .i_sb_arqos     (sb_arqos),
    .i_sb_arvalid   (sb_arvalid),
    .o_sb_arready   (sb_arready),
    .i_sb_wdata     (sb_wdata),
    .i_sb_wstrb     (sb_wstrb),
    .i_sb_wlast     (sb_wlast),
    .i_sb_wvalid    (sb_wvalid),
    .o_sb_wready    (sb_wready),
    .o_sb_bid       (sb_bid),
    .o_sb_bresp     (sb_bresp),
    .o_sb_bvalid    (sb_bvalid),
    .i_sb_bready    (sb_bready),
    .o_sb_rid       (sb_rid),
    .o_sb_rdata     (sb_rdata),
    .o_sb_rresp     (sb_rresp),
    .o_sb_rlast     (sb_rlast),
    .o_sb_rvalid    (sb_rvalid),
    .i_sb_rready    (sb_rready),
    .o_io_awid      (io_awid),
    .o_io_awaddr    (io_awaddr),
    .o_io_awlen     (io_awlen),
    .o_io_awsize    (io_awsize),
    .o_io_awburst   (io_awburst),
    .o_io_awlock    (io_awlock),
    .o_io_awcache   (io_awcache),
    .o_io_awprot    (io_awprot),
    .o_io_awregion  (io_awregion),
    .o_io_awqos     (io_awqos),
    .o_io_awvalid   (io_awvalid),
    .i_io_awready   (io_awready),
    .o_io_arid      (io_arid),
    .o_io_araddr    (io_araddr),
    .o_io_arlen     (io_arlen),
    .o_io_arsize    (io_arsize),
    .o_io_arburst   (io_arburst),
    .o_io_arlock    (io_arlock),
    .o_io_arcache   (io_arcache),
    .o_io_arprot    (io_arprot),
    .o_io_arregion  (io_arregion),
    .o_io_arqos     (io_arqos),
    .o_io_arvalid   (io_arvalid),
    .i_io_arready   (io_arready),
    .o_io_wdata     (io_wdata),
    .o_io_wstrb     (io_wstrb),
    .o_io_wlast     (io_wlast),
    .o_io_wvalid    (io_wvalid),
    .i_io_wready    (io_wready),
    .i_io_bid       (io_bid),
    .i_io_bresp     (io_bresp),
    .i_io_bvalid    (io_bvalid),
    .o_io_bready    (io_bready),
    .i_io_rid       (io_rid),
    .i_io_rdata     (io_rdata),
    .i_io_rresp     (io_rresp),
    .i_io_rlast     (io_rlast),
    .i_io_rvalid    (io_rvalid),
    .o_io_rready    (io_rready),
    .o_ram_awid     (ram_awid),
    .o_ram_awaddr   (ram_awaddr),
    .o_ram_awlen    (ram_awlen),
    .o_ram_awsize   (ram_awsize),
    .o_ram_awburst  (ram_awburst),
    .o_ram_awlock   (ram_awlock),
    .o_ram_awcache  (ram_awcache),
    .o_ram_awprot   (ram_awprot),
    .o_ram_awregion (ram_awregion),
    .o_ram_awqos    (ram_awqos),
    .o_ram_awvalid  (ram_awvalid),
    .i_ram_awready  (ram_awready),
    .o_ram_arid     (ram_arid),
    .o_ram_araddr   (ram_araddr),
    .o_ram_arlen    (ram_arlen),
    .o_ram_arsize   (ram_arsize),
    .o_ram_arburst  (ram_arburst),
    .o_ram_arlock   (ram_arlock),
    .o_ram_arcache  (ram_arcache),
    .o_ram_arprot   (ram_arprot),
    .o_ram_arregion (ram_arregion),
    .o_ram_arqos    (ram_arqos),
    .o_ram_arvalid  (ram_arvalid),
    .i_ram_arready  (ram_arready),
    .o_ram_wdata    (ram_wdata),
    .o_ram_wstrb    (ram_wstrb),
    .o_ram_wlast    (ram_wlast),
    .o_ram_wvalid   (ram_wvalid),
    .i_ram_wready   (ram_wready),
    .i_ram_bid      (ram_bid),
    .i_ram_bresp    (ram_bresp),
    .i_ram_bvalid   (ram_bvalid),
    .o_ram_bready   (ram_bready),
    .i_ram_rid      (ram_rid),
    .i_ram_rdata    (ram_rdata),
    .i_ram_rresp    (ram_rresp),
    .i_ram_rlast    (ram_rlast),
    .i_ram_rvalid   (ram_rvalid),
    .o_ram_rready   (ram_rready));
