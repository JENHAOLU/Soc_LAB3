//  `include "../../bram/bram11.v"

`timescale 1ns / 1ps

module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //Read address channel(RA)
    output  wire                     arready,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     arvalid, 
    //Read data channel(RD)
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    input   wire                     rready,
    //Write address channel(WA)
    output  wire                     awready,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     awvalid,
    //Write data channel(WD)
    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,


    //streaming
    output  wire                     ss_tready,
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    input   wire                     sm_tready, 

    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

reg [(pDATA_WIDTH-1):0] multi_result;

// bram11 bram11_axilite(
//     .CLK        (axis_clk),
//     .WE         (tap_WE),
//     .EN         (tap_EN),
//     .Di         (tap_Di),
//     .Do         (tap_Do),
//     .A          (tap_A)
// );

// bram11 bram11_axistream(
//     .CLK        (axis_clk),
//     .WE         (data_WE),
//     .EN         (data_EN),
//     .Di         (data_Di),
//     .Do         (data_Do),
//     .A          (data_A)
// );


/******************FSM*********************/
parameter                   IDLE = 2'b00;
parameter                   COEF_IN = 2'b01;
parameter                   COMPUTE = 2'b10;
parameter                   DONE = 2'b11;
reg     [1:0]               next_state;
reg     [1:0]               current_state;

/******************AXI LITE*********************/
reg                         write_counter;  // let awready follows awvalid's second clock and pulls up to 1, 
                                            // then pulls down to 0, wready follows wvalid's second clock and pulls up to 1, 
                                            // then pulls down to 0
reg [1:0]                   read_counter;
reg [5:0]                   tap_A_counter;                                         
reg                         awready_reg;    // write address control : if awready is 1, tap_A = awaddr
reg                         wready_reg;

reg [(pADDR_WIDTH-1):0]     tap_A_reg;
reg [(pDATA_WIDTH-1):0]     tap_Di_reg;
reg [3:0]                   tap_WE_reg;

reg                         arready_reg;
reg                         rvalid_reg;

/******************AXI Streaming*********************/
reg     [(pDATA_WIDTH-1):0] data_Di_reg;
reg     [(pADDR_WIDTH-1):0] data_A_reg;
reg     [3:0]               data_WE_reg;

reg                         computing;
reg     [(pADDR_WIDTH-1):0] compute_counter;
reg     [3:0]               pattern_cycle;
reg     [9:0]               pattern_number;

reg    [(pDATA_WIDTH-1):0]  stream_data_1;
reg    [(pDATA_WIDTH-1):0]  stream_data_2;
reg    [(pDATA_WIDTH-1):0]  Coef_data;
reg    [(pDATA_WIDTH-1):0]  Xn;
reg    [(pDATA_WIDTH-1):0]  Xn_multi_Coef;
reg    [(pDATA_WIDTH-1):0]  Yn;
wire                        one_pattern_done;


/***************************************************************CIRCUIT***************************************************************/


/***********************************************/
/******************TAP BRAM*********************/
/***********************************************/
assign tap_EN   = 1'b1;
assign tap_A    = tap_A_reg;
assign tap_Di   = tap_Di_reg;
assign tap_WE   = tap_WE_reg;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        tap_A_counter <= 6'b000000;
    else if (current_state == COEF_IN)
        tap_A_counter <= 0;
    else if (current_state == COMPUTE) begin
        if (data_WE == 4'b1111)
            tap_A_counter <= 6'b000000;
       
        else if (ss_tready == 1)
            tap_A_counter <= 6'b000000;
        else
            tap_A_counter <= tap_A_counter + 1'b1;
    end
    else
        tap_A_counter <= tap_A_counter;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        tap_A_reg <= 12'h000;
    else if (awvalid == 1 && awready == 1 && current_state == COEF_IN) begin
        if (awaddr >= 12'h020)
            tap_A_reg <= awaddr - 12'h020;
        else if (awaddr == 12'h010)
            tap_A_reg <= awaddr;
        else
            tap_A_reg <= 12'h000;
    end
    else if (arvalid == 1 && current_state == COEF_IN) begin
        if (araddr >= 12'h020)
            tap_A_reg <= araddr - 12'h020;
        else if (araddr == 12'h000)
            tap_A_reg <= 12'h000;
        else
            tap_A_reg <= tap_A_reg;
    end
    else if (current_state == COMPUTE) begin    
        case (tap_A_counter)
            6'd0: tap_A_reg <= 12'h000;
            6'd1: tap_A_reg <= 12'h004;
            6'd2: tap_A_reg <= 12'h008;
            6'd3: tap_A_reg <= 12'h00C;
            6'd4: tap_A_reg <= 12'h010;
            6'd5: tap_A_reg <= 12'h014;
            6'd6: tap_A_reg <= 12'h018;
            6'd7: tap_A_reg <= 12'h01C;
            6'd8: tap_A_reg <= 12'h020;
            6'd9: tap_A_reg <= 12'h024;
            6'd10: tap_A_reg <= 12'h028;
            default: tap_A_reg <= tap_A_reg;
        endcase
    end
    else
        tap_A_reg <= tap_A_reg;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        tap_Di_reg <= 32'h0000_0000;
    else if (wvalid && wready == 1)
        tap_Di_reg <= wdata;
    else
        tap_Di_reg <= 32'h0000_0000;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        tap_WE_reg <= 4'b0000;
    else if ((wvalid && wready == 1) && (awaddr != 12'h000))
        tap_WE_reg <= 4'b1111;
    else
        tap_WE_reg <= 4'b0000;
end



/***********************************************/
/******************AXI LITE*********************/
/***********************************************/

assign rdata = (current_state == IDLE) ? (rvalid ? tap_Do : 32'd4) :(current_state == COEF_IN) ?(rvalid ? tap_Do : 32'd4) :
              (current_state == COMPUTE) ? (rvalid ? tap_Do : 32'd4) :
              (current_state == DONE ) ? 32'd2 : 32'd4;//



always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        write_counter <= 1'b0;
    end else if(awaddr != 12'h000) begin
        case ({write_counter, awvalid})
            2'b01: write_counter <= 1'b1;
            2'b10: write_counter <= 1'b0;
            default: write_counter <= write_counter;
        endcase
    end else begin
        write_counter <= 1'b0;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        read_counter <= 1'b0;
    end else if (arvalid) begin
        case (read_counter)
            2'b00: read_counter <= 2'b01;
            2'b01: read_counter <= 2'b10;
            2'b10: read_counter <= 2'b00;
            default: read_counter <= read_counter;
        endcase
    end else begin
        read_counter <= read_counter;
    end
end

assign awready = (write_counter)? 1'b1 : 1'b0;
assign wready = awready;

assign arready = (read_counter == 2'b01) ? 1'b1 : 1'b0;


assign rvalid = rvalid_reg;
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        rvalid_reg <= 1'b0;
    end 
   
    else begin
        rvalid_reg <= arready;
    end
end



/******************************************/
/******************FSM*********************/
/******************************************/
always @(posedge axis_clk or negedge axis_rst_n ) begin
    if ((~axis_rst_n) ) begin //重置
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

always @(*) begin
    case (current_state)
        IDLE: begin
            if (wdata == 32'd600  ) begin
                next_state = COEF_IN;
            end else begin
                next_state = IDLE;
            end
        end
        COEF_IN: begin
            if (awaddr == 12'h000 && wdata == 32'd1) begin
                next_state = COMPUTE;
            end else begin
                next_state = COEF_IN;
            end
        end
        COMPUTE: begin
            if (pattern_number == 600 && sm_tready && sm_tvalid) begin
                next_state = DONE;
            end else begin
                next_state = COMPUTE;
            end
        end
        DONE: begin
            if (rdata == 32'd2 ) begin
                next_state = IDLE;
            end else begin
                next_state = DONE;
            end
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end


/************************************************/
/******************DATA BRAM*********************/
/************************************************/
assign data_Di = data_Di_reg;
assign data_A = data_A_reg;
assign data_EN = 1'b1;
assign data_WE = data_WE_reg;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n) begin
        data_WE_reg <= 4'b0000;
    end else if (wvalid && wready && awaddr != 12'h000) begin
        data_WE_reg <= 4'b1111;
    end else if (current_state == COMPUTE && ss_tready) begin
        data_WE_reg <= 4'b1111;
    end else begin
        data_WE_reg <= 4'b0000;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        data_Di_reg <= 32'b0;
    end 
    else if (current_state == COEF_IN) begin
        data_Di_reg <= 32'b0;
    end 
    else if (current_state == COMPUTE) begin
        data_Di_reg <= ss_tdata;
    end
    else if (current_state == DONE) begin
        data_Di_reg <= 32'b0;
    end 
    else begin
        data_Di_reg <= data_Di_reg;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        data_A_reg <= 12'h000;
    end else if (current_state == COEF_IN) begin
        if (awvalid && awready) begin
            if (awaddr != 12'h010) begin
                data_A_reg <= awaddr - 12'h020;
            end else if (awaddr == 12'h010) begin
                data_A_reg <= 12'h000;
            end else begin
                data_A_reg <= data_A_reg;
            end
        end else if (arvalid) begin
            data_A_reg <= araddr - 12'h020;
        end else begin
            data_A_reg <= data_A_reg;
        end
    end else if (current_state == COMPUTE) begin
        if (ss_tready && pattern_cycle < 12'd11) begin
            data_A_reg <= pattern_cycle << 2; // write data
        end else if (ss_tready && pattern_cycle == 12'd11) begin
            data_A_reg <= 0; // write data
        end else if (!ss_tready) begin
            if (data_A_reg == 12'h000) begin
                data_A_reg <= 12'h028; // read data
            end else begin
                data_A_reg <= data_A_reg - 12'h004;
            end
        end else begin
            data_A_reg <= data_A_reg;
        end
    end else begin
        data_A_reg <= data_A_reg;
    end
end

/****************************************************/
/******************AXI Streaming*********************/
/****************************************************/
assign ss_tready = ((current_state == COMPUTE && compute_counter == 0) ) ? 1 : 0;

assign sm_tvalid = (pattern_number != 9'd1  ) ? one_pattern_done : 0;

assign sm_tlast = (pattern_number == 600 && one_pattern_done == 1) ? 1 : 0;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        compute_counter <= 4'd0;
    end else if (current_state == COMPUTE) begin
        if (compute_counter <= 4'd10) begin
            compute_counter <= compute_counter + 1'b1;
        end else if (compute_counter == 4'd11) begin
            compute_counter <= 0;
        end
    end else begin
        compute_counter <= compute_counter;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        pattern_cycle <= 4'd0;
    end else if (current_state == COMPUTE) begin
        if (ss_tready == 1 && pattern_cycle < 4'd11) begin
            pattern_cycle <= pattern_cycle + 1;
        end else if (pattern_cycle == 4'd11 && ss_tready == 1) begin
            pattern_cycle <= 1;
        end
    end else begin
        pattern_cycle <= pattern_cycle;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        pattern_number <= 9'd0;
    end else if (current_state == COMPUTE && one_pattern_done == 1) begin
        pattern_number <= pattern_number + 1;
    end else if (pattern_number == 603) begin
        pattern_number <= 0;
    end else begin
        pattern_number <= pattern_number;
    end
end


/****************************************************/
/******************FIR Calculate*********************/
/****************************************************/
assign one_pattern_done = (tap_A_counter == 2|| current_state == DONE )? 1:0;//



assign sm_tdata = Yn;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if ((~axis_rst_n) ) begin
        Coef_data <= 0;
    end 
    else begin
        Coef_data <= (tap_A <= 12'h028) ? tap_Do : Coef_data;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        stream_data_1 <= 0;
    end else begin
        stream_data_1 <= (data_WE == 4'b0000) ? data_Do : (data_WE == 4'b1111) ? 0 : stream_data_1;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        stream_data_2 <= 0;
    end else begin
        stream_data_2 <= (data_EN == 1) ? stream_data_1 : stream_data_2;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        Xn <= 0;
    end else begin
        Xn <= (data_EN == 1) ? stream_data_2 : Xn;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        Xn_multi_Coef <= 0;
    end else begin
        Xn_multi_Coef <= (data_EN == 1) ? Xn * Coef_data : Xn_multi_Coef;
    end
end

always @(posedge axis_clk or negedge axis_rst_n ) begin
    if ((~axis_rst_n)) begin //重置
        Yn <= 0;
    end else if (data_EN == 1 && current_state == COMPUTE) begin
        Yn <= (one_pattern_done == 0) ? Yn + Xn_multi_Coef : (one_pattern_done == 1) ? 0 : Yn;
    end
end






endmodule