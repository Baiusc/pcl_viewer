// `include "ini_register_list.v"
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: VITAL_AHGZ_CDJP_LIDAR_CENTER
// Engineer: CJR
//
// Create Date: 2023/05/07
// Design Name:
// Module Name: imx459_initial
// Project Name:Alpha_Lidar_FPGA_V1.0
// Target Devices: ZU3
// Tool Versions: vivado2020.2
// Description: Imx459 initialization process
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module imx459_initial (
    input  wire                         clk                        ,
    input  wire                         rst_n                      ,
    //imx459 标志引脚
    input  wire                         i_READY                    ,
    input  wire                         i_COMREADY                 ,
    //vop负压建立完成
    input  wire                         i_vop_power_good           ,
    //读写控制器返回信号&数据
    input  wire                         i_read_ask_done            ,
    input  wire        [  31:0]         iv_read_data               ,
    input  wire                         i_write_ask_done           ,
    //输出给读写控制器
    output reg                          o_wr_req                   ,
    output reg                          o_rd_req                   ,
    output reg         [  31:0]         ov_wr_data                 ,
    output reg         [  15:0]         ov_rd_addr                 ,
    output reg         [  15:0]         ov_wr_addr                 ,
    output reg         [   2:0]         ov_rd_bytes                ,
    output reg         [   2:0]         ov_wr_bytes                ,
    //初始化完成标志
    output reg                          o_imx459_ini_done          ,
    //imx459复位
    output reg                          o_XCLR                     ,
    //vop负压使能
    output reg                          o_vop_power_en
);

    parameter                           XCLR_LOW   = 24'd200_0000  ;
    parameter                           WR_SB_WA   = 20'd1_000_000 ;
    parameter                           CMD_SW_WA  = 19'd200_000   ;
    parameter                           WR_ACT_WA  = 20'd1_000_000 ;

    parameter                           CMD_NUM    = 'd148;
//459复位脚
reg                    [  23:0]         rv_xclr_low_cnt            ;
always @(posedge clk or negedge rst_n)begin
    if(~rst_n)begin
        rv_xclr_low_cnt <= 0;
    end
    else if(rv_xclr_low_cnt >= XCLR_LOW - 1 )begin
        rv_xclr_low_cnt <= XCLR_LOW;
    end
    else begin
        rv_xclr_low_cnt <= rv_xclr_low_cnt + 1;
    end
end

always @(posedge clk or negedge rst_n)begin
    if(~rst_n)begin
        o_XCLR <= 0;                                                //上电保持为0
    end
    else if(rv_xclr_low_cnt >= XCLR_LOW - 1 )begin
        o_XCLR <= 1;                                                //复位完成拉高
    end
    else begin
        o_XCLR <= 0;
    end
end

reg                    [   1:0]         i_vop_power_good_fifo      ;
wire                                    pos_i_vop_power_good = (i_vop_power_good_fifo[1:0] == 2'b01);
always@(posedge clk or negedge rst_n)begin
   if(~rst_n)begin
        i_vop_power_good_fifo <= 2'b0;
    end
    else begin
        i_vop_power_good_fifo <= {i_vop_power_good_fifo[0],i_vop_power_good};
    end
end

//初始化流程控制状态机
localparam IDLE             = 1 ,
           wait_initial1    = 2 ,                                   //pos_comready
           wr_standby       = 3 ,                                   //wr_ask_done
           wr_standby_wait  = 4 ,                                   //cnt = xx
           rd_standby       = 5 ,                                   //rd_done
           rd_standby_check = 6 ,                                   //rd_data = xx
           wr_cmd           = 7 ,                                   //wr_ask_done
           wr_cmd_switch    = 8 ,                                   //1
           vop_en           = 9 ,                                   //vop_power_good,
           wr_active        = 10,                                   //wr_ask_done
           wr_active_wait   = 11,                                   //cnt = xx
           rd_active        = 12,                                   //rd_done
           rd_active_check  = 13,                                   //rd_data = xx
           ready_check      = 14,                                   //READY
           ini_done         = 15;

wire                                    idle2wait_ini              ;
wire                                    wait_ini2wr_sb             ;
wire                                    wr_sb2wr_sb_w              ;
wire                                    wr_sb_w2rd_sb              ;
wire                                    rd_sb2rd_sb_c              ;
wire                                    rd_sb_c2wr_cmd_s           ;
wire                                    wr_cmd2wr_cmd_s            ;
wire                                    wr_cmd2vop_en              ;
wire                                    wr_cmd_s2wr_cmd            ;
wire                                    vop_en2wr_a                ;
wire                                    wr_a2wr_a_w                ;
wire                                    wr_a_w2rd_a                ;
wire                                    rd_a2rd_a_c                ;
wire                                    rd_a_c2ready_c             ;
wire                                    ready_c2ini_done           ;

reg                    [   8:0]         rv_cmd_num                 ;


reg                    [   4:0]         ini_state_c                ;
reg                    [   4:0]         ini_state_n                ;

reg                    [  19:0]         rv_wr_sb_wa_cnt            ;
always @(posedge clk or negedge rst_n)begin
    if(~rst_n)begin
        rv_wr_sb_wa_cnt <= 0;
    end
    else if(rv_wr_sb_wa_cnt >= WR_SB_WA - 1)begin
        rv_wr_sb_wa_cnt <= WR_SB_WA;
    end
    else if(ini_state_c == wr_standby_wait)begin
        rv_wr_sb_wa_cnt <= rv_wr_sb_wa_cnt + 1;
    end
    else begin
        rv_wr_sb_wa_cnt <= rv_wr_sb_wa_cnt;
    end
end

reg                    [  18:0]         rv_cmd_sw_wa_cnt              ;
always @(posedge clk or negedge rst_n)begin
    if(~rst_n)begin
        rv_cmd_sw_wa_cnt <= 0;
    end
    else if(rv_cmd_sw_wa_cnt >= CMD_SW_WA - 1)begin
        rv_cmd_sw_wa_cnt <= 0;
    end
    else if(ini_state_c == wr_cmd_switch)begin
        rv_cmd_sw_wa_cnt <= rv_cmd_sw_wa_cnt + 1;
    end
    else begin
        rv_cmd_sw_wa_cnt <= rv_cmd_sw_wa_cnt;
    end
end

reg                    [  19:0]         rv_wr_act_wa_cnt           ;
always @(posedge clk or negedge rst_n)begin
    if(~rst_n)begin
        rv_wr_act_wa_cnt <= 0;
    end
    else if(rv_wr_act_wa_cnt >= WR_ACT_WA - 1)begin
        rv_wr_act_wa_cnt <= WR_ACT_WA;
    end
    else if(ini_state_c == wr_active_wait)begin
        rv_wr_act_wa_cnt <= rv_wr_act_wa_cnt + 1;
    end
    else begin
        rv_wr_act_wa_cnt <= rv_wr_act_wa_cnt;
    end
end

reg [31:0] rv_read_data;
always @(posedge clk or negedge rst_n)begin
    if(~rst_n)begin
        rv_read_data <= 0;
    end
    else if(i_read_ask_done)begin
        rv_read_data <= iv_read_data;
    end
    else begin
        rv_read_data <= rv_read_data;
    end
end



//状态机
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin
                ini_state_c <= IDLE;
            end
            else begin
                ini_state_c <= ini_state_n;
            end
        end

        always @(*)begin
            case(ini_state_c)
                IDLE:begin
                    if(idle2wait_ini)
                        ini_state_n = wait_initial1;
                    else
                        ini_state_n = ini_state_c;
                end
                wait_initial1:begin
                    if(wait_ini2wr_sb)
                        ini_state_n = wr_standby;
                    else
                        ini_state_n = ini_state_c;
                end
                wr_standby:begin
                    if(wr_sb2wr_sb_w)
                        ini_state_n = wr_standby_wait;
                    else
                        ini_state_n = ini_state_c;
                end
                wr_standby_wait:begin
                    if(wr_sb_w2rd_sb)
                        ini_state_n = rd_standby;
                    else
                        ini_state_n = ini_state_c;
                end
                rd_standby:begin
                    if(rd_sb2rd_sb_c)
                        ini_state_n = rd_standby_check;
                    else
                        ini_state_n = ini_state_c;
                end
                rd_standby_check:begin
                    if(rd_sb_c2wr_cmd_s)
                        ini_state_n = wr_cmd_switch;
                    else
                        ini_state_n = ini_state_c;
                end
                wr_cmd:begin
                    if(wr_cmd2vop_en)
                        ini_state_n = vop_en;
                    else if(wr_cmd2wr_cmd_s)
                        ini_state_n = wr_cmd_switch;
                    else
                        ini_state_n = ini_state_c;
                end
                wr_cmd_switch:begin
                    if(wr_cmd_s2wr_cmd)
                        ini_state_n = wr_cmd;
                    else
                        ini_state_n = ini_state_c;
                end
                vop_en:begin
                    if(vop_en2wr_a)
                        ini_state_n = wr_active;
                    else
                        ini_state_n = ini_state_c;
                end
                wr_active:begin
                    if(wr_a2wr_a_w)
                        ini_state_n = wr_active_wait;
                    else
                        ini_state_n = ini_state_c;
                end
                wr_active_wait:begin
                    if(wr_a_w2rd_a)
                        ini_state_n = rd_active;
                    else
                        ini_state_n = ini_state_c;
                end
                rd_active:begin
                    if(rd_a2rd_a_c)
                        ini_state_n = rd_active_check;
                    else
                        ini_state_n = ini_state_c;
                end
                rd_active_check:begin
                    if(rd_a_c2ready_c)
                        ini_state_n = ready_check;
                    else
                        ini_state_n = ini_state_c;
                end
                ready_check:begin
                    if(ready_c2ini_done)
                        ini_state_n = ini_done;
                    else
                        ini_state_n = ini_state_c;
                end
                ini_done:begin
                    ini_state_n = ini_state_c;
                end
                default:ini_state_n = IDLE;
            endcase
        end

    assign idle2wait_ini    = ini_state_c == IDLE             && o_XCLR      ;
    assign wait_ini2wr_sb   = ini_state_c == wait_initial1    && i_COMREADY  ;
    assign wr_sb2wr_sb_w    = ini_state_c == wr_standby       && i_write_ask_done;
    assign wr_sb_w2rd_sb    = ini_state_c == wr_standby_wait  && (rv_wr_sb_wa_cnt == (WR_SB_WA - 1));
    assign rd_sb2rd_sb_c    = ini_state_c == rd_standby       && i_read_ask_done;
    assign rd_sb_c2wr_cmd_s = ini_state_c == rd_standby_check && rv_read_data == 'h01;
    assign wr_cmd2wr_cmd_s  = ini_state_c == wr_cmd           && i_write_ask_done;
    assign wr_cmd2vop_en    = ini_state_c == wr_cmd           && rv_cmd_num == CMD_NUM;
    assign wr_cmd_s2wr_cmd  = ini_state_c == wr_cmd_switch    && (rv_cmd_sw_wa_cnt >= CMD_SW_WA - 1);
    assign vop_en2wr_a      = ini_state_c == vop_en           && pos_i_vop_power_good;
    assign wr_a2wr_a_w      = ini_state_c == wr_active        && i_write_ask_done;
    assign wr_a_w2rd_a      = ini_state_c == wr_active_wait   && (rv_wr_act_wa_cnt >= (WR_ACT_WA - 1));
    assign rd_a2rd_a_c      = ini_state_c == rd_active        && i_read_ask_done;
    assign rd_a_c2ready_c   = ini_state_c == rd_active_check  && rv_read_data == 'h03;
    assign ready_c2ini_done = ini_state_c == ready_check      && i_READY;

    //o_wr_req
    //ov_wr_data
    //ov_wr_addr
    //ov_wr_bytes
    reg [31:0] rv_wr_data;
    reg [15:0] rv_wr_addr;
    reg [ 2:0] rv_wr_byte;
    always @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            o_wr_req    <= 0;
            ov_wr_data  <= 32'd1;
            ov_wr_addr  <= 16'd0;
            ov_wr_bytes <= 3'd0;
        end
        else if(wait_ini2wr_sb)begin
            o_wr_req    <= 1;
            ov_wr_data  <= 32'd1;
            ov_wr_addr  <= 16'h2D20;
            ov_wr_bytes <= 3'd1;
        end
        else if(wr_cmd_s2wr_cmd)begin
            o_wr_req    <= 1;
            ov_wr_data  <= rv_wr_data;
            ov_wr_addr  <= rv_wr_addr;
            ov_wr_bytes <= rv_wr_byte;
        end
        else if(vop_en2wr_a)begin
            o_wr_req    <= 1;
            ov_wr_data  <= 32'd0;
            ov_wr_addr  <= 16'h2D20;
            ov_wr_bytes <= 3'd1;
        end
        else begin
            o_wr_req    <= 0;
            ov_wr_data  <= ov_wr_data;
            ov_wr_addr  <= ov_wr_addr;
            ov_wr_bytes <= ov_wr_bytes;
        end
    end

    //rv_cmd_num
    always @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            rv_cmd_num  <= 0;
        end
        else if(wr_cmd2wr_cmd_s)begin
            rv_cmd_num <= rv_cmd_num + 1;
        end
        else begin
            rv_cmd_num <= rv_cmd_num;
        end
    end



    //o_rd_req
    //ov_rd_addr
    //ov_rd_bytes
    always @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            o_rd_req    <= 0;
            ov_rd_addr  <= 0;
            ov_rd_bytes <= 0;
        end
        else if(wr_a_w2rd_a | wr_sb_w2rd_sb)begin
            o_rd_req    <= 1;
            ov_rd_addr  <= 16'h1A0C;
            ov_rd_bytes <= 3'd1;
        end
        else begin
            o_rd_req    <= 0;
            ov_rd_addr  <= ov_rd_addr ;
            ov_rd_bytes <= ov_rd_bytes;
        end
    end


    //o_imx459_ini_done
    always @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            o_imx459_ini_done <= 0;
        end
        else if(ini_state_c == ini_done)begin
            o_imx459_ini_done <= 1;
        end
        else begin
            o_imx459_ini_done <= o_imx459_ini_done;
        end
    end

    //o_vop_power_en
    always @(posedge clk or negedge rst_n)begin
        if(~rst_n)begin
            o_vop_power_en <= 0;
        end
        else if(ini_state_c == vop_en)begin
            o_vop_power_en <= 1;
        end
        else begin
            o_vop_power_en <= o_vop_power_en;
        end
    end

    reg [31:0] INI_REG_DATA[0:CMD_NUM] ;
    reg [15:0] INI_REG_ADDR[0:CMD_NUM] ;
    reg [ 2:0] INI_REG_BYTE[0:CMD_NUM] ;
    initial begin
        INI_REG_ADDR[0  ] = 16'h0005;  INI_REG_BYTE[0  ] = 3'd1;  INI_REG_DATA[0  ] = 32'h2;
        INI_REG_ADDR[1  ] = 16'h0006;  INI_REG_BYTE[1  ] = 3'd1;  INI_REG_DATA[1  ] = 32'h14;
        INI_REG_ADDR[2  ] = 16'h0007;  INI_REG_BYTE[2  ] = 3'd1;  INI_REG_DATA[2  ] = 32'h14;
        INI_REG_ADDR[3  ] = 16'h000C;  INI_REG_BYTE[3  ] = 3'd2;  INI_REG_DATA[3  ] = 32'hC8;
        INI_REG_ADDR[4  ] = 16'h000E;  INI_REG_BYTE[4  ] = 3'd1;  INI_REG_DATA[4  ] = 32'h2;
        INI_REG_ADDR[5  ] = 16'h0010;  INI_REG_BYTE[5  ] = 3'd2;  INI_REG_DATA[5  ] = 32'hC8;
        INI_REG_ADDR[6  ] = 16'h0012;  INI_REG_BYTE[6  ] = 3'd1;  INI_REG_DATA[6  ] = 32'h1;
        INI_REG_ADDR[7  ] = 16'h0014;  INI_REG_BYTE[7  ] = 3'd1;  INI_REG_DATA[7  ] = 32'h0;
        INI_REG_ADDR[8  ] = 16'h0015;  INI_REG_BYTE[8  ] = 3'd1;  INI_REG_DATA[8  ] = 32'h1;
        INI_REG_ADDR[9  ] = 16'h0016;  INI_REG_BYTE[9  ] = 3'd1;  INI_REG_DATA[9  ] = 32'h0;
        INI_REG_ADDR[10 ] = 16'h00AC;  INI_REG_BYTE[10 ] = 3'd2;  INI_REG_DATA[10 ] = 32'h0;
        INI_REG_ADDR[11 ] = 16'h00AE;  INI_REG_BYTE[11 ] = 3'd2;  INI_REG_DATA[11 ] = 32'h5F;
        INI_REG_ADDR[12 ] = 16'h00CC;  INI_REG_BYTE[12 ] = 3'd1;  INI_REG_DATA[12 ] = 32'h0;
        INI_REG_ADDR[13 ] = 16'h00CD;  INI_REG_BYTE[13 ] = 3'd1;  INI_REG_DATA[13 ] = 32'h0;
        INI_REG_ADDR[14 ] = 16'h00D4;  INI_REG_BYTE[14 ] = 3'd4;  INI_REG_DATA[14 ] = 32'h2710;
        INI_REG_ADDR[15 ] = 16'h00DC;  INI_REG_BYTE[15 ] = 3'd1;  INI_REG_DATA[15 ] = 32'h0;
        INI_REG_ADDR[16 ] = 16'h00DD;  INI_REG_BYTE[16 ] = 3'd1;  INI_REG_DATA[16 ] = 32'h0;
        INI_REG_ADDR[17 ] = 16'h012A;  INI_REG_BYTE[17 ] = 3'd1;  INI_REG_DATA[17 ] = 32'h0;
        INI_REG_ADDR[18 ] = 16'h0148;  INI_REG_BYTE[18 ] = 3'd2;  INI_REG_DATA[18 ] = 32'h0;
        INI_REG_ADDR[19 ] = 16'h014A;  INI_REG_BYTE[19 ] = 3'd2;  INI_REG_DATA[19 ] = 32'h3E7;
        INI_REG_ADDR[20 ] = 16'h0150;  INI_REG_BYTE[20 ] = 3'd1;  INI_REG_DATA[20 ] = 32'h1;
        INI_REG_ADDR[21 ] = 16'h0300;  INI_REG_BYTE[21 ] = 3'd1;  INI_REG_DATA[21 ] = 32'h1;
        INI_REG_ADDR[22 ] = 16'h0301;  INI_REG_BYTE[22 ] = 3'd1;  INI_REG_DATA[22 ] = 32'h1;
        INI_REG_ADDR[23 ] = 16'h0303;  INI_REG_BYTE[23 ] = 3'd1;  INI_REG_DATA[23 ] = 32'h0;
        INI_REG_ADDR[24 ] = 16'h0304;  INI_REG_BYTE[24 ] = 3'd1;  INI_REG_DATA[24 ] = 32'h3;
        INI_REG_ADDR[25 ] = 16'h0305;  INI_REG_BYTE[25 ] = 3'd1;  INI_REG_DATA[25 ] = 32'h2;
        INI_REG_ADDR[26 ] = 16'h0306;  INI_REG_BYTE[26 ] = 3'd1;  INI_REG_DATA[26 ] = 32'h1;
        INI_REG_ADDR[27 ] = 16'h0307;  INI_REG_BYTE[27 ] = 3'd1;  INI_REG_DATA[27 ] = 32'h0;
        INI_REG_ADDR[28 ] = 16'h081C;  INI_REG_BYTE[28 ] = 3'd2;  INI_REG_DATA[28 ] = 32'h1;
        INI_REG_ADDR[29 ] = 16'h081E;  INI_REG_BYTE[29 ] = 3'd2;  INI_REG_DATA[29 ] = 32'h2;
        INI_REG_ADDR[30 ] = 16'h0820;  INI_REG_BYTE[30 ] = 3'd2;  INI_REG_DATA[30 ] = 32'h4;
        INI_REG_ADDR[31 ] = 16'h0822;  INI_REG_BYTE[31 ] = 3'd2;  INI_REG_DATA[31 ] = 32'h8;
        INI_REG_ADDR[32 ] = 16'h0824;  INI_REG_BYTE[32 ] = 3'd2;  INI_REG_DATA[32 ] = 32'h10;
        INI_REG_ADDR[33 ] = 16'h0826;  INI_REG_BYTE[33 ] = 3'd2;  INI_REG_DATA[33 ] = 32'h20;
        INI_REG_ADDR[34 ] = 16'h0828;  INI_REG_BYTE[34 ] = 3'd2;  INI_REG_DATA[34 ] = 32'h40;
        INI_REG_ADDR[35 ] = 16'h082A;  INI_REG_BYTE[35 ] = 3'd2;  INI_REG_DATA[35 ] = 32'h80;
        INI_REG_ADDR[36 ] = 16'h082C;  INI_REG_BYTE[36 ] = 3'd2;  INI_REG_DATA[36 ] = 32'h100;
        INI_REG_ADDR[37 ] = 16'h082E;  INI_REG_BYTE[37 ] = 3'd2;  INI_REG_DATA[37 ] = 32'h200;
        INI_REG_ADDR[38 ] = 16'h0830;  INI_REG_BYTE[38 ] = 3'd2;  INI_REG_DATA[38 ] = 32'h400;
        INI_REG_ADDR[39 ] = 16'h0832;  INI_REG_BYTE[39 ] = 3'd2;  INI_REG_DATA[39 ] = 32'h800;
        INI_REG_ADDR[40 ] = 16'h0850;  INI_REG_BYTE[40 ] = 3'd1;  INI_REG_DATA[40 ] = 32'h0;
        INI_REG_ADDR[41 ] = 16'h0852;  INI_REG_BYTE[41 ] = 3'd1;  INI_REG_DATA[41 ] = 32'h0;
        INI_REG_ADDR[42 ] = 16'h0853;  INI_REG_BYTE[42 ] = 3'd1;  INI_REG_DATA[42 ] = 32'h0;
        INI_REG_ADDR[43 ] = 16'h0854;  INI_REG_BYTE[43 ] = 3'd1;  INI_REG_DATA[43 ] = 32'h3;
        INI_REG_ADDR[44 ] = 16'h0856;  INI_REG_BYTE[44 ] = 3'd1;  INI_REG_DATA[44 ] = 32'h3;
        INI_REG_ADDR[45 ] = 16'h0857;  INI_REG_BYTE[45 ] = 3'd1;  INI_REG_DATA[45 ] = 32'h0;
        INI_REG_ADDR[46 ] = 16'h085A;  INI_REG_BYTE[46 ] = 3'd1;  INI_REG_DATA[46 ] = 32'h0;
        INI_REG_ADDR[47 ] = 16'h085C;  INI_REG_BYTE[47 ] = 3'd1;  INI_REG_DATA[47 ] = 32'h0;
        INI_REG_ADDR[48 ] = 16'h085D;  INI_REG_BYTE[48 ] = 3'd1;  INI_REG_DATA[48 ] = 32'h0;
        INI_REG_ADDR[49 ] = 16'h085E;  INI_REG_BYTE[49 ] = 3'd1;  INI_REG_DATA[49 ] = 32'h0;
        INI_REG_ADDR[50 ] = 16'h085F;  INI_REG_BYTE[50 ] = 3'd1;  INI_REG_DATA[50 ] = 32'h0;
        INI_REG_ADDR[51 ] = 16'h0860;  INI_REG_BYTE[51 ] = 3'd1;  INI_REG_DATA[51 ] = 32'h0;
        INI_REG_ADDR[52 ] = 16'h0861;  INI_REG_BYTE[52 ] = 3'd1;  INI_REG_DATA[52 ] = 32'h0;
        INI_REG_ADDR[53 ] = 16'h0862;  INI_REG_BYTE[53 ] = 3'd1;  INI_REG_DATA[53 ] = 32'h0;
        INI_REG_ADDR[54 ] = 16'h0864;  INI_REG_BYTE[54 ] = 3'd1;  INI_REG_DATA[54 ] = 32'h0;
        INI_REG_ADDR[55 ] = 16'h0865;  INI_REG_BYTE[55 ] = 3'd1;  INI_REG_DATA[55 ] = 32'h0;
        INI_REG_ADDR[56 ] = 16'h0866;  INI_REG_BYTE[56 ] = 3'd1;  INI_REG_DATA[56 ] = 32'h0;
        INI_REG_ADDR[57 ] = 16'h0868;  INI_REG_BYTE[57 ] = 3'd1;  INI_REG_DATA[57 ] = 32'h0;
        INI_REG_ADDR[58 ] = 16'h0869;  INI_REG_BYTE[58 ] = 3'd1;  INI_REG_DATA[58 ] = 32'h0;
        INI_REG_ADDR[59 ] = 16'h086A;  INI_REG_BYTE[59 ] = 3'd1;  INI_REG_DATA[59 ] = 32'h0;
        INI_REG_ADDR[60 ] = 16'h086C;  INI_REG_BYTE[60 ] = 3'd1;  INI_REG_DATA[60 ] = 32'h0;
        INI_REG_ADDR[61 ] = 16'h086D;  INI_REG_BYTE[61 ] = 3'd1;  INI_REG_DATA[61 ] = 32'h0;
        INI_REG_ADDR[62 ] = 16'h086E;  INI_REG_BYTE[62 ] = 3'd1;  INI_REG_DATA[62 ] = 32'h0;
        INI_REG_ADDR[63 ] = 16'h0870;  INI_REG_BYTE[63 ] = 3'd1;  INI_REG_DATA[63 ] = 32'h0;
        INI_REG_ADDR[64 ] = 16'h0871;  INI_REG_BYTE[64 ] = 3'd1;  INI_REG_DATA[64 ] = 32'h0;
        INI_REG_ADDR[65 ] = 16'h0872;  INI_REG_BYTE[65 ] = 3'd1;  INI_REG_DATA[65 ] = 32'h0;
        INI_REG_ADDR[66 ] = 16'h0874;  INI_REG_BYTE[66 ] = 3'd1;  INI_REG_DATA[66 ] = 32'h0;
        INI_REG_ADDR[67 ] = 16'h0875;  INI_REG_BYTE[67 ] = 3'd1;  INI_REG_DATA[67 ] = 32'h0;
        INI_REG_ADDR[68 ] = 16'h0876;  INI_REG_BYTE[68 ] = 3'd1;  INI_REG_DATA[68 ] = 32'h0;
        INI_REG_ADDR[69 ] = 16'h0878;  INI_REG_BYTE[69 ] = 3'd1;  INI_REG_DATA[69 ] = 32'h0;
        INI_REG_ADDR[70 ] = 16'h0879;  INI_REG_BYTE[70 ] = 3'd1;  INI_REG_DATA[70 ] = 32'h0;
        INI_REG_ADDR[71 ] = 16'h087A;  INI_REG_BYTE[71 ] = 3'd1;  INI_REG_DATA[71 ] = 32'h0;
        INI_REG_ADDR[72 ] = 16'h087C;  INI_REG_BYTE[72 ] = 3'd1;  INI_REG_DATA[72 ] = 32'h39;
        INI_REG_ADDR[73 ] = 16'h0898;  INI_REG_BYTE[73 ] = 3'd2;  INI_REG_DATA[73 ] = 32'h1C8;
        INI_REG_ADDR[74 ] = 16'h089A;  INI_REG_BYTE[74 ] = 3'd2;  INI_REG_DATA[74 ] = 32'h19;
        INI_REG_ADDR[75 ] = 16'h089C;  INI_REG_BYTE[75 ] = 3'd2;  INI_REG_DATA[75 ] = 32'h1CD;//1CD;
        INI_REG_ADDR[76 ] = 16'h089E;  INI_REG_BYTE[76 ] = 3'd2;  INI_REG_DATA[76 ] = 32'h19;
        INI_REG_ADDR[77 ] = 16'h08A6;  INI_REG_BYTE[77 ] = 3'd2;  INI_REG_DATA[77 ] = 32'h34;
        INI_REG_ADDR[78 ] = 16'h08A8;  INI_REG_BYTE[78 ] = 3'd2;  INI_REG_DATA[78 ] = 32'hFA;
        INI_REG_ADDR[79 ] = 16'h08BC;  INI_REG_BYTE[79 ] = 3'd1;  INI_REG_DATA[79 ] = 32'h14;
        INI_REG_ADDR[80 ] = 16'h08C4;  INI_REG_BYTE[80 ] = 3'd2;  INI_REG_DATA[80 ] = 32'hFFF;
        INI_REG_ADDR[81 ] = 16'h08C6;  INI_REG_BYTE[81 ] = 3'd2;  INI_REG_DATA[81 ] = 32'hFFF;
        INI_REG_ADDR[82 ] = 16'h08FA;  INI_REG_BYTE[82 ] = 3'd2;  INI_REG_DATA[82 ] = 32'h38;
        INI_REG_ADDR[83 ] = 16'h0904;  INI_REG_BYTE[83 ] = 3'd2;  INI_REG_DATA[83 ] = 32'h64;
        INI_REG_ADDR[84 ] = 16'h0A34;  INI_REG_BYTE[84 ] = 3'd1;  INI_REG_DATA[84 ] = 32'h3;
        INI_REG_ADDR[85 ] = 16'h0A38;  INI_REG_BYTE[85 ] = 3'd1;  INI_REG_DATA[85 ] = 32'h1;
        INI_REG_ADDR[86 ] = 16'h0A3C;  INI_REG_BYTE[86 ] = 3'd1;  INI_REG_DATA[86 ] = 32'h1;
        INI_REG_ADDR[87 ] = 16'h0A40;  INI_REG_BYTE[87 ] = 3'd1;  INI_REG_DATA[87 ] = 32'h1;
        INI_REG_ADDR[88 ] = 16'h0A44;  INI_REG_BYTE[88 ] = 3'd1;  INI_REG_DATA[88 ] = 32'h1;
        INI_REG_ADDR[89 ] = 16'h0A48;  INI_REG_BYTE[89 ] = 3'd1;  INI_REG_DATA[89 ] = 32'h1;
        INI_REG_ADDR[90 ] = 16'h0A58;  INI_REG_BYTE[90 ] = 3'd1;  INI_REG_DATA[90 ] = 32'h0;
        INI_REG_ADDR[91 ] = 16'h0A5A;  INI_REG_BYTE[91 ] = 3'd2;  INI_REG_DATA[91 ] = 32'h22;
        INI_REG_ADDR[92 ] = 16'h0A5C;  INI_REG_BYTE[92 ] = 3'd1;  INI_REG_DATA[92 ] = 32'h1;
        INI_REG_ADDR[93 ] = 16'h0A60;  INI_REG_BYTE[93 ] = 3'd1;  INI_REG_DATA[93 ] = 32'h0;
        INI_REG_ADDR[94 ] = 16'h0B18;  INI_REG_BYTE[94 ] = 3'd1;  INI_REG_DATA[94 ] = 32'h0;
        INI_REG_ADDR[95 ] = 16'h0B19;  INI_REG_BYTE[95 ] = 3'd1;  INI_REG_DATA[95 ] = 32'hA;
        INI_REG_ADDR[96 ] = 16'h0B1A;  INI_REG_BYTE[96 ] = 3'd2;  INI_REG_DATA[96 ] = 32'h103;
        INI_REG_ADDR[97 ] = 16'h0B1C;  INI_REG_BYTE[97 ] = 3'd2;  INI_REG_DATA[97 ] = 32'h0;
        INI_REG_ADDR[98 ] = 16'h0B1E;  INI_REG_BYTE[98 ] = 3'd2;  INI_REG_DATA[98 ] = 32'h0;
        INI_REG_ADDR[99 ] = 16'h0B24;  INI_REG_BYTE[99 ] = 3'd2;  INI_REG_DATA[99 ] = 32'h0;
        INI_REG_ADDR[100] = 16'h0B26;  INI_REG_BYTE[100] = 3'd1;  INI_REG_DATA[100] = 32'h0;
        INI_REG_ADDR[101] = 16'h0B27;  INI_REG_BYTE[101] = 3'd1;  INI_REG_DATA[101] = 32'h0;
        INI_REG_ADDR[102] = 16'h0B38;  INI_REG_BYTE[102] = 3'd1;  INI_REG_DATA[102] = 32'h1;
        INI_REG_ADDR[103] = 16'h0B3C;  INI_REG_BYTE[103] = 3'd1;  INI_REG_DATA[103] = 32'h24;
        INI_REG_ADDR[104] = 16'h0B3E;  INI_REG_BYTE[104] = 3'd1;  INI_REG_DATA[104] = 32'h0;
        INI_REG_ADDR[105] = 16'h0B40;  INI_REG_BYTE[105] = 3'd2;  INI_REG_DATA[105] = 32'hA;
        INI_REG_ADDR[106] = 16'h0B42;  INI_REG_BYTE[106] = 3'd1;  INI_REG_DATA[106] = 32'h0;
        INI_REG_ADDR[107] = 16'h0B43;  INI_REG_BYTE[107] = 3'd1;  INI_REG_DATA[107] = 32'h8;
        INI_REG_ADDR[108] = 16'h0B44;  INI_REG_BYTE[108] = 3'd1;  INI_REG_DATA[108] = 32'h8;
        INI_REG_ADDR[109] = 16'h0B46;  INI_REG_BYTE[109] = 3'd2;  INI_REG_DATA[109] = 32'h0;
        INI_REG_ADDR[110] = 16'h0B48;  INI_REG_BYTE[110] = 3'd1;  INI_REG_DATA[110] = 32'h3;
        INI_REG_ADDR[111] = 16'h0B4A;  INI_REG_BYTE[111] = 3'd2;  INI_REG_DATA[111] = 32'h0;
        INI_REG_ADDR[112] = 16'h0B4C;  INI_REG_BYTE[112] = 3'd2;  INI_REG_DATA[112] = 32'h64;
        INI_REG_ADDR[113] = 16'h0B4E;  INI_REG_BYTE[113] = 3'd2;  INI_REG_DATA[113] = 32'h0;
        INI_REG_ADDR[114] = 16'h0B50;  INI_REG_BYTE[114] = 3'd1;  INI_REG_DATA[114] = 32'h0;
        INI_REG_ADDR[115] = 16'h0B51;  INI_REG_BYTE[115] = 3'd1;  INI_REG_DATA[115] = 32'h0;
        INI_REG_ADDR[116] = 16'h0B52;  INI_REG_BYTE[116] = 3'd2;  INI_REG_DATA[116] = 32'h1;
        INI_REG_ADDR[117] = 16'h0B5C;  INI_REG_BYTE[117] = 3'd1;  INI_REG_DATA[117] = 32'h0;
        INI_REG_ADDR[118] = 16'h0B5D;  INI_REG_BYTE[118] = 3'd1;  INI_REG_DATA[118] = 32'h0;
        INI_REG_ADDR[119] = 16'h0B5E;  INI_REG_BYTE[119] = 3'd1;  INI_REG_DATA[119] = 32'h0;
        INI_REG_ADDR[120] = 16'h0B5F;  INI_REG_BYTE[120] = 3'd1;  INI_REG_DATA[120] = 32'h0;
        INI_REG_ADDR[121] = 16'h0B60;  INI_REG_BYTE[121] = 3'd1;  INI_REG_DATA[121] = 32'h0;
        INI_REG_ADDR[122] = 16'h0B61;  INI_REG_BYTE[122] = 3'd1;  INI_REG_DATA[122] = 32'h0;
        INI_REG_ADDR[123] = 16'h0B62;  INI_REG_BYTE[123] = 3'd1;  INI_REG_DATA[123] = 32'h0;
        INI_REG_ADDR[124] = 16'h0B63;  INI_REG_BYTE[124] = 3'd1;  INI_REG_DATA[124] = 32'h0;
        INI_REG_ADDR[125] = 16'h0B64;  INI_REG_BYTE[125] = 3'd1;  INI_REG_DATA[125] = 32'h0;
        INI_REG_ADDR[126] = 16'h0B65;  INI_REG_BYTE[126] = 3'd1;  INI_REG_DATA[126] = 32'h0;
        INI_REG_ADDR[127] = 16'h0B66;  INI_REG_BYTE[127] = 3'd1;  INI_REG_DATA[127] = 32'h0;
        INI_REG_ADDR[128] = 16'h0B67;  INI_REG_BYTE[128] = 3'd1;  INI_REG_DATA[128] = 32'h0;
        INI_REG_ADDR[129] = 16'h0B68;  INI_REG_BYTE[129] = 3'd1;  INI_REG_DATA[129] = 32'h0;
        INI_REG_ADDR[130] = 16'h0B69;  INI_REG_BYTE[130] = 3'd1;  INI_REG_DATA[130] = 32'h0;
        INI_REG_ADDR[131] = 16'h0B6A;  INI_REG_BYTE[131] = 3'd1;  INI_REG_DATA[131] = 32'h0;
        INI_REG_ADDR[132] = 16'h0B6B;  INI_REG_BYTE[132] = 3'd1;  INI_REG_DATA[132] = 32'h0;
        INI_REG_ADDR[133] = 16'h0B6C;  INI_REG_BYTE[133] = 3'd1;  INI_REG_DATA[133] = 32'h40;
        INI_REG_ADDR[134] = 16'h0B6D;  INI_REG_BYTE[134] = 3'd1;  INI_REG_DATA[134] = 32'h0;
        INI_REG_ADDR[135] = 16'h0B6E;  INI_REG_BYTE[135] = 3'd1;  INI_REG_DATA[135] = 32'h0;
        INI_REG_ADDR[136] = 16'h0B6F;  INI_REG_BYTE[136] = 3'd1;  INI_REG_DATA[136] = 32'h0;
        INI_REG_ADDR[137] = 16'h0B70;  INI_REG_BYTE[137] = 3'd1;  INI_REG_DATA[137] = 32'h0;
        INI_REG_ADDR[138] = 16'h0B71;  INI_REG_BYTE[138] = 3'd1;  INI_REG_DATA[138] = 32'h0;
        INI_REG_ADDR[139] = 16'h0B72;  INI_REG_BYTE[139] = 3'd1;  INI_REG_DATA[139] = 32'h0;
        INI_REG_ADDR[140] = 16'h0B73;  INI_REG_BYTE[140] = 3'd1;  INI_REG_DATA[140] = 32'h0;
        INI_REG_ADDR[141] = 16'h0B74;  INI_REG_BYTE[141] = 3'd1;  INI_REG_DATA[141] = 32'h0;
        INI_REG_ADDR[142] = 16'h0B75;  INI_REG_BYTE[142] = 3'd1;  INI_REG_DATA[142] = 32'h0;
        INI_REG_ADDR[143] = 16'h0B76;  INI_REG_BYTE[143] = 3'd1;  INI_REG_DATA[143] = 32'h0;
        INI_REG_ADDR[144] = 16'h0B77;  INI_REG_BYTE[144] = 3'd1;  INI_REG_DATA[144] = 32'h0;
        INI_REG_ADDR[145] = 16'h0B78;  INI_REG_BYTE[145] = 3'd1;  INI_REG_DATA[145] = 32'h0;
        INI_REG_ADDR[146] = 16'h0B79;  INI_REG_BYTE[146] = 3'd1;  INI_REG_DATA[146] = 32'h0;
        INI_REG_ADDR[147] = 16'h0B7A;  INI_REG_BYTE[147] = 3'd1;  INI_REG_DATA[147] = 32'h0;
        INI_REG_ADDR[148] = 16'h0B7B;  INI_REG_BYTE[148] = 3'd1;  INI_REG_DATA[148] = 32'h0;
    end

    integer  i;
    always @(posedge clk )begin
        if(wr_cmd2wr_cmd_s)begin
            for(i = 0; i <= CMD_NUM; i = i + 1)begin
                INI_REG_DATA[i] <= INI_REG_DATA[i + 1];
                INI_REG_ADDR[i] <= INI_REG_ADDR[i + 1];
                INI_REG_BYTE[i] <= INI_REG_BYTE[i + 1];
            end
        end
        else begin
            for(i = 0; i <= CMD_NUM; i = i + 1)begin
                INI_REG_DATA[i] <= INI_REG_DATA[i];
                INI_REG_ADDR[i] <= INI_REG_ADDR[i];
                INI_REG_BYTE[i] <= INI_REG_BYTE[i];
            end
        end
    end

    always @(*)begin
        if(~rst_n)begin
            rv_wr_data = 0;
            rv_wr_addr = 0;
            rv_wr_byte = 0;
        end
        else begin
            rv_wr_data <= INI_REG_DATA[0];
            rv_wr_addr <= INI_REG_ADDR[0];
            rv_wr_byte <= INI_REG_BYTE[0];
        end
    end

    // ila_imx459_ini u_ila_imx459_ini(
    // .clk                               (clk                       ),
    // .probe0                            (ini_state_c               ),
    // .probe1                            (rv_cmd_num                ),
    // .probe2                            (rv_wr_addr                ),
    // .probe3                            (rv_read_data              ),
    // .probe4                            (i_READY                   ),
    // .probe5                            (i_COMREADY                ),
    // .probe6                            (o_imx459_ini_done         )
    // );
endmodule
