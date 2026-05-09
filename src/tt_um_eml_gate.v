`default_nettype none

module tt_um_eml_gate (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire mosi = ui_in[0];
    wire sclk = ui_in[1];
    wire cs_n = ui_in[2];

    wire miso;
    wire busy_ext;
    wire done_ext;
    wire error_ext;

    assign uo_out[0]   = miso;
    assign uo_out[1]   = busy_ext;
    assign uo_out[2]   = done_ext;
    assign uo_out[3]   = error_ext;
    assign uo_out[7:4] = 4'b0000;

    assign uio_out = 8'd0;
    assign uio_oe  = 8'd0;

    wire _unused = &{ena, ui_in[7:3], uio_in, 1'b0};

    eml_spi_gate u_eml_spi_gate (
        .clk   (clk),
        .rst_n (rst_n),
        .mosi  (mosi),
        .sclk  (sclk),
        .cs_n  (cs_n),
        .miso  (miso),
        .busy  (busy_ext),
        .done  (done_ext),
        .error (error_ext)
    );

endmodule
