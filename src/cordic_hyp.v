`include "fp_pkg.vh"

module cordic_hyp #(
    parameter WIDTH = 24,
    parameter FRAC  = 16
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [1:0]         mode, // 0: Rotation, 1: Vectoring
    input  wire signed [WIDTH-1:0] x_in,
    input  wire signed [WIDTH-1:0] y_in,
    input  wire signed [WIDTH-1:0] z_in,
    output wire signed [WIDTH-1:0] x_out,
    output wire signed [WIDTH-1:0] y_out,
    output wire signed [WIDTH-1:0] z_out,
    output wire               done
);

    reg [3:0] i;
    reg signed [WIDTH-1:0] x, y, z;
    reg [1:0] state;
    reg repeated;
    wire _unused_frac = &{1'b0, FRAC[0]};

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    wire d = (mode == 2'b0) ? (z < 0) : (y > 0);
    wire signed [WIDTH-1:0] atanh_i = get_atanh(i);
    wire signed [WIDTH-1:0] next_x_w = d ? (x - (y >>> i)) : (x + (y >>> i));
    wire signed [WIDTH-1:0] next_y_w = d ? (y - (x >>> i)) : (y + (x >>> i));
    wire signed [WIDTH-1:0] next_z_w = d ? (z + atanh_i) : (z - atanh_i);
    wire repeat_iter = ((i == 4'd4) || (i == 4'd13)) && !repeated;

    assign x_out = x;
    assign y_out = y;
    assign z_out = z;
    assign done  = (state == S_DONE);

    function signed [WIDTH-1:0] get_atanh;
        input [3:0] idx;
        case (idx)
            4'd1:  get_atanh = 24'sd2250;
            4'd2:  get_atanh = 24'sd1046;
            4'd3:  get_atanh = 24'sd515;
            4'd4:  get_atanh = 24'sd256;
            4'd5:  get_atanh = 24'sd128;
            4'd6:  get_atanh = 24'sd64;
            4'd7:  get_atanh = 24'sd32;
            4'd8:  get_atanh = 24'sd16;
            4'd9:  get_atanh = 24'sd8;
            4'd10: get_atanh = 24'sd4;
            4'd11: get_atanh = 24'sd2;
            4'd12: get_atanh = 24'sd1;
            4'd13: get_atanh = 24'sd1;
            4'd14: get_atanh = 24'sd0;
            4'd15: get_atanh = 24'sd0;
            default: get_atanh = 0;
        endcase
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            i <= 4'd0;
            x <= {WIDTH{1'b0}};
            y <= {WIDTH{1'b0}};
            z <= {WIDTH{1'b0}};
            repeated <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        x <= x_in; y <= y_in; z <= z_in;
                        i <= 4'd1;
                        repeated <= 0;
                        state <= S_CALC;
                    end
                end
                S_CALC: begin
                    x <= next_x_w;
                    y <= next_y_w;
                    z <= next_z_w;

                    if (repeat_iter) begin
                        repeated <= 1;
                    end else begin
                        repeated <= 0;
                        if (i == 4'd15) begin
                            state <= S_DONE;
                        end else begin
                            i <= i + 1;
                        end
                    end
                end
                S_DONE: begin
                    state <= S_IDLE;
                end
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule
