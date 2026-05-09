`include "fp_pkg.vh"

module cordic_hyp (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,
    input  wire signed [`Q_WIDTH-1:0] z_in,
    input  wire               is_vectoring_in,
    output wire signed [`Q_WIDTH-1:0] x_out,
    output wire signed [`Q_WIDTH-1:0] y_out,
    output wire signed [`Q_WIDTH-1:0] z_out,
    output wire               done
);

    localparam INT_FRAC  = `CORDIC_FRAC;
    localparam INT_WIDTH = `CORDIC_WIDTH;

    reg [3:0] i;
    reg signed [INT_WIDTH-1:0] x, y, z;
    (* fsm_encoding = "binary" *) reg [1:0] state;
    reg is_vectoring;
    reg repeated;

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    wire d_pos = is_vectoring ? (y < 0) : (z >= 0);

    wire signed [INT_WIDTH-1:0] angle_i =
        (i >= 4'd5) ? ($signed({{(INT_WIDTH-1){1'b0}}, 1'b1}) <<< (INT_FRAC - i)) :
        get_atanh(i);

    wire signed [INT_WIDTH-1:0] x_shift = x >>> i;
    wire signed [INT_WIDTH-1:0] y_shift = y >>> i;

    wire signed [INT_WIDTH-1:0] next_x_w = d_pos ? (x + y_shift) : (x - y_shift);
    wire signed [INT_WIDTH-1:0] next_y_w = d_pos ? (y + x_shift) : (y - x_shift);
    wire signed [INT_WIDTH-1:0] next_z_w = d_pos ? (z - angle_i) : (z + angle_i);

    wire repeat_iter = ((i == 4'd4) || (i == 4'd13)) && !repeated;

    wire [3:0] last_i   = 4'd14;

    assign x_out = x;
    assign y_out = y;
    assign z_out = z;
    assign done  = (state == S_DONE);

    function signed [INT_WIDTH-1:0] get_atanh;
        input [3:0] idx;
        begin
            case (idx)
            4'd1: get_atanh = 20'sd9000;
            4'd2: get_atanh = 20'sd4185;
            4'd3: get_atanh = 20'sd2059;
            4'd4: get_atanh = 20'sd1025;
            default: get_atanh = 0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        x <= x_in;
                        y <= y_in;
                        z <= z_in;
                        is_vectoring <= is_vectoring_in;
                        repeated <= 0;
                        i <= 1;
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
                        if (i == last_i) begin
                            state <= S_DONE;
                        end else begin
                            i <= i + 1;
                        end
                    end
                end
                S_DONE: begin
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
