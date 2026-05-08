`include "fp_pkg.vh"

module cordic_hyp #(
    parameter EXT_WIDTH = `Q_WIDTH,      // External port width (16)
    parameter EXT_FRAC  = `Q_FRAC        // External fractional bits (10)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [1:0]         mode,
    // mode 2'b00: Hyperbolic rotation  (exp → cosh+sinh)
    // mode 2'b01: Hyperbolic vectoring (ln  → atanh)
    // mode 2'b10: Circular rotation    (sincos → cos,sin)
    // mode 2'b11: Circular vectoring   (atan2)
    input  wire signed [EXT_WIDTH-1:0] x_in,
    input  wire signed [EXT_WIDTH-1:0] y_in,
    input  wire signed [EXT_WIDTH-1:0] z_in,
    output wire signed [EXT_WIDTH-1:0] x_out,
    output wire signed [EXT_WIDTH-1:0] y_out,
    output wire signed [EXT_WIDTH-1:0] z_out,
    output wire               done
);

    // ── Internal precision ──
    localparam INT_FRAC  = `CORDIC_FRAC;       // 14
    localparam INT_WIDTH = `CORDIC_WIDTH;       // 20
    localparam EXTRA     = INT_FRAC - EXT_FRAC; // 4 extra bits

    reg [3:0] i;
    reg signed [INT_WIDTH-1:0] x, y, z;
    reg [1:0] state;
    reg [1:0] mode_reg;
    reg repeated;

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    wire is_circular   = mode_reg[1];
    wire is_vectoring  = mode_reg[0];

    // Direction: rotation drives z→0, vectoring drives y→0
    wire d = is_vectoring ? (y > 0) : (z < 0);

    // ── Angle lookup: short table for i<5, computed shift for i≥5 ──
    wire signed [INT_WIDTH-1:0] angle_i =
        (i >= 4'd5) ? ($signed({{(INT_WIDTH-1){1'b0}}, 1'b1}) <<< (INT_FRAC - i)) :
        (is_circular ? get_atan(i) : get_atanh(i));

    // Shifted values (arithmetic shift for sign preservation)
    wire signed [INT_WIDTH-1:0] x_shift = x >>> i;
    wire signed [INT_WIDTH-1:0] y_shift = y >>> i;

    // Circular: x' = x - d*y>>i,  y' = y + d*x>>i  (minus on x)
    // Hyperbolic: x' = x + d*y>>i, y' = y + d*x>>i  (plus on x)
    wire signed [INT_WIDTH-1:0] next_x_w = d
        ? (is_circular ? (x + y_shift) : (x - y_shift))
        : (is_circular ? (x - y_shift) : (x + y_shift));
    wire signed [INT_WIDTH-1:0] next_y_w = d
        ? (y - x_shift) : (y + x_shift);
    wire signed [INT_WIDTH-1:0] next_z_w = d
        ? (z + angle_i) : (z - angle_i);

    // Repeat iterations for hyperbolic convergence: i=4 and i=13
    wire repeat_iter = !is_circular && ((i == 4'd4) || (i == 4'd13)) && !repeated;

    // Iteration limits: circular starts at 0, hyperbolic at 1
    wire [3:0] start_i  = is_circular ? 4'd0 : 4'd1;
    wire [3:0] last_i   = 4'd14;

    // ── Output: truncate Q6.14 → Q6.10 ──
    assign x_out = x[INT_WIDTH-1:EXTRA];
    assign y_out = y[INT_WIDTH-1:EXTRA];
    assign z_out = z[INT_WIDTH-1:EXTRA];
    assign done  = (state == S_DONE);

    // ── atanh short LUT (Q6.14, i=1..4 only) ──
    function signed [INT_WIDTH-1:0] get_atanh;
        input [3:0] idx;
        case (idx)
            4'd1:  get_atanh = 20'sd9000;   // atanh(0.5)
            4'd2:  get_atanh = 20'sd4185;   // atanh(0.25)
            4'd3:  get_atanh = 20'sd2059;   // atanh(0.125)
            4'd4:  get_atanh = 20'sd1025;   // atanh(0.0625)
            default: get_atanh = 0;
        endcase
    endfunction

    // ── atan short LUT (Q6.14, i=0..4 only) ──
    function signed [INT_WIDTH-1:0] get_atan;
        input [3:0] idx;
        case (idx)
            4'd0:  get_atan = 20'sd12868;   // atan(1)   = π/4
            4'd1:  get_atan = 20'sd7596;    // atan(1/2)
            4'd2:  get_atan = 20'sd4014;    // atan(1/4)
            4'd3:  get_atan = 20'sd2037;    // atan(1/8)
            4'd4:  get_atan = 20'sd1023;    // atan(1/16)
            default: get_atan = 0;
        endcase
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        // Pad inputs from Q6.10 → Q6.14
                        x <= $signed(x_in) <<< EXTRA;
                        y <= $signed(y_in) <<< EXTRA;
                        z <= $signed(z_in) <<< EXTRA;
                        mode_reg <= mode;
                        i <= mode[1] ? 4'd0 : 4'd1;  // circular starts at 0
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
