`include "fp_pkg.vh"

module cordic_hyp #(
    parameter WIDTH = `Q_WIDTH,
    parameter FRAC  = `Q_FRAC
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire [1:0]         mode,
    // mode 2'b00: Hyperbolic rotation  (exp → cosh+sinh)
    // mode 2'b01: Hyperbolic vectoring (ln  → atanh)
    // mode 2'b10: Circular rotation    (sincos → cos,sin)
    // mode 2'b11: Circular vectoring   (atan2)
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
    reg [1:0] mode_reg;
    reg repeated;

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    wire is_circular   = mode_reg[1];
    wire is_vectoring  = mode_reg[0];

    // Direction: rotation drives z→0, vectoring drives y→0
    wire d = is_vectoring ? (y > 0) : (z < 0);

    // ── Angle lookup: short table for i<5, barrel shift for i≥5 ──
    // For i≥5: atanh(2^-i) ≈ atan(2^-i) ≈ 2^-i = 1 << (FRAC - i)
    wire signed [WIDTH-1:0] angle_i =
        (i >= 4'd5) ? ($signed({{(WIDTH-1){1'b0}}, 1'b1}) <<< (FRAC - i)) :
        (is_circular ? get_atan(i) : get_atanh(i));

    // Shifted values
    wire signed [WIDTH-1:0] x_shift = x >>> i;
    wire signed [WIDTH-1:0] y_shift = y >>> i;

    // Circular: x' = x - d*y>>i,  y' = y + d*x>>i
    // Hyperbolic: x' = x + d*y>>i, y' = y + d*x>>i
    wire signed [WIDTH-1:0] next_x_w = d
        ? (is_circular ? (x + y_shift) : (x - y_shift))
        : (is_circular ? (x - y_shift) : (x + y_shift));
    wire signed [WIDTH-1:0] next_y_w = d
        ? (y - x_shift) : (y + x_shift);
    wire signed [WIDTH-1:0] next_z_w = d
        ? (z + angle_i) : (z - angle_i);

    // Repeat iteration 4 for hyperbolic convergence
    wire repeat_iter = !is_circular && (i == 4'd4) && !repeated;

    // Iteration limits
    wire [3:0] start_i  = is_circular ? 4'd0 : 4'd1;
    wire [3:0] last_i   = 4'd12;

    assign x_out = x;
    assign y_out = y;
    assign z_out = z;
    assign done  = (state == S_DONE);

    // ── atanh short LUT (Q6.10, i=1..4 only) ──
    function signed [WIDTH-1:0] get_atanh;
        input [3:0] idx;
        case (idx)
            4'd1:  get_atanh = 16'sd563;   // atanh(0.5)
            4'd2:  get_atanh = 16'sd262;   // atanh(0.25)
            4'd3:  get_atanh = 16'sd129;   // atanh(0.125)
            4'd4:  get_atanh = 16'sd64;    // atanh(0.0625)
            default: get_atanh = 0;
        endcase
    endfunction

    // ── atan short LUT (Q6.10, i=0..4 only) ──
    function signed [WIDTH-1:0] get_atan;
        input [3:0] idx;
        case (idx)
            4'd0:  get_atan = 16'sd804;    // atan(1)   = π/4
            4'd1:  get_atan = 16'sd475;    // atan(1/2)
            4'd2:  get_atan = 16'sd251;    // atan(1/4)
            4'd3:  get_atan = 16'sd127;    // atan(1/8)
            4'd4:  get_atan = 16'sd64;     // atan(1/16)
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
                        x <= x_in; y <= y_in; z <= z_in;
                        mode_reg <= mode;
                        i <= mode[1] ? 4'd0 : 4'd1;
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
