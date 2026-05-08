`include "fp_pkg.vh"

module eml_gate_top (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [1:0]                 opcode,
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,

    output wire signed [`Q_WIDTH-1:0] result,
    output wire signed [`Q_WIDTH-1:0] result_secondary,
    output wire                       done,
    output wire                       busy,
    output reg                        error
);

    // ── Opcodes ──
    localparam [1:0] OP_EML    = 2'd0;
    localparam [1:0] OP_MUL    = 2'd1;
    localparam [1:0] OP_SINCOS = 2'd2;
    localparam [1:0] OP_ATAN2  = 2'd3;

    // ── State encoding ──
    localparam [3:0] S_IDLE            = 4'd0;
    localparam [3:0] S_EML_SCALE_X     = 4'd1;
    localparam [3:0] S_EML_NORM        = 4'd2;
    localparam [3:0] S_EML_WAIT_LN_MUL = 4'd3;
    localparam [3:0] S_EML_WAIT_LN_COR = 4'd4;
    localparam [3:0] S_EML_MUL_EXP     = 4'd5;
    localparam [3:0] S_EML_PREP_CORDIC = 4'd6;
    localparam [3:0] S_EML_CORDIC_EXP  = 4'd7;
    localparam [3:0] S_EML_EXP_SHIFT   = 4'd8;
    localparam [3:0] S_EML_FINISH      = 4'd9;
    localparam [3:0] S_DONE            = 4'd10;
    localparam [3:0] S_WAIT_MUL        = 4'd11;
    localparam [3:0] S_WAIT_CORDIC     = 4'd12;

    reg [3:0] state;

    // ── Working registers ──
    // reg_work_0 doubles as secondary output (sin from SINCOS)
    reg signed [`Q_WIDTH-1:0] reg_x;
    reg signed [`Q_WIDTH-1:0] reg_work_0;
    reg signed [`Q_WIDTH-1:0] reg_work_1;
    reg signed [4:0]          reg_k;       // 5-bit: range ±16 suffices for Q6.10
    reg [1:0]                 reg_opcode;

    // ── Shared multiplier ──
    reg                         mul_start_r;
    reg  signed [`Q_WIDTH-1:0]  mul_a_r;
    reg  signed [`Q_WIDTH-1:0]  mul_b_r;
    wire signed [`Q_WIDTH-1:0]  mul_result;
    wire                        mul_done;

    fp_mul_seq u_shared_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start_r),
        .a(mul_a_r), .b(mul_b_r), .result(mul_result), .done(mul_done)
    );

    // ── Shared CORDIC ──
    reg                         cordic_start_r;
    reg  [1:0]                  cordic_mode_r;
    reg  signed [`Q_WIDTH-1:0]  cordic_x_in_r;
    reg  signed [`Q_WIDTH-1:0]  cordic_y_in_r;
    reg  signed [`Q_WIDTH-1:0]  cordic_z_in_r;
    wire signed [`Q_WIDTH-1:0]  cordic_x_out;
    wire signed [`Q_WIDTH-1:0]  cordic_y_out;
    wire signed [`Q_WIDTH-1:0]  cordic_z_out;
    wire                        cordic_done;

    cordic_hyp u_shared_cordic (
        .clk(clk), .rst_n(rst_n), .start(cordic_start_r),
        .mode(cordic_mode_r),
        .x_in(cordic_x_in_r), .y_in(cordic_y_in_r), .z_in(cordic_z_in_r),
        .x_out(cordic_x_out), .y_out(cordic_y_out), .z_out(cordic_z_out),
        .done(cordic_done)
    );

    // ── Wide intermediates (17-bit guard) ──
    wire signed [`Q_WIDTH:0] exp_sum_wide =
        $signed({cordic_x_out[`Q_WIDTH-1], cordic_x_out}) +
        $signed({cordic_y_out[`Q_WIDTH-1], cordic_y_out});

    wire signed [`Q_WIDTH:0] ln_full_wide =
        ($signed({cordic_z_out[`Q_WIDTH-1], cordic_z_out}) <<< 1) +
        $signed({mul_result[`Q_WIDTH-1], mul_result});

    wire signed [`Q_WIDTH:0] final_result_wide =
        $signed({reg_work_1[`Q_WIDTH-1], reg_work_1}) -
        $signed({reg_work_0[`Q_WIDTH-1], reg_work_0});

    // ── Constants ──
    localparam signed [`Q_WIDTH-1:0] INT_ZERO = `FP_ZERO;

    // ── reg_k scaled to Q6.10 ──
    wire signed [`Q_WIDTH-1:0] reg_k_scaled =
        $signed({{(`Q_WIDTH-5){reg_k[4]}}, reg_k}) <<< `Q_FRAC;

    // ── Exp range reduction: extract integer part of x/ln2 ──
    wire signed [`Q_WIDTH-1:0] exp_k_rounded =
        reg_work_1[`Q_WIDTH-1]
            ? (reg_work_1 - (16'sd1 <<< (`Q_FRAC-1)))
            : (reg_work_1 + (16'sd1 <<< (`Q_FRAC-1)));
    wire signed [`Q_WIDTH-1:0] exp_k_shifted = exp_k_rounded >>> `Q_FRAC;

    // ── Saturation helper (simplified) ──
    function signed [`Q_WIDTH-1:0] saturate;
        input signed [`Q_WIDTH:0] value;
        begin
            if (value[`Q_WIDTH] != value[`Q_WIDTH-1])
                // overflow: MSBs disagree → clamp based on sign
                saturate = value[`Q_WIDTH] ? {1'b1, {(`Q_WIDTH-1){1'b0}}} :
                                             {1'b0, {(`Q_WIDTH-1){1'b1}}};
            else
                saturate = value[`Q_WIDTH-1:0];
        end
    endfunction

    // ── Output assignments ──
    // reg_work_0 doubles as secondary (sin from SINCOS)
    assign result           = reg_work_1;
    assign result_secondary = reg_work_0;
    assign done   = (state == S_DONE);
    assign busy   = (state != S_IDLE);

    // Suppress warnings
    wire _unused = &{exp_sum_wide[`Q_WIDTH], ln_full_wide[`Q_WIDTH],
                     exp_k_shifted[`Q_WIDTH-1:5], 1'b0};

    // ── Main FSM ──
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            error          <= 1'b0;
            mul_start_r    <= 1'b0;
            cordic_start_r <= 1'b0;
        end else begin
            mul_start_r    <= 1'b0;
            cordic_start_r <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        error        <= 1'b0;
                        reg_work_1   <= INT_ZERO;
                        reg_work_0   <= INT_ZERO;
                        reg_opcode   <= opcode;

                        case (opcode)
                            OP_MUL: begin
                                mul_a_r     <= x_in;
                                mul_b_r     <= y_in;
                                mul_start_r <= 1'b1;
                                state       <= S_WAIT_MUL;
                            end
                            OP_SINCOS: begin
                                cordic_mode_r  <= 2'b10;
                                cordic_x_in_r  <= `CORDIC_INV_GAIN_CIRC;
                                cordic_y_in_r  <= INT_ZERO;
                                cordic_z_in_r  <= x_in;
                                cordic_start_r <= 1'b1;
                                state          <= S_WAIT_CORDIC;
                            end
                            OP_ATAN2: begin
                                cordic_mode_r  <= 2'b11;
                                cordic_x_in_r  <= x_in;
                                cordic_y_in_r  <= y_in;
                                cordic_z_in_r  <= INT_ZERO;
                                cordic_start_r <= 1'b1;
                                state          <= S_WAIT_CORDIC;
                            end
                            default: begin // OP_EML — no special-case handling, host manages inf/nan
                                reg_x       <= x_in;
                                reg_work_0  <= y_in;
                                reg_k       <= 5'sd0;
                                mul_a_r     <= x_in;
                                mul_b_r     <= `FP_INV_LN2;
                                mul_start_r <= 1'b1;
                                state       <= S_EML_SCALE_X;
                            end
                        endcase
                    end
                end

                // ── OP_MUL: wait for multiplier ──
                S_WAIT_MUL: begin
                    if (mul_done) begin
                        reg_work_1 <= mul_result;
                        state      <= S_DONE;
                    end
                end

                // ── OP_SINCOS / OP_ATAN2: wait for CORDIC ──
                S_WAIT_CORDIC: begin
                    if (cordic_done) begin
                        if (reg_opcode == OP_SINCOS) begin
                            reg_work_1 <= cordic_x_out;  // cos
                            reg_work_0 <= cordic_y_out;  // sin (doubles as secondary output)
                        end else begin
                            reg_work_1 <= cordic_z_out;  // atan2
                        end
                        state <= S_DONE;
                    end
                end

                // ── EML states ──
                S_EML_SCALE_X: begin
                    if (mul_done) begin
                        reg_work_1 <= mul_result;
                        state      <= S_EML_NORM;
                    end
                end

                S_EML_NORM: begin
                    if (reg_work_0 >= `FP_TWO) begin
                        reg_work_0 <= reg_work_0 >>> 1;
                        reg_k      <= reg_k + 5'sd1;
                    end else if (reg_work_0 < `FP_ONE) begin
                        reg_work_0 <= reg_work_0 <<< 1;
                        reg_k      <= reg_k - 5'sd1;
                    end else begin
                        mul_a_r     <= reg_k_scaled;
                        mul_b_r     <= `FP_LN2;
                        mul_start_r <= 1'b1;
                        state       <= S_EML_WAIT_LN_MUL;
                    end
                end

                S_EML_WAIT_LN_MUL: begin
                    if (mul_done) begin
                        cordic_mode_r  <= 2'b01;
                        cordic_x_in_r  <= reg_work_0 + `FP_ONE;
                        cordic_y_in_r  <= reg_work_0 - `FP_ONE;
                        cordic_z_in_r  <= INT_ZERO;
                        cordic_start_r <= 1'b1;
                        state          <= S_EML_WAIT_LN_COR;
                    end
                end

                S_EML_WAIT_LN_COR: begin
                    if (cordic_done) begin
                        reg_work_0 <= ln_full_wide[`Q_WIDTH-1:0];
                        reg_k      <= exp_k_shifted[4:0];
                        state      <= S_EML_MUL_EXP;
                    end
                end

                S_EML_MUL_EXP: begin
                    mul_a_r     <= reg_k_scaled;
                    mul_b_r     <= `FP_LN2;
                    mul_start_r <= 1'b1;
                    state       <= S_EML_PREP_CORDIC;
                end

                S_EML_PREP_CORDIC: begin
                    if (mul_done) begin
                        cordic_mode_r  <= 2'b00;
                        cordic_x_in_r  <= `CORDIC_INV_GAIN_HYP;
                        cordic_y_in_r  <= INT_ZERO;
                        cordic_z_in_r  <= reg_x - mul_result;
                        cordic_start_r <= 1'b1;
                        state          <= S_EML_CORDIC_EXP;
                    end
                end

                S_EML_CORDIC_EXP: begin
                    if (cordic_done) begin
                        reg_work_1 <= exp_sum_wide[`Q_WIDTH-1:0];
                        state      <= S_EML_EXP_SHIFT;
                    end
                end

                S_EML_EXP_SHIFT: begin
                    if (reg_k > 0) begin
                        if (reg_work_1 > ($signed({1'b0, {(`Q_WIDTH-1){1'b1}}}) >>> 1))
                            reg_work_1 <= {1'b0, {(`Q_WIDTH-1){1'b1}}};
                        else
                            reg_work_1 <= reg_work_1 <<< 1;
                        reg_k <= reg_k - 5'sd1;
                    end else if (reg_k < 0) begin
                        reg_work_1 <= reg_work_1 >>> 1;
                        reg_k      <= reg_k + 5'sd1;
                    end else begin
                        state <= S_EML_FINISH;
                    end
                end

                S_EML_FINISH: begin
                    reg_work_1 <= saturate(final_result_wide);
                    state      <= S_DONE;
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            if (start && (state != S_IDLE)) error <= 1'b1;
        end
    end
endmodule
