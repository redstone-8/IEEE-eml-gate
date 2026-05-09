`include "fp_pkg.vh"

module eml_gate_top (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [1:0]                 opcode,
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,

    output wire signed [`Q_WIDTH-1:0] result,
    output wire                       done,
    output wire                       busy,
    output reg                        error,
    output reg                        domain_error,
    output reg                        overflow
);

    localparam [1:0] OP_MUL    = 2'd1;

    localparam [3:0] S_IDLE            = 4'd0;
    localparam [3:0] S_WAIT_MUL        = 4'd1;
    localparam [3:0] S_EML_SCALE_X     = 4'd2;
    localparam [3:0] S_EML_NORM        = 4'd3;
    localparam [3:0] S_EML_WAIT_LN_MUL = 4'd4;
    localparam [3:0] S_EML_WAIT_LN_COR = 4'd5;
    localparam [3:0] S_EML_MUL_EXP     = 4'd6;
    localparam [3:0] S_EML_PREP_CORDIC = 4'd7;
    localparam [3:0] S_EML_CORDIC_EXP  = 4'd8;
    localparam [3:0] S_EML_EXP_SHIFT   = 4'd9;
    localparam [3:0] S_EML_FINISH      = 4'd10;
    localparam [3:0] S_DONE            = 4'd11;

    (* fsm_encoding = "binary" *) reg [3:0] state;

    reg signed [`Q_WIDTH-1:0] reg_x;
    reg signed [`Q_WIDTH-1:0] reg_work_0;
    reg signed [`Q_WIDTH-1:0] reg_work_1;
    reg signed [5:0]          reg_k;
    reg [4:0]                 shift_cnt;

    localparam signed [`Q_WIDTH-1:0] INT_ZERO = `FP_ZERO;
    localparam signed [`Q_WIDTH-1:0] INT_NEG_TEN = -20'sd163840;

    wire signed [`Q_WIDTH-1:0] reg_k_scaled =
        $signed({{(`Q_WIDTH-6){reg_k[5]}}, reg_k}) <<< `Q_FRAC;

    reg                         mul_start_r;
    wire signed [`Q_WIDTH-1:0]  mul_result;
    wire                        mul_done;

    wire signed [`Q_WIDTH-1:0] mul_a_w =
        (state == S_WAIT_MUL) ? reg_x :
        (state == S_EML_SCALE_X) ? reg_x :
        (state == S_EML_WAIT_LN_MUL) ? reg_k_scaled :
        (state == S_EML_PREP_CORDIC) ? reg_k_scaled :
        INT_ZERO;

    wire signed [`Q_WIDTH-1:0] mul_b_w =
        (state == S_WAIT_MUL) ? reg_work_0 :
        (state == S_EML_SCALE_X) ? `FP_INV_LN2 :
        (state == S_EML_WAIT_LN_MUL) ? `FP_LN2 :
        (state == S_EML_PREP_CORDIC) ? `FP_LN2 :
        INT_ZERO;

    fp_mul_seq u_shared_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start_r),
        .a(mul_a_w), .b(mul_b_w), .result(mul_result), .done(mul_done)
    );

    reg                         cordic_start_r;
    wire signed [`Q_WIDTH-1:0]  cordic_x_out;
    wire signed [`Q_WIDTH-1:0]  cordic_y_out;
    wire signed [`Q_WIDTH-1:0]  cordic_z_out;
    wire                        cordic_done;

    wire is_vectoring_w = (state == S_EML_WAIT_LN_COR);

    wire signed [`Q_WIDTH-1:0] cordic_x_in_w =
        (state == S_EML_WAIT_LN_COR) ? (reg_work_0 + `FP_ONE) :
        (state == S_EML_CORDIC_EXP) ? `CORDIC_INV_GAIN_HYP :
        INT_ZERO;

    wire signed [`Q_WIDTH-1:0] cordic_y_in_w =
        (state == S_EML_WAIT_LN_COR) ? (reg_work_0 - `FP_ONE) :
        INT_ZERO;

    wire signed [`Q_WIDTH-1:0] cordic_z_in_w =
        (state == S_EML_CORDIC_EXP) ? (reg_x - mul_result) :
        INT_ZERO;

    cordic_hyp u_shared_cordic (
        .clk(clk), .rst_n(rst_n), .start(cordic_start_r),
        .is_vectoring_in(is_vectoring_w),
        .x_in(cordic_x_in_w), .y_in(cordic_y_in_w), .z_in(cordic_z_in_w),
        .x_out(cordic_x_out), .y_out(cordic_y_out), .z_out(cordic_z_out),
        .done(cordic_done)
    );

    wire signed [`Q_WIDTH:0] exp_sum_wide =
        $signed({cordic_x_out[`Q_WIDTH-1], cordic_x_out}) +
        $signed({cordic_y_out[`Q_WIDTH-1], cordic_y_out});

    wire signed [`Q_WIDTH:0] ln_full_wide =
        ($signed({cordic_z_out[`Q_WIDTH-1], cordic_z_out}) <<< 1) +
        $signed({mul_result[`Q_WIDTH-1], mul_result});

    wire signed [`Q_WIDTH:0] final_result_wide =
        $signed({reg_work_1[`Q_WIDTH-1], reg_work_1}) -
        $signed({reg_work_0[`Q_WIDTH-1], reg_work_0});

    wire signed [`Q_WIDTH-1:0] exp_k_rounded = reg_work_1 + (20'sd1 <<< (`Q_FRAC-1));
    wire signed [`Q_WIDTH-1:0] exp_k_shifted = exp_k_rounded >>> `Q_FRAC;

    wire signed [5:0] k_s = reg_k;
    wire [5:0] neg_k_s = -k_s;
    wire [4:0] k_abs = (k_s >= 0) ? k_s[4:0] : neg_k_s[4:0];

    wire x_is_pos_inf = (x_in == `FP_POS_INF);
    wire x_is_neg_inf = (x_in == `FP_NEG_INF);
    wire y_is_pos_inf = (y_in == `FP_POS_INF);
    wire y_is_nan     = (y_in == `FP_NAN_VAL);
    wire x_is_nan     = (x_in == `FP_NAN_VAL);

    function signed [`Q_WIDTH-1:0] saturate;
        input signed [`Q_WIDTH:0] value;
        begin
            if (value >= $signed({1'b0, `FP_POS_INF}))
                saturate = `FP_POS_INF;
            else if (value <= $signed({1'b1, `FP_NEG_INF}))
                saturate = `FP_NEG_INF;
            else
                saturate = value[`Q_WIDTH-1:0];
        end
    endfunction

    assign result           = reg_work_1;
    assign done   = (state == S_DONE);
    assign busy   = (state != S_IDLE);

    wire _unused = &{exp_sum_wide[`Q_WIDTH], ln_full_wide[`Q_WIDTH],
                     exp_k_shifted[`Q_WIDTH-1:8], neg_k_s[5], 1'b0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            error          <= 1'b0;
            domain_error   <= 1'b0;
            overflow       <= 1'b0;
            mul_start_r    <= 1'b0;
            cordic_start_r <= 1'b0;
        end else begin
            mul_start_r    <= 1'b0;
            cordic_start_r <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        reg_x        <= x_in;
                        reg_work_0   <= y_in;
                        reg_k        <= 0;
                        domain_error <= 1'b0;
                        overflow     <= 1'b0;
                        error        <= 1'b0;
                        reg_work_1   <= INT_ZERO;

                        case (opcode)
                            OP_MUL: begin
                                mul_start_r <= 1'b1;
                                state       <= S_WAIT_MUL;
                            end
                            default: begin
                                if (x_is_nan || y_is_nan) begin
                                    reg_work_1 <= `FP_NAN_VAL;
                                    state <= S_DONE;
                                end else if (y_in <= `FP_ZERO) begin
                                    reg_work_1   <= `FP_POS_INF;
                                    domain_error <= (y_in != `FP_ZERO);
                                    state        <= S_DONE;
                                end else if (x_is_pos_inf && y_is_pos_inf) begin
                                    reg_work_1   <= `FP_NAN_VAL;
                                    domain_error <= 1'b1;
                                    state        <= S_DONE;
                                end else if (x_is_pos_inf) begin
                                    reg_work_1 <= `FP_POS_INF;
                                    state <= S_DONE;
                                end else if (y_is_pos_inf) begin
                                    reg_work_1 <= `FP_NEG_INF;
                                    state <= S_DONE;
                                end else if (x_is_neg_inf) begin
                                    reg_x      <= INT_NEG_TEN;
                                    reg_work_0 <= y_in;
                                    reg_k      <= 6'sd0;
                                    reg_work_1 <= INT_NEG_TEN;
                                    state      <= S_EML_NORM;
                                end else begin
                                    reg_k       <= 6'sd0;
                                    mul_start_r <= 1'b1;
                                    state       <= S_EML_SCALE_X;
                                end
                            end
                        endcase
                    end
                end

                S_WAIT_MUL: begin
                    if (mul_done) begin
                        reg_work_1 <= mul_result;
                        state      <= S_DONE;
                    end
                end

                S_EML_SCALE_X: begin
                    if (mul_done) begin
                        reg_work_1 <= mul_result;
                        state      <= S_EML_NORM;
                    end
                end

                S_EML_NORM: begin

                    if (reg_work_0 > 20'sd0 && reg_work_0 < 20'sd8192) begin
                        reg_work_0 <= reg_work_0 <<< 1;
                        reg_k      <= reg_k - 6'sd1;
                    end else if (reg_work_0 >= 20'sd16384) begin
                        reg_work_0 <= reg_work_0 >>> 1;
                        reg_k      <= reg_k + 6'sd1;
                    end else begin
                        mul_start_r <= 1'b1;
                        state       <= S_EML_WAIT_LN_MUL;
                    end
                end

                S_EML_WAIT_LN_MUL: begin
                    if (mul_done) begin
                        cordic_start_r <= 1'b1;
                        state          <= S_EML_WAIT_LN_COR;
                    end
                end

                S_EML_WAIT_LN_COR: begin
                    if (cordic_done) begin
                        reg_work_0 <= ln_full_wide[`Q_WIDTH-1:0];
                        reg_k      <= exp_k_shifted[5:0];
                        state      <= S_EML_MUL_EXP;
                    end
                end

                S_EML_MUL_EXP: begin
                    mul_start_r <= 1'b1;
                    state       <= S_EML_PREP_CORDIC;
                end

                S_EML_PREP_CORDIC: begin
                    if (mul_done) begin
                        if (reg_x < -20'sd98304) begin
                            reg_work_1 <= `FP_ZERO;
                            state      <= S_EML_FINISH;
                        end else if (reg_x > 20'sd98304) begin
                            reg_work_1 <= `FP_POS_INF;
                            state      <= S_EML_FINISH;
                        end else begin
                            cordic_start_r <= 1'b1;
                            state          <= S_EML_CORDIC_EXP;
                        end
                    end
                end

                S_EML_CORDIC_EXP: begin
                    if (cordic_done) begin
                        if (k_s >= 6'sd12) begin
                            reg_work_1 <= `FP_POS_INF;
                            shift_cnt  <= 0;
                            state      <= S_EML_FINISH;
                        end else if (k_s <= -6'sd12) begin
                            reg_work_1 <= `FP_ZERO;
                            shift_cnt  <= 0;
                            state      <= S_EML_FINISH;
                        end else begin
                            reg_work_1 <= exp_sum_wide[`Q_WIDTH-1:0];
                            shift_cnt  <= k_abs;
                            state      <= S_EML_EXP_SHIFT;
                        end
                    end
                end

                S_EML_EXP_SHIFT: begin
                    if (shift_cnt == 0) begin
                        state <= S_EML_FINISH;
                    end else begin
                        if (k_s >= 0)
                            reg_work_1 <= reg_work_1 <<< 1;
                        else
                            reg_work_1 <= reg_work_1 >>> 1;
                        shift_cnt <= shift_cnt - 1;
                    end
                end

                S_EML_FINISH: begin
                    reg_work_1 <= saturate(final_result_wide);
                    overflow   <= (final_result_wide > $signed({1'b0, `FP_POS_INF})) ||
                                  (final_result_wide < $signed({1'b1, `FP_NEG_INF}));
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
