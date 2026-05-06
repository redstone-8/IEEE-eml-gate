`include "fp_pkg.vh"

module eml_gate_top (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [3:0]                 func_id,
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,

    output wire signed [`Q_WIDTH-1:0] result,
    output wire                       done,
    output wire                       busy,
    output reg                        error,
    output reg                        domain_error,
    output reg                        overflow
);

    localparam [3:0] FUNC_RAW_EML = 4'd15;

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

    reg [3:0] state;

    reg signed [`Q_WIDTH_I-1:0] reg_x;
    reg signed [`Q_WIDTH_I-1:0] reg_work_0;
    reg signed [`Q_WIDTH_I-1:0] reg_work_1;
    reg signed [7:0]            reg_k;

    reg                         mul_start_r;
    reg  signed [`Q_WIDTH_I-1:0] mul_a_r;
    reg  signed [`Q_WIDTH_I-1:0] mul_b_r;
    wire signed [`Q_WIDTH_I-1:0] mul_result;
    wire                        mul_done;

    fp_mul_seq #(
        .WIDTH(`Q_WIDTH_I),
        .FRAC (`Q_FRAC_I)
    ) u_shared_mul (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (mul_start_r),
        .a      (mul_a_r),
        .b      (mul_b_r),
        .result (mul_result),
        .done   (mul_done)
    );

    reg                         cordic_start_r;
    reg  [1:0]                  cordic_mode_r;
    reg  signed [`Q_WIDTH_I-1:0] cordic_x_in_r;
    reg  signed [`Q_WIDTH_I-1:0] cordic_y_in_r;
    reg  signed [`Q_WIDTH_I-1:0] cordic_z_in_r;
    wire signed [`Q_WIDTH_I-1:0] cordic_x_out;
    wire signed [`Q_WIDTH_I-1:0] cordic_y_out;
    wire signed [`Q_WIDTH_I-1:0] cordic_z_out;
    wire                        cordic_done;

    cordic_hyp #(
        .WIDTH(`Q_WIDTH_I)
    ) u_shared_cordic (
        .clk   (clk),
        .rst_n (rst_n),
        .start (cordic_start_r),
        .mode  (cordic_mode_r),
        .x_in  (cordic_x_in_r),
        .y_in  (cordic_y_in_r),
        .z_in  (cordic_z_in_r),
        .x_out (cordic_x_out),
        .y_out (cordic_y_out),
        .z_out (cordic_z_out),
        .done  (cordic_done)
    );

    // Wide intermediate values
    wire signed [`Q_WIDTH_I+1:0] exp_sum_wide =
        $signed({cordic_x_out[`Q_WIDTH_I-1], cordic_x_out}) +
        $signed({cordic_y_out[`Q_WIDTH_I-1], cordic_y_out});

    wire signed [`Q_WIDTH_I+1:0] ln_full_wide =
        ($signed({cordic_z_out[`Q_WIDTH_I-1], cordic_z_out}) <<< 1) +
        $signed({mul_result[`Q_WIDTH_I-1], mul_result});

    wire signed [`Q_WIDTH_I+1:0] final_result_wide =
        $signed({reg_work_1[`Q_WIDTH_I-1], reg_work_1}) -
        $signed({reg_work_0[`Q_WIDTH_I-1], reg_work_0});

    wire signed [`Q_WIDTH_I+1:0] reg_work_1_wide =
        {{2{reg_work_1[`Q_WIDTH_I-1]}}, reg_work_1};

    wire signed [`Q_WIDTH_I-1:0] reg_k_scaled =
        $signed({{(`Q_WIDTH_I-8){reg_k[7]}}, reg_k}) <<< 12;

    wire signed [`Q_WIDTH_I-1:0] exp_k_rounded =
        reg_work_1[`Q_WIDTH_I-1]
            ? (reg_work_1 - (24'sd1 <<< 11))
            : (reg_work_1 + (24'sd1 <<< 11));

    wire signed [`Q_WIDTH_I-1:0] exp_k_shifted = exp_k_rounded >>> 12;
    wire _unused_wide = &{exp_sum_wide[`Q_WIDTH_I+1:`Q_WIDTH_I],
                          ln_full_wide[`Q_WIDTH_I+1:`Q_WIDTH_I],
                          exp_k_shifted[`Q_WIDTH_I-1:8], 1'b0};

    // Format conversion and special value detection
    wire x_is_pos_inf = (x_in == `FP_POS_INF);
    wire x_is_neg_inf = (x_in == `FP_NEG_INF);
    wire y_is_pos_inf = (y_in == `FP_POS_INF);
    wire y_is_nan     = (y_in == `FP_NAN_VAL);
    wire x_is_nan     = (x_in == `FP_NAN_VAL);

    function signed [`Q_WIDTH-1:0] internal_to_external;
        input signed [`Q_WIDTH_I+1:0] value;
        begin
            if (value[23:0] == {8'd0, `FP_NAN_VAL})
                internal_to_external = `FP_NAN_VAL;
            else if (value[23:0] == {8'd0, `FP_POS_INF})
                internal_to_external = `FP_POS_INF;
            else if (value[23:0] == {8'd0, `FP_NEG_INF})
                internal_to_external = `FP_NEG_INF;
            else if (value >= 26'sd131072) // 32.0 in Q12.12
                internal_to_external = `FP_POS_INF;
            else if (value <= -26'sd131072)
                internal_to_external = `FP_NEG_INF;
            else
                // Correctly map Q12.12 to Q6.10
                // Sign (25) -> Sign (15)
                // Int [16:12] -> Int [14:10]
                // Frac [11:2] -> Frac [9:0]
                internal_to_external = { value[25], value[16:12], value[11:2] };
        end
    endfunction

    assign result = internal_to_external(reg_work_1_wide);
    assign done   = (state == S_DONE);
    assign busy   = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            error           <= 1'b0;
            domain_error    <= 1'b0;
            overflow        <= 1'b0;
            mul_start_r     <= 1'b0;
            cordic_start_r  <= 1'b0;
            reg_work_1      <= 24'sd0;
            reg_work_0      <= 24'sd0;
            reg_x           <= 24'sd0;
            reg_k           <= 8'sd0;
        end else begin
            mul_start_r     <= 1'b0;
            cordic_start_r  <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        domain_error <= 1'b0;
                        overflow     <= 1'b0;
                        error        <= 1'b0;
                        reg_work_1   <= 24'sd0;
                        if (func_id != FUNC_RAW_EML) begin
                            reg_work_1 <= 24'sd0;
                            error    <= 1'b1;
                            state    <= S_DONE;
                        end else if (x_is_nan || y_is_nan) begin
                            reg_work_1 <= {8'd0, `FP_NAN_VAL};
                            state <= S_DONE;
                        end else if (y_in <= `FP_ZERO) begin
                            // ln(0) or ln(neg)
                            if (y_in == `FP_ZERO) begin
                                // exp(x) - (-inf) = +inf
                                reg_work_1 <= {8'd0, `FP_POS_INF};
                            end else begin
                                reg_work_1 <= {8'd0, `FP_POS_INF};
                                domain_error <= 1'b1;
                            end
                            state <= S_DONE;
                        end else if (x_is_pos_inf && y_is_pos_inf) begin
                            // inf - inf = NaN
                            reg_work_1 <= {8'd0, `FP_NAN_VAL};
                            domain_error <= 1'b1;
                            state <= S_DONE;
                        end else if (x_is_pos_inf) begin
                            reg_work_1 <= {8'd0, `FP_POS_INF};
                            state <= S_DONE;
                        end else if (y_is_pos_inf) begin
                            reg_work_1 <= {8'd0, `FP_NEG_INF};
                            state <= S_DONE;
                        end else if (x_is_neg_inf) begin
                            // exp(-inf) - ln(y) = 0 - ln(y)
                            reg_x <= -24'sd40960; // -10.0 in Q12.12
                            reg_work_0 <= $signed({ {6{y_in[15]}}, y_in, 2'b0 });
                            reg_k <= 8'sd0;
                            reg_work_1 <= -24'sd40960; // -10.0 scaled
                            state <= S_EML_NORM; // skip scaling x
                        end else begin
                            reg_x      <= $signed({ {6{x_in[15]}}, x_in, 2'b0 });
                            reg_work_0 <= $signed({ {6{y_in[15]}}, y_in, 2'b0 });
                            reg_k      <= 8'sd0;
                            mul_a_r    <= $signed({ {6{x_in[15]}}, x_in, 2'b0 });
                            mul_b_r    <= `FP_INV_LN2_I;
                            mul_start_r<= 1'b1;
                            state      <= S_EML_SCALE_X;
                        end
                    end
                end

                S_EML_SCALE_X: begin
                    if (mul_done) begin
                        reg_work_1 <= mul_result; // k_exp scaled
                        state      <= S_EML_NORM;
                    end
                end

                S_EML_NORM: begin
                    if (reg_work_0 >= (24'sd2 <<< 12)) begin
                        reg_work_0 <= reg_work_0 >>> 1;
                        reg_k      <= reg_k + 8'sd1;
                    end else if (reg_work_0 < `FP_ONE_I) begin
                        reg_work_0 <= reg_work_0 <<< 1;
                        reg_k      <= reg_k - 8'sd1;
                    end else begin
                        mul_a_r       <= reg_k_scaled;
                        mul_b_r       <= `FP_LN2_I;
                        mul_start_r   <= 1'b1;
                        state         <= S_EML_WAIT_LN_MUL;
                    end
                end

                S_EML_WAIT_LN_MUL: begin
                    if (mul_done) begin
                        cordic_mode_r <= 2'b01;
                        cordic_x_in_r <= reg_work_0 + `FP_ONE_I;
                        cordic_y_in_r <= reg_work_0 - `FP_ONE_I;
                        cordic_z_in_r <= 24'sd0;
                        cordic_start_r<= 1'b1;
                        state         <= S_EML_WAIT_LN_COR;
                    end
                end

                S_EML_WAIT_LN_COR: begin
                    if (cordic_done) begin
                        reg_work_0 <= ln_full_wide[`Q_WIDTH_I-1:0];
                        // Calculate k_exp
                        reg_k <= exp_k_shifted[7:0];
                        state <= S_EML_MUL_EXP;
                    end
                end

                S_EML_MUL_EXP: begin
                    mul_a_r    <= reg_k_scaled;
                    mul_b_r    <= `FP_LN2_I;
                    mul_start_r<= 1'b1;
                    state      <= S_EML_PREP_CORDIC;
                end

                S_EML_PREP_CORDIC: begin
                    if (mul_done) begin
                        cordic_mode_r <= 2'b00;
                        cordic_x_in_r <= `CORDIC_INV_GAIN_HYP_I;
                        cordic_y_in_r <= 24'sd0;
                        cordic_z_in_r <= reg_x - mul_result;
                        cordic_start_r<= 1'b1;
                        state         <= S_EML_CORDIC_EXP;
                    end
                end

                S_EML_CORDIC_EXP: begin
                    if (cordic_done) begin
                        reg_work_1 <= exp_sum_wide[`Q_WIDTH_I-1:0];
                        state      <= S_EML_EXP_SHIFT;
                    end
                end

                S_EML_EXP_SHIFT: begin
                    if (reg_k > 0) begin
                        if (reg_work_1 > (24'sd1 <<< (24-2))) reg_work_1 <= (24'sd1 <<< (24-1)) - 1;
                        else reg_work_1 <= reg_work_1 <<< 1;
                        reg_k <= reg_k - 8'sd1;
                    end else if (reg_k < 0) begin
                        reg_work_1 <= reg_work_1 >>> 1;
                        reg_k    <= reg_k + 8'sd1;
                    end else begin
                        state <= S_EML_FINISH;
                    end
                end

                S_EML_FINISH: begin
                    reg_work_1 <= final_result_wide[`Q_WIDTH_I-1:0];
                    overflow   <= (internal_to_external(final_result_wide) == `FP_POS_INF) || (internal_to_external(final_result_wide) == `FP_NEG_INF);
                    state    <= S_DONE;
                end

                S_DONE: begin
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
            if (start && (state != S_IDLE)) error <= 1'b1;
        end
    end
endmodule
