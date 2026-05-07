`include "fp_pkg.vh"

module eml_serial_gate (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ser_in,
    input  wire       shift_en,
    input  wire       start,

    output wire       ser_out,
    output wire       busy,
    output wire       done,
    output reg        error,
    output wire       rx_full,
    output wire       tx_pending
);

    localparam [7:0] SOF_BYTE = 8'hA5;

    localparam S_IDLE     = 1'b0;
    localparam S_EVAL_EML = 1'b1;

    reg       state_reg;

    reg [6:0] rx_byte_shift_reg;
    reg [2:0] rx_bit_count_reg;
    reg [15:0] op_a_reg, op_b_reg;
    reg [2:0] rx_byte_count_reg;
    reg       frame_ready_reg;

    reg [7:0]  tx_byte_shift_reg;
    reg [2:0]  tx_bit_count_reg;
    reg [2:0]  tx_byte_idx_reg;
    reg        tx_pending_reg;
    reg        done_reg;

    wire gate_start_w;
    wire signed [`Q_WIDTH-1:0] gate_result;
    wire gate_done;
    wire gate_busy;
    wire gate_error;
    wire gate_domain_error;
    wire gate_overflow;

    wire launch_eval = (state_reg == S_IDLE) && start && frame_ready_reg && !shift_en && !tx_pending_reg;

    assign ser_out    = tx_pending ? tx_byte_shift_reg[7] : 1'b0;
    assign busy       = state_reg || gate_busy;
    assign done       = done_reg;
    assign rx_full    = frame_ready_reg;
    assign tx_pending = tx_pending_reg;
    assign gate_start_w = launch_eval;

    eml_gate_top u_eml_gate_top (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (gate_start_w),
        .x_in         (op_a_reg[`Q_WIDTH-1:0]),
        .y_in         (op_b_reg[`Q_WIDTH-1:0]),
        .result       (gate_result),
        .done         (gate_done),
        .busy         (gate_busy),
        .error        (gate_error),
        .domain_error (gate_domain_error),
        .overflow     (gate_overflow)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg        <= S_IDLE;
            rx_byte_shift_reg<= 7'd0;
            rx_bit_count_reg <= 3'd0;
            rx_byte_count_reg<= 3'd0;
            frame_ready_reg  <= 1'b0;
            tx_byte_shift_reg<= 8'd0;
            tx_bit_count_reg <= 3'd0;
            tx_byte_idx_reg  <= 3'd0;
            tx_pending_reg <= 1'b0;
            done_reg       <= 1'b0;
            error          <= 1'b0;
            op_a_reg   <= 16'd0;
            op_b_reg   <= 16'd0;
        end else begin
            done_reg <= 1'b0;
            if (shift_en && tx_pending_reg) begin
                if (tx_bit_count_reg == 3'd0) begin
                    if (tx_byte_idx_reg == 3'd4) begin
                        tx_pending_reg <= 1'b0;
                    end else begin
                        tx_byte_idx_reg <= tx_byte_idx_reg + 3'd1;
                        tx_bit_count_reg <= (tx_byte_idx_reg == 3'd3) ? 3'd3 : 3'd7;
                        case (tx_byte_idx_reg)
                            3'd0: tx_byte_shift_reg <= op_a_reg[15:8];
                            3'd1: tx_byte_shift_reg <= op_a_reg[7:0];
                            3'd2: tx_byte_shift_reg <= op_b_reg[15:8];
                            3'd3: tx_byte_shift_reg <= op_b_reg[7:0];
                            default: tx_byte_shift_reg <= 8'd0;
                        endcase
                    end
                end else begin
                    tx_byte_shift_reg <= {tx_byte_shift_reg[6:0], 1'b0};
                    tx_bit_count_reg <= tx_bit_count_reg - 3'd1;
                end
            end else if (shift_en) begin
                if (start || busy) begin
                    error <= 1'b1;
                end else begin
                    rx_byte_shift_reg <= {rx_byte_shift_reg[5:0], ser_in};
                    if (rx_bit_count_reg == 3'd7) begin
                        rx_bit_count_reg <= 3'd0;
                        if ({rx_byte_shift_reg[6:0], ser_in} == SOF_BYTE
                            && (rx_byte_count_reg == 3'd0 || frame_ready_reg)) begin
                            rx_byte_count_reg <= 3'd0;
                            frame_ready_reg   <= 1'b0;
                        end else if (!frame_ready_reg) begin
                            case (rx_byte_count_reg)
                                3'd0: op_a_reg[15:8] <= {rx_byte_shift_reg[6:0], ser_in};
                                3'd1: op_a_reg[7:0]  <= {rx_byte_shift_reg[6:0], ser_in};
                                3'd2: op_b_reg[15:8] <= {rx_byte_shift_reg[6:0], ser_in};
                                3'd3: begin
                                    op_b_reg[7:0]   <= {rx_byte_shift_reg[6:0], ser_in};
                                    frame_ready_reg <= 1'b1;
                                end
                                default: error <= 1'b1;
                            endcase
                            rx_byte_count_reg <= rx_byte_count_reg + 3'd1;
                        end else begin
                            error <= 1'b1;
                        end
                    end else begin
                        rx_bit_count_reg <= rx_bit_count_reg + 3'd1;
                    end
                end
            end

            case (state_reg)
                S_IDLE: begin
                    if (launch_eval) begin
                        error <= 1'b0;
                        frame_ready_reg     <= 1'b0;
                        rx_bit_count_reg    <= 3'd0;
                        rx_byte_count_reg   <= 3'd0;
                        state_reg <= S_EVAL_EML;
                    end else if (start) begin
                        error <= 1'b1;
                    end
                end

                S_EVAL_EML: begin
                    if (gate_done) begin
                        tx_byte_shift_reg <= { (gate_error | error), gate_domain_error, gate_overflow, gate_result[15:11] };
                        
                        op_a_reg <= { gate_result[10:0], 5'b0 };
                        op_b_reg <= 16'd0;
                        
                        tx_bit_count_reg <= 3'd7;
                        tx_byte_idx_reg <= 3'd0;
                        tx_pending_reg <= 1'b1;
                        done_reg       <= 1'b1;
                        state_reg <= S_IDLE;
                    end
                end

                default: state_reg <= S_IDLE;
            endcase

            if (start && shift_en) begin
                error <= 1'b1;
            end

            if (state_reg == S_IDLE && !start && !shift_en && !tx_pending_reg) begin
                if (error && !frame_ready_reg)
                    error <= 1'b0;
            end

        end
    end

endmodule
