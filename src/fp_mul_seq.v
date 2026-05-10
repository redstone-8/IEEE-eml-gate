`include "fp_pkg.vh"

module fp_mul_seq #(
    parameter WIDTH = `Q_WIDTH,
    parameter FRAC  = `Q_FRAC
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [WIDTH-1:0] a,
    input  wire signed [WIDTH-1:0] b,
    output wire signed [WIDTH-1:0] result,
    output wire               done
);

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    (* fsm_encoding = "binary" *) reg [1:0] state;

    reg [4:0] count;
    localparam [4:0] LAST_COUNT = WIDTH - 1;

    reg signed [WIDTH:0]   p_reg;
    reg signed [WIDTH-1:0] a_reg;
    reg signed [WIDTH-1:0] b_reg;

    localparam signed [WIDTH-1:0] SAT_POS = {1'b0, {(WIDTH-1){1'b1}}};
    localparam signed [WIDTH-1:0] SAT_NEG = {1'b1, {(WIDTH-1){1'b0}}};

    wire signed [WIDTH:0] p_plus_a = p_reg + {a_reg[WIDTH-1], a_reg};

    localparam LSHIFT = WIDTH - 1 - FRAC;

    wire signed [WIDTH:0] p_result = p_reg <<< LSHIFT;

    wire p_overflow = (p_result[WIDTH:WIDTH-1] != {2{p_result[WIDTH-1]}});

    assign result = p_overflow ? (p_reg[WIDTH] ? SAT_NEG : SAT_POS)
                               : p_result[WIDTH-1:0];
    assign done   = (state == S_DONE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            count <= 5'd0;
            p_reg <= {(WIDTH+1){1'b0}};
            a_reg <= {WIDTH{1'b0}};
            b_reg <= {WIDTH{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        a_reg <= a;
                        b_reg <= b;
                        p_reg <= 0;
                        count <= 5'd0;
                        state <= S_CALC;
                    end
                end

                S_CALC: begin
                    if (count == LAST_COUNT) begin
                        if (b_reg[0])
                            p_reg <= p_reg - {a_reg[WIDTH-1], a_reg};
                        state <= S_DONE;
                    end else begin
                        if (b_reg[0])
                            p_reg <= p_plus_a >>> 1;
                        else
                            p_reg <= p_reg >>> 1;
                        b_reg <= {b_reg[WIDTH-1], b_reg[WIDTH-1:1]};
                        count <= count + 5'd1;
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
