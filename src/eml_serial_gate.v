`include "fp_pkg.vh"

module eml_spi_gate (
    input  wire clk,
    input  wire rst_n,

    input  wire mosi,
    input  wire sclk,
    input  wire cs_n,
    output wire miso,

    output wire busy,
    output wire done,
    output reg  error
);

    reg [1:0] sclk_sync;
    reg [1:0] cs_n_sync;
    reg [1:0] mosi_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync <= 2'b0;
            cs_n_sync <= 2'b11;
            mosi_sync <= 2'b0;
        end else begin
            sclk_sync <= {sclk_sync[0], sclk};
            cs_n_sync <= {cs_n_sync[0], cs_n};
            mosi_sync <= {mosi_sync[0], mosi};
        end
    end

    wire sclk_rise   = (sclk_sync == 2'b01);
    wire sclk_fall   = (sclk_sync == 2'b10);
    wire cs_n_active = ~cs_n_sync[0];
    wire cs_n_rise   = (cs_n_sync == 2'b01);

    reg [55:0] shift_reg;
    reg        miso_reg;
    reg        start_reg;

    wire signed [`Q_WIDTH-1:0] gate_result;
    wire gate_done, gate_busy, gate_error, gate_domain_error, gate_overflow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 56'd0;
            miso_reg  <= 1'b0;
            start_reg <= 1'b0;
            error     <= 1'b0;
        end else begin
            start_reg <= 1'b0;

            if (gate_done) begin
                shift_reg <= {
                    1'b0,
                    gate_error | error,
                    gate_domain_error,
                    gate_overflow,
                    4'b0,
                    {{(24-`Q_WIDTH){gate_result[`Q_WIDTH-1]}}, gate_result},
                    24'b0
                };
            end else if (cs_n_active) begin
                if (sclk_rise)
                    shift_reg <= {shift_reg[54:0], mosi_sync[1]};
            end

            if (cs_n_active) begin
                if (sclk_fall)
                    miso_reg <= shift_reg[55];
            end else begin
                miso_reg <= shift_reg[55];
            end

            if (cs_n_rise) begin
                if (shift_reg[55] == 1'b1) begin
                    if (gate_busy) begin
                        error <= 1'b1;
                    end else begin
                        start_reg <= 1'b1;
                        error     <= 1'b0;
                    end
                end
            end
        end
    end

    assign miso = miso_reg;
    assign busy = gate_busy;
    assign done = gate_done;

    eml_gate_top u_eml_gate_top (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start_reg),
        .opcode       (shift_reg[49:48]),
        .x_in         (shift_reg[43:24]),
        .y_in         (shift_reg[19:0]),
        .result       (gate_result),
        .done         (gate_done),
        .busy         (gate_busy),
        .error        (gate_error),
        .domain_error (gate_domain_error),
        .overflow     (gate_overflow)
    );

endmodule
