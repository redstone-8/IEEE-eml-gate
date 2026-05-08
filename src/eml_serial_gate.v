`include "fp_pkg.vh"

module eml_spi_gate (
    input  wire clk,
    input  wire rst_n,

    // SPI Interface (Mode 0)
    input  wire mosi,
    input  wire sclk,
    input  wire cs_n,
    output wire miso,

    // Status
    output wire busy,
    output wire done,
    output reg  error
);

    // Synchronizers
    reg [2:0] sclk_sync;
    reg [2:0] cs_n_sync;
    reg [1:0] mosi_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync <= 3'b0;
            cs_n_sync <= 3'b111; // cs_n is active low, idle high
            mosi_sync <= 2'b0;
        end else begin
            sclk_sync <= {sclk_sync[1:0], sclk};
            cs_n_sync <= {cs_n_sync[1:0], cs_n};
            mosi_sync <= {mosi_sync[0], mosi};
        end
    end

    wire sclk_rise = (sclk_sync[2:1] == 2'b01);
    wire sclk_fall = (sclk_sync[2:1] == 2'b10);
    wire cs_n_active = ~cs_n_sync[1];
    wire cs_n_rise = (cs_n_sync[2:1] == 2'b01);

    // Shift Register
    reg [39:0] shift_reg;
    reg        miso_reg;
    reg        start_reg;

    // Gate outputs
    wire signed [`Q_WIDTH-1:0] gate_result;
    wire signed [`Q_WIDTH-1:0] gate_secondary;
    wire gate_done, gate_busy, gate_error, gate_domain_error, gate_overflow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 40'd0;
            miso_reg  <= 1'b0;
            start_reg <= 1'b0;
            error     <= 1'b0;
        end else begin
            start_reg <= 1'b0;
            
            if (gate_done) begin
                // Load result when computation finishes
                shift_reg <= {
                    1'b0, // bit 39
                    gate_error | error, // bit 38 (accumulate protocol error if any)
                    gate_domain_error, // bit 37
                    gate_overflow, // bit 36
                    4'b0, // bits 35:32
                    gate_result, // bits 31:16
                    gate_secondary // bits 15:0
                };
            end else if (cs_n_active) begin
                if (sclk_rise) begin
                    // Shift in MOSI on SCLK rising edge
                    shift_reg <= {shift_reg[38:0], mosi_sync[1]};
                end
            end
            
            if (cs_n_active) begin
                if (sclk_fall) begin
                    // Update MISO on SCLK falling edge
                    miso_reg <= shift_reg[39];
                end
            end else begin
                // Pre-load MISO for the first bit when CS_N goes low
                miso_reg <= shift_reg[39];
            end
            
            if (cs_n_rise) begin
                if (shift_reg[39] == 1'b1) begin // RW = 1 means Start
                    if (gate_busy) begin
                        error <= 1'b1; // Protocol error: tried to start while busy
                    end else begin
                        start_reg <= 1'b1;
                        error <= 1'b0; // Clear error on successful start
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
        .opcode       (shift_reg[33:32]),
        .x_in         (shift_reg[31:16]),
        .y_in         (shift_reg[15:0]),
        .result       (gate_result),
        .result_secondary (gate_secondary),
        .done         (gate_done),
        .busy         (gate_busy),
        .error        (gate_error),
        .domain_error (gate_domain_error),
        .overflow     (gate_overflow)
    );

endmodule
