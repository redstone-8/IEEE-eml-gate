`ifndef FP_PKG_VH
`define FP_PKG_VH

// ── Q6.10 fixed-point format (unified, no internal upscaling) ──
`define Q_INT    6
`define Q_FRAC   10
`define Q_WIDTH  (`Q_INT + `Q_FRAC)

// Basic constants in Q6.10
`define FP_ZERO      16'sd0
`define FP_ONE       16'sd1024        // 1.0
`define FP_TWO       16'sd2048        // 2.0
`define FP_HALF      16'sd512         // 0.5
`define FP_LN2       16'sd710         // ln(2) ≈ 0.6931 × 1024
`define FP_INV_LN2   16'sd1477        // 1/ln(2) ≈ 1.4427 × 1024

// Saturation / limits
`define FP_SHIFT_SAT_POS   16'sd16383
`define FP_SHIFT_SAT_NEG  -16'sd16384
`define FP_POS_MAX         16'sd32767
`define FP_NEG_MAX        -16'sd32768

// Special sentinel values for Q6.10 I/O
`define FP_POS_INF         16'h7FFF
`define FP_NEG_INF         16'h8001
`define FP_NAN_VAL         16'h7FFE

// CORDIC configuration
`define CORDIC_N             12
`define CORDIC_INV_GAIN_HYP  16'sd1234   // 1/K_hyp ≈ 1.2051 × 1024
`define CORDIC_INV_GAIN_CIRC 16'sd622    // 1/K_circ ≈ 0.6073 × 1024

`endif
