`ifndef FP_PKG_VH
`define FP_PKG_VH

`define Q_INT    6
`define Q_FRAC   10
`define Q_WIDTH  (`Q_INT + `Q_FRAC)

// Internal 24-bit precision
`define Q_INT_I    12
`define Q_FRAC_I   12
`define Q_WIDTH_I  (`Q_INT_I + `Q_FRAC_I)

`define FP_ZERO      16'sd0
`define FP_ONE       16'sd1024
`define FP_TWO       16'sd2048
`define FP_HALF      16'sd512
`define FP_LN2       16'sd710
`define FP_INV_LN2   16'sd1477

`define FP_SHIFT_SAT_POS   16'sd16383
`define FP_SHIFT_SAT_NEG  -16'sd16384
`define FP_POS_MAX         16'sd32767
`define FP_NEG_MAX        -16'sd32768

// Special values for Q6.10 I/O
`define FP_POS_INF         16'h7FFF
`define FP_NEG_INF         16'h8001
`define FP_NAN_VAL         16'h7FFE

`define CORDIC_N  12
`define CORDIC_INV_GAIN_HYP   16'sd1236

// Internal precision constants
`define FP_ONE_I           24'sd4096
`define FP_LN2_I           24'sd2839
`define FP_INV_LN2_I       24'sd5909
`define CORDIC_N_I         18
`define CORDIC_INV_GAIN_HYP_I 24'sd4946

`endif
