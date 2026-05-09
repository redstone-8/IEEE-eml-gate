/* EML Package Header */
`ifndef FP_PKG_VH
`define FP_PKG_VH

`define Q_INT    6
`define Q_FRAC   14
`define Q_WIDTH  (`Q_INT + `Q_FRAC)

`define CORDIC_FRAC   14  
`define CORDIC_WIDTH  20  

`define FP_ZERO      20'sd0
`define FP_ONE       20'sd16384        
`define FP_TWO       20'sd32768        
`define FP_HALF      20'sd8192         
`define FP_LN2       20'sd11356        
`define FP_INV_LN2   20'sd23637        

`define FP_SHIFT_SAT_POS   20'sd262143
`define FP_SHIFT_SAT_NEG  -20'sd262144
`define FP_POS_MAX         20'sd524287
`define FP_NEG_MAX        -20'sd524288

`define FP_POS_INF         20'h7FFFF
`define FP_NEG_INF         20'h80001
`define FP_NAN_VAL         20'h7FFFE

`define CORDIC_N             14
`define CORDIC_INV_GAIN_HYP  20'sd19783  
`define CORDIC_INV_GAIN_CIRC 20'sd9949   

`endif
