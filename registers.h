/*
 * registers.h
 *
 *  Created on: 2 May 2018
 *      Author: jigal
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

#define NUM_ELEMENTS_IN_VEC(v)      (sizeof(v)/sizeof(v[0]))

//
// Corresponding FPGA image:
// ~/Dropbox/work/postdoc/fpga_work/terasic/jigal_work/pid_2
//
// 1. PID H/W has ability to run continuously without S/W.
//    and can fallback to S/W only operation.
//


//#define SAMPLES_TRACE

#define HW_REGS_BASE ( ALT_LWFPGASLVS_OFST )
#define HW_REGS_SPAN ( 2*1024*1024 )  // 2MiB
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define AXI_FPGA_BASE 0xC0000000
#define AXI_FPGA_SPAN 0x00040000
#define AXI_FPGA_MASK (AXI_FPGA_SPAN - 1)


#define handle_error_en(en, msg) \
           do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#define handle_error(msg) \
           do { perror(msg); exit(EXIT_FAILURE); } while (0)



#define PID_REG_SPACE_BASE  0
#define PREDICTOR_HW_BASE   (0 + PID_REG_SPACE_BASE)
#define PREDICTOR_HW_BASE_2 (PREDICTOR_HW_BASE + sizeof(predictor_hw_regs_t))




struct predictor_hw_reg_i_config_s
{
    /* Bit [    0] */ unsigned int  sw_reset     :   1   ;    // "1" ==> S/W reset.
    /* Bit [    1] */ unsigned int  continuous   :   1   ;    // "1" ==> continous, "0" ==> S/W mode.
    /* Bit [    2] */ unsigned int  input_select :   1   ;    // "0/1" ==> A/B inputs
    /* Bit [    3] */ unsigned int  pre_dither_manual_enable
                                                 :   1   ;    // "0"   ==> normal input to dither output (= output of pid).
                                                              // "1"   ==> manual input to dither output (= manual register value, pid result is discarded)
    /* Bits[ 8: 4] */ unsigned int  output_precision_size
                                                 :   5   ;    // # of bits in high precision PWM DAC shaper
                                                              // (essentially, the fractional # of bits to consider)

    /* Bit [    9] */ unsigned int  adc_align_enable
                                                 :   1   ;    // "1"   ==> input is taken from aligned ADC logic
                                                              // "0"   ==> input is taken straight from module input (the previous default).
    /* Bit [   10] */ unsigned int  do_delay     :   1   ;    // "1"   ==> do delay after predictor finished calculating its output.
                                                              //           the actual delay is delay_count*2 cycles of 200MHz.
                                                              //           See below the definition of the register.

    /* Bit [   11] */ unsigned int  manual_dac_output_enable
                                                 :   1   ;    // "1"   ==> o_z_n is set by manual_dac_output register
                                                              // "0"   ==> o_z_n is set by predictor output
    /* Bit [   12] */ unsigned int  invert_y_n   :   1   ;    // "1"   ==> invert y_n = y_input - y_reference
    /* Bit [   13] */ unsigned int  invert_output:   1   ;    // "1"   ==> invert o_z_n = ~ o_z_n
    /* Bit [   14] */ unsigned int  output_precision_enable
                                                 :   1   ;    // "1"   ==> high precision PWM DAC shaper is running
    /* Bit [   15] */ unsigned int  input_averaging_enable
                                                 :   1   ;    // "1"   ==> summing of input signals to enable higher
                                                              //           precision input.

    /* Bits[21:16] */ unsigned int  output_shift :   6   ;    // Shifting of the output (not high precision)
    /* Bits[23:22] */ unsigned int  kick_predictor:  1   ;    //
    /* Bits[23   ] */ unsigned int  reserved_23   :  1   ;    //
    /* Bits[28:24] */ unsigned int  output_precision_shift
                                                  :  5   ;    // Bit offset of the output high precision from the calculated
                                                              // predictor output.
    /* Bit[29]     */ unsigned int  y_input_diff_enable
                                                  :  1   ;    // Is predictor input a difference of the 2 ADC's?
                                                              // "1"  ===> predictor input is the difference between 2 ADC's.
                                                              // "0"  ===> predictor input is determined by the selected input ('input_select' above).
    /* Bit[30]     */ unsigned int  second_integrator_output_enable
                                                  :  1   ;    // "1"  ===> 2nd integrator output is enabled. Need to select between actual integrator
                                                              //           or just plain constant value given in another register.
    /* Bit[31]     */ unsigned int  second_integrator_enable
                                                  :  1   ;    // "1"  ===> 2nd integrator is enabled and accumulating (with its i2_gain).
                                                              // "0"  ===> 2nd integrator is not enabled - output a constant value.
};


struct predictor_hw_reg_o_config_s
{
    /* Bit [    0] */ unsigned int  reserved_0   :   1   ;
    /* Bit [    1] */ unsigned int  continuous   :   1   ;    // "1" ==> continous, "0" ==> S/W mode.
    /* Bit [    2] */ unsigned int  input_select :   1   ;    // "0/1" ==> A/B inputs
    /* Bit [    3] */ unsigned int  pre_dither_manual_enable
                                                 :   1   ;    // "0"   ==> normal input to dither output (= output of pid).
                                                              // "1"   ==> manual input to dither output (= manual register value, pid result is discarded)
    /* Bits[ 8: 4] */ unsigned int  output_precision_size
                                                 :   5   ;    // # of bits in high precision PWM DAC shaper
                                                              // (essentially, the fractional # of bits to consider)

    /* Bit [    9] */ unsigned int  adc_align_enable
                                                 :   1   ;    // "1"   ==> input is taken from aligned ADC logic
                                                              // "0"   ==> input is taken straight from module input (the previous default).
    /* Bit [   10] */ unsigned int  do_delay     :   1   ;    // "1"   ==> do delay after predictor finished calculating its output.
                                                              //           the actual delay is delay_count*2 cycles of 200MHz.
                                                              //           See below the definition of the register.

    /* Bit [   11] */ unsigned int  manual_dac_output_enable
                                                 :   1   ;    // "1"   ==> o_z_n is set by manual_dac_output register
                                                              // "0"   ==> o_z_n is set by predictor output
    /* Bit [   12] */ unsigned int  invert_y_n   :   1   ;    // "1"   ==> invert y_n = y_input - y_reference
    /* Bit [   13] */ unsigned int  invert_output:   1   ;    // "1"   ==> invert o_z_n = ~ o_z_n
    /* Bit [   14] */ unsigned int  output_precision_enable
                                                 :   1   ;    // "1"   ==> high precision PWM DAC shaper is running
    /* Bit [   15] */ unsigned int  input_averaging_enable
                                                 :   1   ;    // "1"   ==> summing of input signals to enable higher
                                                              //           precision input.

    /* Bits[21:16] */ unsigned int  output_shift :   6   ;    // Shifting of the output (not high precision)
    /* Bits[23:22] */ unsigned int  reserved_23_22:  2   ;    //
    /* Bits[28:24] */ unsigned int  output_precision_shift
                                                  :  5   ;    // Bit offset of the output high precision from the calculated
                                                              // predictor output.
    /* Bit[29]     */ unsigned int  y_input_diff_enable
                                                  :  1   ;    // Is predictor input a difference of the 2 ADC's?
                                                              // "1"  ===> predictor input is the difference between 2 ADC's.
                                                              // "0"  ===> predictor input is determined by the selected input ('input_select' above).
    /* Bit[30]     */ unsigned int  second_integrator_output_enable
                                                  :  1   ;    // "1"  ===> 2nd integrator output is enabled. Need to select between actual integrator
                                                              //           or just plain constant value given in another register.
    /* Bit[31]     */ unsigned int  second_integrator_enable
                                                  :  1   ;    // "1"  ===> 2nd integrator is enabled and accumulating (with its i2_gain).
                                                              // "0"  ===> 2nd integrator is not enabled - output a constant value.
};

//assign DAC_DA = (    (debug_output_dacb_config[ 0] == 1'b1) ?   ADC_DA
//                 : ( (debug_output_dacb_config[ 1] == 1'b1) ?   14'h2000
//                 : ( (debug_output_dacb_config[ 2] == 1'b1) ?   pid_predictor_hw_o_y_n    [13: 0]
//                 : ( (debug_output_dacb_config[ 3] == 1'b1) ?   pid_predictor2_hw_o_y_n_1 [13: 0]
//                 : ( (debug_output_dacb_config[ 4] == 1'b1) ?   pid_predictor2_hw_o_y_n_2 [13: 0]
//                 : ( (debug_output_dacb_config[ 5] == 1'b1) ?   pid_predictor_hw_o_y_n_3  [13: 0]
//                 : ( (debug_output_dacb_config[ 6] == 1'b1) ?   pid_predictor2_hw_o_y_n_4 [13: 0]
//                 : ( (debug_output_dacb_config[ 7] == 1'b1) ?   pid_predictor_hw_o_z_n    [13: 0]
//                 : ( (debug_output_dacb_config[ 8] == 1'b1) ?   pid_predictor2_hw_o_y_n_5 [13: 0]
//                 : ( (debug_output_dacb_config[ 9] == 1'b1) ?   pid_predictor_hw_o_count  [13: 0]
//                 : ( (debug_output_dacb_config[10] == 1'b1) ?   pid_predictor2_hw_o_y_n_6 [13: 0]
//                 : ( (debug_output_dacb_config[11] == 1'b1) ?   pid_predictor2_hw_o_y_n_7 [13: 0]
//                 : ( (debug_output_dacb_config[12] == 1'b1) ?   pid_predictor_hw_o_dither_input_counter[13:0]
//                 : ( (debug_output_dacb_config[13] == 1'b1) ?   pid_predictor_hw_o_z_n_no_integral[13:0]
//                 : ( (debug_output_dacb_config[14] == 1'b1) ?   pid_predictor_hw_o_y_input        [13:0]
//                 : ( (debug_output_dacb_config[15] == 1'b1) ?   pid_predictor_hw_o_integral_sum   [13:0]
//                 : ( (debug_output_dacb_config[16] == 1'b1) ?   14'b0
//                 : ( (debug_output_dacb_config[17] == 1'b1) ?   14'b0
//                 :   14'b0
//
//
struct dacb_output_config_s
{
    /* Bit [    0] */ unsigned int  adc_da                :   1   ;
    /* Bit [    1] */ unsigned int  pid_output            :   1   ;
    /* Bit [    2] */ unsigned int  predictor_y_n         :   1   ;
    /* Bit [    3] */ unsigned int  predictor_y_n_1       :   1   ;
    /* Bit [    4] */ unsigned int  predictor_y_n_2       :   1   ;
    /* Bit [    5] */ unsigned int  predictor_y_n_3       :   1   ;
    /* Bit [    6] */ unsigned int  predictor_y_n_4       :   1   ;
    /* Bit [    7] */ unsigned int  predictor_z_n         :   1   ;
    /* Bit [    8] */ unsigned int  predictor_y_n_5       :   1   ;
    /* Bit [    9] */ unsigned int  predictor_count       :   1   ;
    /* Bit [   10] */ unsigned int  predictor_y_n_6       :   1   ;
    /* Bit [   11] */ unsigned int  predictor_y_n_7       :   1   ;
    /* Bit [   12] */ unsigned int  dither_input_counter  :   1   ;
    /* Bit [   13] */ unsigned int  predictor_z_n_no_integral
                                                          :   1   ;
    /* Bit [   14] */ unsigned int  predictor_y_input     :   1   ;
    /* Bit [   15] */ unsigned int  predictor_integral_sum:   1   ;

    /* Bit [   16] */ unsigned int  zero_16               :   1   ;
    /* Bit [   17] */ unsigned int  zero_17               :   1   ;

    /* Bits[31:18] */ unsigned int  reserved_31_18       :   14   ;    // Upper 14 bits unused at the moment.
};


union predictor_hw_reg_i_config_u
{
    uint32_t                            _w32;
    struct predictor_hw_reg_i_config_s  c;
};

union predictor_hw_reg_o_config_u
{
    uint32_t                            _w32;
    struct predictor_hw_reg_o_config_s  c;
};

union dacb_output_config_u
{
    uint32_t                            _w32 ;
    struct dacb_output_config_s         c    ;
};



struct predictor2_hw_reg_i_dither_config_1_s
{
    /* Bit [15: 0] */ unsigned int  output_amplitude :   16   ;
    /* Bit [31:16] */ unsigned int  input_init_count :   16   ;
};

struct predictor2_hw_reg_o_dither_config_1_s
{
    /* Bit [15: 0] */ unsigned int  output_amplitude :   16   ;
    /* Bit [31:16] */ unsigned int  input_init_count :   16   ;
};

struct predictor2_hw_reg_i_dither_config_2_s
{
    /* Bit [15: 0] */ unsigned int  input_next_count :   16   ;
    /* Bit [31:16] */ unsigned int  output_init_count:   16   ;
};

struct predictor2_hw_reg_o_dither_config_2_s
{
    /* Bit [15: 0] */ unsigned int  input_next_count :   16   ;
    /* Bit [31:16] */ unsigned int  output_init_count:   16   ;
};

struct predictor2_hw_reg_i_dither_config_3_s
{
    /* Bit [28: 0] */ unsigned int  unused_28_0         :   29   ;
    /* Bit [   29] */ unsigned int  input_init_polarity :   1    ;
    /* Bit [   30] */ unsigned int  dither_output_enable:   1    ;
    /* Bit [   31] */ unsigned int  dither_input_enable :   1    ;
};

struct predictor2_hw_reg_o_dither_config_3_s
{
    /* Bit [28: 0] */ unsigned int  unused_28_0         :   29   ;
    /* Bit [   29] */ unsigned int  input_init_polarity :   1    ;
    /* Bit [   30] */ unsigned int  dither_output_enable:   1    ;
    /* Bit [   31] */ unsigned int  dither_input_enable :   1    ;
};

struct predictor2_hw_reg_o_dither_count_1_s
{
    /* Bit [15: 0] */ unsigned int  input_count         :   16   ;
    /* Bit [31:16] */ unsigned int  output_count        :   16   ;
};

struct predictor2_hw_reg_o_dither_count_2_s
{
    /* Bit [24: 0] */ unsigned int  unused_24_0         :   25   ;
    /* Bit [   25] */ unsigned int  input_polarity      :    1   ;
    /* Bit [   26] */ unsigned int  output_polarity     :    1   ;
    /* Bit [31:27] */ unsigned int  input_state         :    5   ;
};


struct predictor2_hw_reg_o_2nd_config_s
{
    /* Bit [15: 0] */ unsigned int  output_shift_2nd    :   16   ;  // Output shift of the 2nd integrator
    /* Bit [31:16] */ unsigned int  i0_shift            :   16   ;  // Extra shift of the 1st integrator before summing
                                                                    // the output with the other terms
                                                                    // (the predictive terms).
};


union config_2nd_u
{
    uint32_t                                _w32;
    struct predictor2_hw_reg_o_2nd_config_s c   ;
};

struct predictor2_hw_reg_o_3rd_config_s
{
    /* Bit [15: 0] */ unsigned int  i0_2nd              :   16   ;  // Gain of the 2nd integrator
    /* Bit [31:16] */ unsigned int  unused_31_16        :   16   ;
};

union config_3rd_u
{
    uint32_t                                _w32;
    struct predictor2_hw_reg_o_3rd_config_s c   ;
};



union dither_config_1
{
    uint32_t                                     _w32 ;
    struct predictor2_hw_reg_i_dither_config_1_s r    ;
    struct predictor2_hw_reg_o_dither_config_1_s w    ;
};

union dither_config_2
{
    uint32_t                                     _w32 ;
    struct predictor2_hw_reg_i_dither_config_2_s r    ;
    struct predictor2_hw_reg_o_dither_config_2_s w    ;
};

union dither_config_3
{
    uint32_t                                     _w32 ;
    struct predictor2_hw_reg_i_dither_config_3_s r    ;
    struct predictor2_hw_reg_o_dither_config_3_s w    ;
};

union dither_output_1
{
    uint32_t                                     _w32 ;
    struct predictor2_hw_reg_o_dither_count_1_s  r    ;
};

union dither_output_2
{
    uint32_t                                     _w32 ;
    struct predictor2_hw_reg_o_dither_count_2_s  r    ;
};


// //
// // DEPRECATED ALERT:
// // The current implementation does not use the slave template for templating 16 regs each.
// // Instead, it uses PIO core, which is a set of 4 register per register...
// // This is kept for reference.
// //
// typedef volatile struct predictor2_hw_regs_read_s
// {
//     /* [ 0] */ const volatile uint32_t  o_dither_config_1   ;
//     /* [ 1] */ const volatile uint32_t  o_dither_config_2   ;
//     /* [ 2] */ const volatile uint32_t  o_dither_config_3   ;
//     /* [ 3] */ const volatile uint32_t  o_dither_count_1    ;
//     /* [ 4] */ const volatile uint32_t  o_dither_count_2    ;
//     /* [ 5] */ const volatile uint32_t  o_dither_count_3    ;
//     /* [ 6] */ const volatile uint32_t  o_2nd_out_offset    ;
//     /* [ 7] */ const volatile uint32_t  o_2nd_config        ;
//     /* [ 8] */ const volatile uint32_t  o_3rd_config        ;
//     /* [ 9] */ const volatile uint32_t  o_y_n_4             ;
//     /* [10] */ const volatile uint32_t  o_y_n_5             ;
//     /* [11] */ const volatile uint32_t  o_y_n_6             ;
//     /* [12] */ const volatile uint32_t  o_y_n_7             ;
//     /* [13] */ const volatile uint32_t  o_2nd_output        ;
//     /* [14] */ const volatile uint32_t  o_manual_dac_output ;
//     /* [15] */ const volatile uint32_t  o_y_input           ;
// }   predictor2_hw_regs_read_t;


// typedef volatile struct predictor2_hw_regs_write_s
// {
//     /* [ 0] */ volatile uint32_t  i_dither_config_1     ;
//     /* [ 1] */ volatile uint32_t  i_dither_config_2     ;
//     /* [ 2] */ volatile uint32_t  i_dither_config_3     ;
//     /* [ 3] */ volatile uint32_t  i_unused_3            ;
//     /* [ 4] */ volatile uint32_t  i_unused_4            ;
//     /* [ 5] */ volatile uint32_t  i_unused_5            ;
//     /* [ 6] */ volatile uint32_t  i_2nd_out_offset      ;
//     /* [ 7] */ volatile uint32_t  i_2nd_config          ;
//     /* [ 8] */ volatile uint32_t  i_3rd_config          ;
//     /* [ 9] */ volatile uint32_t  i_unused_9            ;
//     /* [10] */ volatile uint32_t  i_unused_10           ;
//     /* [11] */ volatile uint32_t  i_unused_11           ;
//     /* [12] */ volatile uint32_t  i_unused_12           ;
//     /* [13] */ volatile uint32_t  i_2nd_output          ;
//     /* [14] */ volatile uint32_t  i_manual_dac_output   ;
//     /* [15] */ volatile uint32_t  i_unused_15           ;
// }   predictor2_hw_regs_write_t;




// typedef volatile struct predictor_hw_regs_read_s
// {
//     /* [ 0] */ const volatile uint32_t  o_y_n           ;
//     /* [ 1] */ const volatile uint32_t  o_q0_q4         ;
//     /* [ 2] */ const volatile uint32_t  o_q1_q5         ;
//     /* [ 3] */ const volatile uint32_t  o_q2_q6         ;
//     /* [ 4] */ const volatile uint32_t  o_q3_q7         ;
//     /* [ 5] */ const volatile uint32_t  o_config        ;
//     /* [ 6] */ const volatile uint32_t  o_y_reference   ;
//     /* [ 7] */ const volatile uint32_t  o_i0            ;
//     /* [ 8] */ const volatile uint32_t  o_z_n           ;
//     /* [ 9] */ const volatile uint32_t  o_count         ;
//     /* [10] */ const volatile uint32_t  o_y_n_3         ;
//     /* [11] */ const volatile uint32_t  o_delay_count   ;
//     /* [12] */ const volatile uint32_t  o_delay_counter ;
//     /* [13] */ const volatile uint32_t  o_out_offset    ;
//     /* [14] */ const volatile uint32_t  o_magic         ;
//     /* [15] */ const volatile uint32_t  o_dacb_output   ;
// }   predictor_hw_regs_read_t;


// typedef volatile struct predictor_hw_regs_write_s
// {
//     /* [ 0] */ volatile uint32_t  i_sw_yn       ;
//     /* [ 1] */ volatile uint32_t  i_q0_q4       ;
//     /* [ 2] */ volatile uint32_t  i_q1_q5       ;
//     /* [ 3] */ volatile uint32_t  i_q2_q6       ;   // q6 and q7 are not in use
//     /* [ 4] */ volatile uint32_t  i_q3_q7       ;   // only q0...q5
//     /* [ 5] */ volatile uint32_t  i_config      ;
//     /* [ 6] */ volatile uint32_t  i_y_reference ;
//     /* [ 7] */ volatile uint32_t  i_i0          ;
//     /* [ 8] */ volatile uint32_t  unused_08     ;
//     /* [ 9] */ volatile uint32_t  unused_09     ;
//     /* [10] */ volatile uint32_t  unused_10     ;
//     /* [11] */ volatile uint32_t  i_delay_count ;
//     /* [12] */ volatile uint32_t  unused_12     ;
//     /* [13] */ volatile uint32_t  i_out_offset  ;
//     /* [14] */ volatile uint32_t  unused_14     ;
//     /* [15] */ volatile uint32_t  i_dacb_output ;
// }   predictor_hw_regs_write_t;

// typedef union predictor_hw_regs_u
// {
//     struct predictor_hw_regs_write_s w;
//     struct predictor_hw_regs_read_s  r;
// }   predictor_hw_regs_t ;

// typedef union predictor2_hw_regs_u
// {
//     struct predictor2_hw_regs_write_s w;
//     struct predictor2_hw_regs_read_s  r;
// }   predictor2_hw_regs_t ;



typedef volatile union pid_hw_reg_i_config_u  pid_hw_reg_i_config_t ;

//extern predictor_hw_regs_t*   predictor_hw_regs ;
//extern predictor2_hw_regs_t*  predictor2_hw_regs ;


// Specification of PIO core registers (one for each PIO instance, and there are many...)
// Reference: https://www.intel.com/content/www/us/en/docs/programmable/683130/22-3/register-map-33025.html
typedef volatile struct pio_base_s
{
    uint32_t  data;         // 31:0 data, both for input (read) and output (write).
    uint32_t  direction;    // 31:0 direction for each pin - relevant for bi-directional lines (not our case).
    uint32_t  irq_mask;     // interrupt enable bit for each pio line - also, not in our case.
    uint32_t  edge;         // edge capture for each interrupt.
}   pio_base_t;

const static uint32_t PID_MAGIC = 0x11347698;



typedef struct pid_pio_regs_s
{
    /*   0  */ pio_base_t  pid_magic;          // input only, contains a magic value to indicate healthness.
    /*   1  */ pio_base_t  pid_version;        // input only, version of pid.
    /*   2  */ pio_base_t  pid_live_counter;   // input only, just running counter to indicate liveliness.
    /*   3  */ pio_base_t  pid_o_y_n;          // input only, the latest y_n input for the predictor.
    /*   4  */ pio_base_t  pid_o_q0_q4;        // q0 and q4 coefficients of the predictor.
    /*   5  */ pio_base_t  pid_o_q1_q5;        // q1 and q5 coefficients of the predictor.
    /*   6  */ pio_base_t  pid_o_q2_q6;        // q2 and q6 coefficients of the predictor.
    /*   7  */ pio_base_t  pid_o_q3_q7;        // q3 and q7 coefficients of the predictor.
    /*   8  */ pio_base_t  pid_o_config;       // pid config - see predictor_hw_reg_o_config_s.
    /*   9  */ pio_base_t  pid_o_y_reference;  // configure reference signal for y input (actual signal is y_n - y_ref);
    /*  10  */ pio_base_t  pid_o_i0;           // PID coefficient for the integration part.
    /*  11  */ pio_base_t  pid_o_z_n;          // predicted output
    /*  12  */ pio_base_t  pid_o_count;        // input only, # of times prediction was done
    /*  13  */ pio_base_t  pid_o_y_n_3;        // input only, input 3 time steps ago.
    /*  14  */ pio_base_t  pid_o_delay_count;  // # of iterations before doing next predictor cycle (if enabled).
    /*  15  */ pio_base_t  pid_o_delay_counter;// input only, delay counter before next iteration.
    /*  16  */ pio_base_t  pid_o_out_offset;   // reference for output signal (the output if the predicted value is 0).
    /*  17  */ pio_base_t  pid_o_magic;        // magic value from the predictor itself.
    /*  18  */ pio_base_t  pid_o_dacb_output;  // controls the value of DACB (i.e., the 2nd DAC, DAC #B) of the board. see dacb_output_config_s.
                                               // dithering config - note that 1 or more regs can have different format
                                               // for write and read.
    /*  19  */ pio_base_t  pid_o_dither_config_1; // dither config part 1, if dithering is used (predictor2_hw_reg_i_dither_config_1_s).
    /*  20  */ pio_base_t  pid_o_dither_config_2; // dither config part 2, if dithering is used (predictor2_hw_reg_i_dither_config_2_s)
    /*  21  */ pio_base_t  pid_o_dither_config_3; // dither config part 3, if dithering is used (predictor2_hw_reg_i_dither_config_3_s)
    /*  22  */ pio_base_t  pid_o_dither_count_1; // input only, dither counter #1.
    /*  23  */ pio_base_t  pid_o_dither_count_2; // input only, dither counter #2.
    /*  24  */ pio_base_t  pid_o_dither_count_3; // input only, dither counter #3 (see dither logic for what these counters mean)
    /*  25  */ pio_base_t  pid_o_2nd_out_offset; // reference for 2nd output signal (which mostly runs 2nd level integration)
    /*  26  */ pio_base_t  pid_o_2nd_config;    // pid 2nd configuration register (predictor2_hw_reg_o_2nd_config_s).
    /*  27  */ pio_base_t  pid_o_3rd_config;    // pid 3rd configuration register (predictor2_hw_reg_o_3rd_config_s).
    /*  28  */ pio_base_t  pid_o_y_n_4;         // input only, input 4 time steps ago.
    /*  29  */ pio_base_t  pid_o_y_n_5;         // input only, input 5 time steps ago.
    /*  30  */ pio_base_t  pid_o_y_n_6;         // input only, input 6 time steps ago.
    /*  31  */ pio_base_t  pid_o_y_n_7;         // input only, input 7 time steps ago.
    /*  32  */ pio_base_t  pid_o_2nd_output;    // input only, input 7 time steps ago.
    /*  33  */ pio_base_t  pid_o_manual_dac_output;
                                              // manual dac output, if set.
    /*  34  */ pio_base_t  pid_o_y_input;       // input only, raw input of the predictor (before bias?)
    /*  35  */ pio_base_t  pid_o_dac_a;         // copy of DAC_DA
    /*  36  */ pio_base_t  pid_o_dac_b;         // copy of DAC_DB
    /*  37  */ pio_base_t  pid_o_debug_reg_1;   // debug - mostly to see why the 2nd integral signal is identically zero when 2nd integral disabled.
    /*  38  */ pio_base_t  pid_o_pre_dither_manual_value
                                ;   // if pre-dither manual mode is enabled, this is the value fed into the output dither.
                                    // (or output in general if output dither is disabled.)
    /*  39  */ pio_base_t  pid_o_current_sum_before_rebase
                                ;   // internal summation register for debug
   
    /*  40  */ pio_base_t  pid_o_current_sum_total_low
                                ;   // internal low [31:0] register value of the low 32 bits of the 48 bits current_sum_total
    /*  41  */ pio_base_t  pid_o_current_sum_total_high
                                ;   // internal low [47:32] register value of the low high 16 bits of the 48 bits current_sum_total
    /*  42  */ pio_base_t  pid_o_dac_output
                                ;   // intended dac_output (up to inversion and or manual selection).
    /*  43  */ pio_base_t  pid_o_test_1
                                ;   // test value 1.
    /*  44  */ pio_base_t  pid_o_test_2
                                ;   // test value 2.
    /*  45  */ pio_base_t  pid_o_test_3
                                ;   // test value 3.
    /*  46  */ pio_base_t  pid_o_test_4
                                ;   // test value 4.
    /*  47  */ pio_base_t  pid_o_test_5
                                ;   // test value 5.
    /*  48  */ pio_base_t  pid_o_test_6
                                ;   // test value 6.
    /*  49  */ pio_base_t  pid_o_test_7
                                ;   // test value 7.
    /*  50  */ pio_base_t  pid_o_test_8
                                ;   // test value 8.
    /*  51  */ pio_base_t  pid_o_test_9
                                ;   // test value 9.
    /*  52  */ pio_base_t  pid_o_test_10
                                ;   // test value 10.
    /*  53  */ pio_base_t  pid_o_test_11
                                ;   // test value 11.
    /*  54  */ pio_base_t  pid_o_test_12
                                ;   // test value 12.
    /*  55  */ pio_base_t  pid_o_test_13
                                ;   // test value 13.
    /*  56  */ pio_base_t  pid_o_test_14
                                ;   // test value 14.
    /*  57  */ pio_base_t  pid_o_test_15
                                ;   // test value 15.
    /*  58  */ pio_base_t  pid_o_test_16
                                ;   // test value 16.

}   pid_pio_regs_t;

_Static_assert(sizeof(pid_pio_regs_t) == (59 * 4 * 4), "Struct size mismatch!");

const static uint32_t pid_num_regs = sizeof(pid_pio_regs_t) / sizeof(pio_base_t);

extern pid_pio_regs_t *g_predictor_regs;

#define READ_PIO_REG(_reg_name_)  \
    (g_predictor_regs->pid_ ## _reg_name_.data)

#define GET_PIO_REG_ADDR(_reg_name_) \
    (& (g_predictor_regs->pid_ ## _reg_name_.data))

#define GET_PIO_REG_ADDR_BY_INDEX(_reg_index_) \
    ( &((& ((pio_base_t *)g_predictor_regs)[_reg_index_])->data) )

#define GET_PIO_REG_BY_INDEX(_reg_index_) \
    ( ((& ((pio_base_t *)g_predictor_regs)[_reg_index_])->data) )



#define WRITE_PIO_REG(_reg_name_, _value_)  \
    (g_predictor_regs->pid_ ## _reg_name_.data = _value_)


#endif /* REGISTERS_H_ */
