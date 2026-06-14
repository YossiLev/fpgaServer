#define _GNU_SOURCE
#define soc_cv_av

#define DEBUG_SCAN_OUTPUT 0

#define __PACKED  __attribute__((packed))
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "socal/alt_clkmgr.h"
#include "hps_0.h"
#include <pthread.h>
#include <error.h>
#include <stdlib.h>
#include <string.h>
#include "gsl/gsl_linalg.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include "registers.h"
#include "predictor_api.h"
#include <assert.h>

#define SERVER_PORT 8877
#define AVERAGING_CYCLE_NSEC        10
#define MINIMAL_AVERAGING_CYCLES    2   // Predictor at least works for 2 cycles of 10nsec, or 20 nsec
#define MAX_SAMPLES                 32768
#define MAX_SAMPLES_INTERVAL        32768
#define SLICE_SAMPLES               512
#define MAX_NUM_OF_BUFFESRS         8

extern void hex_dump(const char *title, const char *buffer, const size_t size);

const static char  version_string[] =
{
                __FILE__
                ":"
                __DATE__
                ":"
                __TIME__
                "  pid controller net version 0.1"
};


#define SYNC_WORD   0x12345678

#define PACKET_TYPE_NULL        0
#define PACKET_TYPE_COMMAND     1
#define PACKET_TYPE_RESPONSE    2
#define PACKET_TYPE_ERROR       -1

#define COMMAND_ERROR                    -1     // Indicates packet header is fault
#define COMMAND_GET_VERSION              0
#define COMMAND_PING                     1
#define COMMAND_SET_PREDICTOR_ORDER      2
#define COMMAND_SET_PREDICTOR_ALPHA      3
#define COMMAND_SET_AVERAGING_TIME       4
#define COMMAND_SET_GAIN                 5

#define COMMAND_GET_REGISTER_DUMP               8
#define COMMAND_SET_OUTPUT_SHIFT                9
#define COMMAND_SET_INPUT_SELECT                10
#define COMMAND_SET_INPUT_OFFSET                11
#define COMMAND_SET_OUTPUT_OFFSET               12
#define COMMAND_SET_INPUT_AVERAGING_ENABLE      13
#define COMMAND_SET_DITHERING_PARAMETERS        14
#define COMMAND_STOP                            15
#define COMMAND_START                           16
#define COMMAND_TERMINATE                       17
#define COMMAND_GET_SINGLE_REGISTER_N_SAMPLES   18
#define COMMAND_GET_TWO_REGISTERS_N_SAMPLES     19
#define COMMAND_SET_REGISTER_AND_GET_TWO_REGISTER_SAMPLES                    \
                                                20
#define COMMAND_SELECT_OUTPUT_SIGNAL            21
#define COMMAND_SET_2ND_INTEGRATOR_ENABLE       22
#define COMMAND_SET_MANUAL_DAC_OUTPUT           23

#define COMMAND_SCAN_BEGIN                      24
#define COMMAND_SCAN_END                        25
#define COMMAND_GET_TWO_REGISTERS_QUICK_SAMPLES 26

#define COMMAND_REBOOT                          99

#define PACKET_MAX_LEN                          32768

#define INPUT_SELECT_A                          1
#define INPUT_SELECT_B                          0
#define INPUT_SELECT_A_MINUS_B                  2

#define STATUS_INVALID_AVERAGING_TIME           101
#define STATUS_INVALID_REGISTER_INDEX           102
#define STATUS_INVALID_NUM_SAMPLES              103
#define STATUS_ERROR_NO_MEMORY                  104
#define STATUS_INVALID_SIGNAL_INDEX             105
#define STATUS_INVALID_PACKET_LENGTH            106
#define STATUS_INVALID_INPUT_SELECT             107
#define STATUS_INVALID_INTERVAL_SAMPLES         108
#define STATUS_WARNING_ALREADY_STOPPED            1
#define STATUS_WARNING_ALREADY_STARTED            2


enum scan_type_e
{
    SCAN_TYPE_SAW_TOOTH = 1,
    SCAN_TYPE_TRIANGLE  = 2,
    SCAN_TYPE_NONE      = 0 // pick the default which is saw tooth
}; 

static const char *scan_type_to_str(const enum scan_type_e scan_type)
{
    switch(scan_type)
    {
        case SCAN_TYPE_SAW_TOOTH:
            return "saw-tooth";
        
        case SCAN_TYPE_TRIANGLE:
            return "triangle";

        case SCAN_TYPE_NONE:
            return "none?";

        default:
            return "<UNKNOWN!>";
    }

}

// 
// scan payload used by scan state
//
typedef struct __attribute__((packed)) scan_begin_payload_s
{
    uint32_t    command                     ;  // should be COMMAND_SCAN_BEGIN
    uint32_t    offset                      ;
    uint32_t    frequency_hz                ;
    uint32_t    amplitude                   ;
    uint32_t    scan_type                   ;
}   scan_begin_payload_t;

typedef struct __attribute__((packed))
{
}   scan_end_payload_t;

typedef struct
{
    double   alpha   ;   // alpha = predictor order. alpha = 1 means predicting one time step into the future.
    int32_t  n       ;   // n = predictor order (n = 4 is the default)
    double   p_gain  ;   // proportional gain
    int32_t  i_gain  ;   // integral gain
    int32_t  i2_gain ;   // 2nd integral gain
    int32_t  delay   ;   // delay = # of extra predictor 10ns cycle added prolonging its normally 20ns period.
                        // The delay can be used for slower prediction/work cycle but using averaging to increase noise resilience.
    int32_t  input_offset
                     ;
    int32_t  input_select
                     ;   // "0" = ADC A, "1" = ADC B
    uint32_t output_shift
                     ;
    uint32_t i0_shift
                    ;
    uint32_t i0_2nd_shift
                    ;
    int32_t output_offset
                    ;
    int32_t output_offset_2nd
                    ;
    int32_t averaging_enable
                    ;
    int32_t started ;

    int32_t manual_dac_output_enable;  // if true, o_z_n   = manual_dac_output   = i2_manual_dac_output
    int32_t pre_dither_manual_enable;  // if true, pre_dither_manual_enable is set, which means that the output dither
                                       // is fed with the value stored in the pre_dither_manual_value register.
    uint32_t
            manual_dac_output_value;   // value to write to manual_dac_output in case manual is enabled

    uint32_t
            output_signal_index;    // Which bit to set in DACB register

    int32_t
            i2_enabled;             // 2nd integrator is enabled?
    uint32_t
            i2_disabled_value;      // If 2nd integrator is __disabled__, what's the value
    int32_t input_diff_enable;      // 'true' --> y_input of the predictor is the numerical difference of ADC A and ADC B.
                                    // 'false'--> y_input is the selected input.

    struct dither_parameters
    {
        int32_t output_enabled          ;
        int32_t output_phase_1_count    ;
        int32_t output_amplitude        ;

        int32_t input_enabled           ;
        int32_t input_phase_1_count     ;
        int32_t input_phase_2_count     ;
        int32_t input_init_polarity     ;
    }   dither_parameters;

    // forward declaration (can be do this inside a C struct?)
    struct scan_state_s
    {
        pthread_t   scan_thread         ;
        sem_t       scan_thread_work    ;
        uint32_t    scan_delay_per_sec  ;  // this is calibrate below
        _Bool       scan_thread_started ;
        volatile _Bool
                    scan_in_progress    ;
        struct scan_begin_payload_s
                    scan_parameters_cmd ;
        struct scan_begin_payload_s
                    scan_parms_running  ;
        uint32_t    scan_run_loop_count ;

            
        // version counters for scan args
        // such that the thread can update itself
        // if a change occurred.
        volatile uint32_t    scan_change_count   ;   
        volatile uint32_t    scan_run_count      ;
        volatile uint32_t    last_tick_count     ;
        //
        // boolean set by server thread,
        // on scan-begin/end.
        // 
        volatile _Bool       scan_begin          ;
    }   scan_state;

}   predictor_state_t;


typedef struct __attribute__((packed))
{
    uint32_t    command     ;
    uint32_t    arg0        ;
    uint32_t    arg1        ;
    uint32_t    arg2        ;
}   command_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t    command     ;
    double      alpha       ;
}   set_alpha_payload_t     ;

typedef struct __attribute__((packed))
{
    uint32_t    command     ;
    int32_t     n           ;
}   set_order_payload_t     ;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    int32_t     averaging_time_cycles   ;
}   set_averaging_time_payload_t        ;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    int32_t     i_gain                  ;
    int32_t     i2_gain                 ;
    double      p_gain                  ;   // gain to be used in setting alpha
}   set_gain_payload_t                  ;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    output_shift            ;
    uint32_t    i0_shift                ;
    uint32_t    i0_2nd_shift            ;
}   set_output_shift_payload_t                  ;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    output_offset           ;   // Output value to be added to the predictor result
    uint32_t    output_offset_2nd       ;   // Output value to be added to the 2nd integrator output
}   set_output_offset_payload_t                  ;


typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    input_select            ;   // 0 = A, 1 = B, 2 = difference between A and B
}   set_input_select_payload_t           ;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    int32_t     input_offset            ;   // Signed offset, only 14 bits count
}   set_input_offset_payload_t          ;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    int32_t     input_averaging_enable  ;   // 0 = disabled, 1 = enabled
}   set_input_averaging_enable_payload_t;


typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    int32_t     output_enable           ;   // dithering on output enable: 0 = disabled
    int32_t     output_amplitude        ;   // Amount, in units of DAC, to modulate (dither) the output
    int32_t     output_phase_1_count    ;   // # of DAC cycles in which the high polarity (which is always how the output dithering starts)
                                            // before switching to the lower polarity. Basically, should be 1/2 predictor cycle
                                            // (4 cycles + delay cycles) in units of DAC clock.
    int32_t     input_enable            ;   // dithering on output enable: 0 = disabled
    int32_t     input_phase_1_count     ;   // dithering on the input: # of DAC (= ADC) cycles with the initial polarity.
    int32_t     input_phase_2_count     ;   // dithering on the input: # of DAC (= ADC) cycles with the 2nd phase polarity
    int32_t     input_init_polarity     ;   // dithering input polarity at the 1st cycle = polarity of phase 1.
}   set_dithering_parameters_payload_t;


typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    register_index          ;   // index of register to read, 0...15 from predictor space, 16..31 from predictor2 space
    uint32_t    num_samples             ;   // # of samples to return
    double      interval_msec           ;   // interval (in msec) between samples (0 = no delay)
}   get_single_register_n_samples_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t    time_stamp              ;
    uint32_t    sample                  ;
}   get_single_register_n_samples_response_entry_t;


typedef struct __attribute__((packed))
{
    uint32_t    num_samples             ;
    get_single_register_n_samples_response_entry_t
                samples[0]              ;
}   get_single_register_n_samples_response_t;


typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    register_index_1        ;   // index of register to read, 0...15 from predictor space, 16..31 from predictor2 space
    uint32_t    register_index_2        ;   // ditto for the 2nd register
    uint32_t    num_samples             ;   // # of samples to return
    double      interval_msec           ;   // interval (in msec) between samples (0 = no delay)
}   get_two_registers_n_samples_payload_t;


typedef struct __attribute__((packed))
{
    uint32_t    time_stamp              ;
    uint32_t    sample_1                ;   // Sample of register 1
    uint32_t    sample_2                ;   // Sample of register 2
}   get_two_registers_n_samples_response_entry_t;


typedef struct __attribute__((packed))
{
    uint32_t    num_samples             ;
    uint32_t    register_index_1        ;
    uint32_t    register_index_2        ;
    uint32_t    register_address_1      ;
    uint32_t    register_address_2      ;
    get_two_registers_n_samples_response_entry_t
                samples[0]              ;
}   get_two_registers_n_samples_response_t;


typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    register_index_1        ;   // index of register to read, 0...15 from predictor space, 16..31 from predictor2 space
    uint32_t    register_index_2        ;   // ditto for the 2nd register
    uint32_t    interval_samples        ;   // interval in samples
}   get_two_registers_quick_samples_payload_t;


typedef struct __attribute__((packed))
{
    uint32_t    sample_1                ;   // Sample of register 1
    uint32_t    sample_2                ;   // Sample of register 2
}   get_two_registers_quick_samples_response_entry_t;


typedef struct __attribute__((packed))
{
    uint32_t    num_samples             ;
    uint32_t    register_index_1        ;
    uint32_t    register_index_2        ;
    get_two_registers_quick_samples_response_entry_t
                samples[0]              ;
}   get_two_registers_quick_samples_response_t;

typedef struct __attribute__((packed))
{
    uint32_t    command                 ;
    uint32_t    signal_index            ;   // signal index taken from DACB output
}   select_output_signal_payload_t;


typedef struct __attribute__((packed))
{
    //
    // This command writes to a register a set of samples given.
    // After each set, it reads the two registers.
    // Once the set is complete the entire get registers are sent back.
    // This command is intended to provide the host with a tool to do active
    // setting of the pid and watching the inputs to see how the system responds.
    //
    uint32_t    command                 ;
    uint32_t    set_register_index      ;   // register index to set its values
    uint32_t    get_register_index_1    ;   // 1st register index to read
    uint32_t    get_register_index_2    ;   // 2nd register index to read
    uint32_t    num_samples             ;   // # of samples to set and return
    double      interval_msec           ;   // interval (in msec) between samples (0 = no delay)
    uint32_t    samples[0]              ;   // Should have num_samples # of entries
}   set_register_and_get_two_registers_samples_payload_t;


typedef struct __attribute__((packed))
{
    //
    // 2nd integrator is normally disabled.
    //
    uint32_t    command                 ;
    int32_t     is_enabled              ;
    uint32_t    disabled_value          ;   // Relevant only if disabled.
}   set_2nd_integrator_enable_payload_t;


typedef struct __attribute__((packed))
{
    //
    // Override manual value of the primary dac output.
    // (actual, this is the predictor output override, the final value is set by dacb
    //  register set...)
    //
    uint32_t    command                     ;
    int32_t     is_manual_dac_output_enabled;   // if true, value is the following..
    uint32_t    manual_dac_output_value     ;   // Relevant only if manual is enabled.
}   set_manual_dac_output_payload_t;



//
// Instead of sweep command, decided to allow the host to write samples directly
// via the 'set_register_and_get_two_registers_samples...' above...
//
//typedef struct __attribute__((packed))
//{
//    uint32_t    command                 ;
//    uint32_t    sweep_mode              ;   // 1 = saw tooth,
//    uint32_t    amplitude               ;   // amplitude of sweep
//    uint32_t    offset                  ;   // offset from 0 amplitude
//    uint32_t    frequency               ;   // frequency of sweep
//}   begin_sweep_payload_t;


typedef struct __attribute__((packed))
{
    uint32_t    command     ;   // the same as the command that triggered the response
    uint32_t    status      ;
    uint8_t     response[0] ;
}   command_response_t  ;

typedef struct __attribute__((packed))
{
    uint32_t    sync        ;
    uint32_t    type        ;
    uint32_t    len         ;
    uint8_t     payload[0]  ;
}  communication_packet_t;

typedef struct
{
    int                 socket          ;
    predictor_state_t   predictor_state ;
}   server_t    ;

server_t    g_server;

#if DEBUG_SCAN_OUTPUT
#define SCAN_TRACE_SIZE 65536
struct scan_trace_entry
{
    uint32_t  current;
};

struct scan_trace
{
    uint32_t index;
    uint32_t count;

    struct scan_trace_entry vec[SCAN_TRACE_SIZE];
};


struct scan_trace g_scan_trace = {0};
void scan_trace_enter(struct scan_state_s *state, const uint32_t current)
{
    struct scan_trace *pg = & g_scan_trace;

    uint32_t index = pg->index;
    struct scan_trace_entry *p = &pg->vec[index];

    p->current = current;

    index = (index + 1) & (SCAN_TRACE_SIZE - 1);
    pg->index = index;
    pg->count ++ ;

    if (pg->count < 10)
    {
        printf("[[[trace enter]]]: index=%d, count=%d\n", pg->index, pg->count);
    }
}


void scan_trace_dump(void)
{
    const static char *dump_file_name = "scan_debug.dump";
    const struct scan_trace *pg = & g_scan_trace;
    const uint32_t index = pg->index;
    FILE *fp = fopen(dump_file_name, "w");
    struct scan_state_s *state = &g_server.predictor_state.scan_state;

    if (!fp)
    {
        printf("***ERROR: failed to fopen() trace file, errno=%d\n", errno);
        return;
    }

    const time_t now = time(NULL);    

    fprintf(fp, "scan debug started: [%s]\n", asctime(localtime(&now)));
    fprintf(fp, "scan state: change count %d, run count %d\n", state->scan_change_count, state->scan_run_count);
    fprintf(fp, "thread started? %d, in progress? %d, scan begin? %d\n", state->scan_thread_started, state->scan_in_progress, state->scan_begin);
    fprintf(fp, "current: offset=%d, freq=%d, amplitude=%d, scan_type=%s\n", 
            state->scan_parms_running.offset, 
            state->scan_parms_running.frequency_hz, 
            state->scan_parms_running.amplitude,
            scan_type_to_str(state->scan_parms_running.scan_type));

    fprintf(fp, "scan run count:%d, last tick count:%d\n", state->scan_run_loop_count, state->last_tick_count);

    fprintf(fp, "count %d, index %d\n\n", pg->count, pg->index);

    for (int i = 0 ; i < SCAN_TRACE_SIZE ; i++)
    {
        fprintf(fp, "[%s%d] = %d\n", (i == index ? "*" : " "), i, pg->vec[i].current);
    }

    fclose(fp);

    printf("finished dumping scan trace to %s\n", dump_file_name);
}

#else

void scan_trace_dump(void)
{
    printf("***WARN****: scan trace is not compiled, nothing dumped\n");
}
#endif // DEBUG_SCAN_OUTPUT //


static
size_t get_data(const int s, uint8_t* buffer, const size_t payload_size)
{
    size_t remaining_size  = payload_size;
    size_t pos             = 0;
    int    should_continue = 1;

    while(should_continue)
    {
        int rc = recv(s, & buffer[pos], remaining_size, 0);
        if (rc < 0)
        {
            printf("recv [%d] failed! errno %d\n", s, errno);
            should_continue = 0;
            pos = -1;
        } // recv() failed... //
        else if (rc > 0)
        {
            remaining_size -= rc;
            pos            += rc;
            if (remaining_size <= 0)
            {
                should_continue = 0;
            }
        } // if got actual bytes.... //
        else
        {
            printf("client[%d]: recv() returned 0, has peer shut the connection?\n", s);
            should_continue = 0;
        } // else--> 0 was returned.. //
    } // while more data to receive... //

    return pos;
} // function get_data(). //


static
int send_data(const int s, uint8_t* buffer, const size_t payload_size)
{
    size_t remaining_size  = payload_size;
    size_t pos             = 0;
    int    should_continue = 1;

    while(should_continue)
    {
        int rc = send(s, & buffer[pos], remaining_size, 0);
        if (rc < 0)
        {
            printf("send [%d] failed! errno %d\n", s, errno);
            should_continue = 0;
            pos = -1;
        } // send() failed... //
        else if (rc > 0)
        {
            remaining_size -= rc;
            pos            += rc;
            if (remaining_size <= 0)
            {
                should_continue = 0;
            }
        } // if got actual bytes.... //
        else
        {
            printf("client[%d]: send() returned 0, has peer shut the connection?\n", s);
            should_continue = 0;
        } // else--> 0 was returned.. //
    } // while more data to receive... //

    return pos;
} // function get_data(). //


static
void send_response(const int s, const communication_packet_t *packet_header, uint8_t *response_payload, const size_t response_payload_size) {
    communication_packet_t response_header;
    response_header.sync = packet_header->sync;
    response_header.len  = response_payload_size;
    response_header.type = PACKET_TYPE_RESPONSE;

    const int result_buffer_size = response_payload_size + 12;
    uint8_t * result_buffer_bytes = (uint8_t *)malloc(result_buffer_size);
    memcpy(result_buffer_bytes, &response_header, 12);
    memcpy(result_buffer_bytes + 12, response_payload, response_payload_size);

    send_data(s, result_buffer_bytes, result_buffer_size);

    free(result_buffer_bytes);
}


static
void send_response2(const int s, const communication_packet_t *packet_header, 
        uint8_t *response_payload1, const size_t response_payload_size1, 
        uint8_t *response_payload2, const size_t response_payload_size2) {
    communication_packet_t response_header;
    response_header.sync = packet_header->sync;
    response_header.len  = response_payload_size1 + response_payload_size2;
    response_header.type = PACKET_TYPE_RESPONSE;

    const int result_buffer_size = response_payload_size1 + response_payload_size2 + 12;
    char* result_buffer_bytes = malloc(result_buffer_size);
    memcpy(result_buffer_bytes, &response_header, 12);
    memcpy(result_buffer_bytes + 12, response_payload1, response_payload_size1);
    memcpy(result_buffer_bytes + 12 + response_payload_size1, response_payload2, response_payload_size2);

    send_data(s, result_buffer_bytes, result_buffer_size);

    free(result_buffer_bytes);
}


static
communication_packet_t   get_packet_header(const int s)
{
    communication_packet_t header = {0};

    printf("get header size for %d\n", s);
    const size_t packet_header_size = get_data(s, (uint8_t*)&header, sizeof(header));
    printf("header size %d\n", packet_header_size);

    if (packet_header_size == -1)
    {
        printf("***ERROR: get for header failed...\n");
        header.type = PACKET_TYPE_ERROR;
    }
    else if (packet_header_size != sizeof(header))
    {
        printf("***ERROR: peer shut the connection?\n");
        header.type = PACKET_TYPE_ERROR;
    }
    else if ((header.sync & 0x0000FFFF) != (SYNC_WORD & 0x0000FFFF))
    {
        printf("***ERROR: expected sync[0x%8.8x] and got[0x%8.8x]\n", SYNC_WORD, header.sync);
        header.type = PACKET_TYPE_ERROR;
    }
    else if (header.len > PACKET_MAX_LEN)
    {
        printf("***ERROR: packet len[%d] > max len[%d]\n", header.len, PACKET_MAX_LEN);
        header.type = PACKET_TYPE_ERROR;
    }
    else
    {
        printf("[%d]: Got good new packet, type[%d], len[%d]\n", s, header.type, header.len);
    }

    return header;
} // function get_packet_header(). //

static void scan_print_parameters(const scan_begin_payload_t *scan_parms)
{
    uint32_t    offset                      ;
    uint32_t    frequency_hz                ;
    uint32_t    amplitude                   ;
    uint32_t    scan_type                   ;

    printf("Scan parameters at %p: \n"
           "scan type (@0x%x).......%s\n"
           "scan freq (Hz) (@0x%x)..%d\n"
           "amplitude......(@0x%x)..%d\n"
           "offset.........(@0x%x)..%d\n",
           scan_parms,
           offsetof(scan_begin_payload_t, scan_type     ), scan_type_to_str((enum scan_type_e)scan_parms->scan_type),
           offsetof(scan_begin_payload_t, frequency_hz  ), scan_parms->frequency_hz,
           offsetof(scan_begin_payload_t, amplitude     ), scan_parms->amplitude,
           offsetof(scan_begin_payload_t, offset        ), scan_parms->offset);
} // function scan_print_parameters(). //



static void scan_output_value(struct scan_state_s *state, const int value)
{
    WRITE_PIO_REG(o_pre_dither_manual_value, value);
#if DEBUG_SCAN_OUTPUT
    scan_trace_enter(state, value);
#endif
}


volatile uint32_t g_calibration = {0};

//
// calibration finds what is the desired loop count to delay for a particular amount of time.
//
static void scan_calibrate_delay(struct scan_state_s *state)
{
    struct timespec resolution;
    
    int rc = clock_getres(CLOCK_MONOTONIC_RAW, &resolution);

    if (rc < 0)
    {
        printf("**ERROR: failed to get clock resolution, errno=%d\n", errno);
        state->scan_delay_per_sec = 1;
        return;
    }

    double precision = 1e-9 * (double)resolution.tv_nsec + resolution.tv_sec;

    if (precision > 1e-6)
    {
        printf("***ERROR!!!! precision way too coarse at [%g] seconds", precision);
    }

    struct timespec t1;
    struct timespec t2;

    clock_gettime(CLOCK_MONOTONIC_RAW, &t1);

#define LOOP_SIZE   (1000*1000*1000)    
    //
    // count to LOOP_SIZE to do a long busy wait.
    //
    for (int i ; i < LOOP_SIZE ; i++) g_calibration++ ;

    clock_gettime(CLOCK_MONOTONIC_RAW, &t2);

    double t1d = 1e-9*(double)t1.tv_nsec + t1.tv_sec;
    double t2d = 1e-9*(double)t2.tv_nsec + t2.tv_sec;

    printf(">>> calibration: %d iterations took %g seconds, precision=%g\n", LOOP_SIZE, t2d - t1d, precision);

    //
    // tick = amount of ticks per second.
    //
    double tick = (double)LOOP_SIZE / (t2d - t1d);

    state->scan_delay_per_sec = (uint32_t)tick;

    printf(">>> tick=%g, normalized to uint32_t=%d\n", tick, state->scan_delay_per_sec);
}


static void scan_set_pre_dither_manual_mode(const int32_t value)
{
    //
    // write the scan output register.
    //
    g_server.predictor_state.pre_dither_manual_enable = (_Bool)value;


    union predictor_hw_reg_o_config_u config;

    config._w32 = READ_PIO_REG(o_config);
    config.c.pre_dither_manual_enable = value;
    WRITE_PIO_REG(o_config, config._w32);
}


static void scan_run(struct scan_state_s *state)
{
    state->scan_run_loop_count++;
    // lowest rate is 1Hz.
    if (state->scan_parms_running.frequency_hz == 0)
    {
        state->scan_parms_running.frequency_hz = 1;
    }

    // minimal amplitude is 1
    if (state->scan_parms_running.amplitude == 0)
    {
        state->scan_parms_running.amplitude = 1;
    }

    // ensure scan type is good.
    if (    state->scan_parms_running.scan_type != SCAN_TYPE_SAW_TOOTH 
        &&  state->scan_parms_running.scan_type != SCAN_TYPE_TRIANGLE)
    {
        state->scan_parms_running.scan_type = SCAN_TYPE_SAW_TOOTH;
    }

    enum scan_type_e scan_type = state->scan_parms_running.scan_type;
    double interval_usec = 1e6 * 1.0 / (double)state->scan_parms_running.frequency_hz;
    int start = (int)state->scan_parms_running.offset;
    int end   = start + (int)state->scan_parms_running.amplitude;
    //
    // span depends on type
    //
    double span    = (double)(end - start);
    double delay_usec = interval_usec / span;
    //
    // for triangular waveform, the frequency refers
    // to both the rising and falling slopes of the triangle.
    // for sawtooth - the frequency refers just to the rising slope,
    // as the falling slope is just an immediate drop.
    //
    if (scan_type == SCAN_TYPE_TRIANGLE)
    {
        delay_usec = delay_usec / 2.0;
    }


    double delay_sec = delay_usec / 1e6;
    double ticks_to_delay_d = delay_sec * (double)state->scan_delay_per_sec;
    uint32_t ticks_to_delay = (uint32_t)ticks_to_delay_d;

    state->last_tick_count = ticks_to_delay;

    int current = (int)start;
    int direction = 1;      

    uint32_t version = state->scan_run_count;

    printf("scan thread run: version=%u, delay=%g (usec), ticks to delay=(%g => %d), span=%g, type=%s\n", 
                version, delay_usec, ticks_to_delay_d, ticks_to_delay, span, scan_type_to_str(scan_type));

    //
    // scan command enabled - let's go.
    //
    while (state->scan_begin)
    {
        if (version != state->scan_change_count)
        {
            printf("scan thread: version changed detected from %d ==> %d\n", version, state->scan_change_count);
            // easiest way to reenter the function from above.
            state->scan_parms_running = state->scan_parameters_cmd;
            state->scan_run_count = state->scan_change_count;
            return;
        }

        scan_output_value(state, current);

        if (scan_type == SCAN_TYPE_SAW_TOOTH)
        {
            current += direction;
            if (current >= end) 
            {
                current = start;
            }
        }

        if (scan_type == SCAN_TYPE_TRIANGLE)
        {
            current += direction;
            if (current >= end)
            {
                direction = -1;
            }
            else if (current <= start)
            {
                direction = +1;
            }
        }

        //
        // busy loop till the next iteration.
        //
        for (uint32_t i = 0 ; i < ticks_to_delay ; i++) g_calibration++ ;
    }

}



static void *scan_thread_entry(void *arg)
{
    struct scan_state_s *scan_state = (struct scan_state_s *)arg;

    scan_calibrate_delay(scan_state);

    while(1)
    {
        // we're nog locking anything, we're waiting for a work tick.
        int rc = sem_wait(& scan_state->scan_thread_work);

        if (rc < 0)
        {
            printf("ERROR in scan thread: sem_wait failed on %d\n", errno);
            return NULL;
        }

        //
        // see what is requested
        //
        if (!scan_state->scan_begin)
        {
            printf("stop scan requested, returning to sleep\n");
            continue;
        }

        // 1st stage - pickup the latest scan parameters.
        uint32_t version = scan_state->scan_change_count;

        scan_state->scan_run_count = version;

        scan_state->scan_parms_running = scan_state->scan_parameters_cmd;

        while (scan_state->scan_begin)
        {
            scan_run(scan_state);
        }
    }
}





static void scan_state_create_thread(struct scan_state_s *state)
{
    if (!state->scan_thread_started)
    {
        // semaphore tha would be used to tick the scan thread for the next scan request.
        int rc = sem_init(&state->scan_thread_work, false, 0);

        if (rc < 0)
        {
            printf("FAILED to initialize scan thread semaphore, errno=%d\n", errno);
            return;
        }

        rc = pthread_create(& state->scan_thread, NULL, scan_thread_entry, state);

        if (rc < 0)
        {
            printf("FAILED to initialize pthread, errno=%d\n", errno);
            return;
        }
        state->scan_thread_started = true;
        printf("scan thread started ok\n");
    }
}


static void scan_state_apply(struct scan_state_s *state, const scan_begin_payload_t *cmd)
{
    printf("About to apply scan state to\n");
    scan_print_parameters(cmd);

    if (!state->scan_thread_started)
    {
        printf("scan thread not running - starting it\n");
        scan_state_create_thread(state);
    }

    //
    // set H/W to pre-dither manual mode.
    // scan thread would set the pre-manual value depending 
    // on the scan parameters.
    //
    scan_set_pre_dither_manual_mode(1);

    state->scan_parameters_cmd = cmd[0];
    // NOTE: barrier needed here... as setting 
    // of the command parameters may delay after the counter
    // increment, when perceived by the CPU running the 
    // scan thread.
    state->scan_change_count ++ ;
    state->scan_begin = true;
    state->scan_in_progress = true;
    // tick the thread, in case it is parking.
    sem_post(&state->scan_thread_work);

} // function scan_state_apply(). //



static void scan_state_disable(struct scan_state_s *state)
{
    printf("About to stop scan\n");

    scan_set_pre_dither_manual_mode(0);

    if (!state->scan_thread_started)
    {
        printf("scan thread not running - ignoring request\n");
        return;
    }

    //
    // TODO: disable H/W scan state
    // 

    state->scan_begin = false;
    state->scan_in_progress = false;
    // tick the thread, in case it is parking.
    sem_post(&state->scan_thread_work);
} // function scan_state_disable(). //




static void handle_register_dump_response(const int s, const communication_packet_t* packet)
{
    typedef struct
    {
        volatile uint32_t *address ;
        uint32_t    value   ;
    }   register_entry_t;

    const int num_regs = sizeof(pid_pio_regs_t) / sizeof(pio_base_t);

    typedef struct
    {
        register_entry_t    regs[num_regs];
    }   predictor_registers_t;

    typedef struct
    {
        command_response_t      header;
        predictor_registers_t   p;
    }   register_dump_payload_t;

    communication_packet_t response;

    // response.sync = packet->sync;
    // response.len  = sizeof(register_dump_payload_t);
    // response.type = PACKET_TYPE_RESPONSE;

    register_dump_payload_t response_payload;
    response_payload.header.command = COMMAND_GET_REGISTER_DUMP;
    response_payload.header.status  = 0;

    //printf("%s: num regs %d, expected payload size %d\n", __func__, num_regs, sizeof(predictor_registers_t));

    for (int i = 0 ; i < num_regs ; i++)
    {
        response_payload.p.regs[i].value = GET_PIO_REG_BY_INDEX(i);
        response_payload.p.regs[i].address = GET_PIO_REG_ADDR_BY_INDEX(i);
        printf("Reg %2d - %d\n", i, response_payload.p.regs[i].value);
    }

    send_response(s, packet, (uint8_t*)& response_payload, sizeof(response_payload));
    // send_data(s, (uint8_t*)& response           , sizeof(response        ));
    // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));

} // function handle_register_dump_response(). //


static void set_predictor(void)
{
    set_alpha(g_server.predictor_state.alpha    ,
              g_server.predictor_state.p_gain   ,
              g_server.predictor_state.n        );

    union predictor_hw_reg_o_config_u config;

    config._w32 = READ_PIO_REG(o_config);

    WRITE_PIO_REG(o_delay_count, g_server.predictor_state.delay);

    WRITE_PIO_REG(o_i0, g_server.predictor_state.i_gain);

    union config_2nd_u  config_2nd;
    config_2nd._w32                     = READ_PIO_REG(o_2nd_config);
    config_2nd.c.i0_shift               = g_server.predictor_state.i0_shift;
    config_2nd.c.output_shift_2nd       = g_server.predictor_state.i0_2nd_shift;
    WRITE_PIO_REG(o_2nd_config, config_2nd._w32);

    union config_3rd_u  config_3rd;
    config_3rd._w32                     = READ_PIO_REG(o_3rd_config);
    config_3rd.c.i0_2nd                 = g_server.predictor_state.i2_gain;
    WRITE_PIO_REG(o_3rd_config, config_3rd._w32);

    config.c.input_averaging_enable =  g_server.predictor_state.averaging_enable  ;
    config.c.input_select           =  g_server.predictor_state.input_select      ;
    config.c.continuous             =  g_server.predictor_state.started           ;
    config.c.y_input_diff_enable    =  g_server.predictor_state.input_diff_enable ;
    config.c.pre_dither_manual_enable
                                    =  g_server.predictor_state.pre_dither_manual_enable    
                                                                                  ;
    config.c.second_integrator_output_enable
                                    =  1;   // Always enable the 2nd output, but the integration itself is controllable.
    config.c.second_integrator_enable
                                    =  g_server.predictor_state.i2_enabled        ;

    config.c.manual_dac_output_enable
                                    =  g_server.predictor_state.manual_dac_output_enable;



    //
    // delay = prolonging the predictor cycle, providing, at the expense of
    //         latency and frequency, the possibility of  higher precision through
    //         input averaging (which is essentially summation).
    //
    config.c.do_delay               =  (g_server.predictor_state.delay != 0)      ;
    config.c.output_shift           =  g_server.predictor_state.output_shift      ;

    printf("------ offsets (set) %d %d\n", g_server.predictor_state.output_offset, g_server.predictor_state.output_offset_2nd);

    // const int aa = GET_PIO_REG_BY_INDEX(25);
    // const int aa2 = READ_PIO_REG(o_2nd_out_offset);
    WRITE_PIO_REG(o_delay_count     , g_server.predictor_state.delay        )  ;
    WRITE_PIO_REG(o_i0              , g_server.predictor_state.i_gain       )  ;
    WRITE_PIO_REG(o_out_offset      , g_server.predictor_state.output_offset)  ;
    WRITE_PIO_REG(o_2nd_out_offset  , g_server.predictor_state.output_offset_2nd)  ;
    // const int bb = GET_PIO_REG_BY_INDEX(25);
    // const int bb2 = READ_PIO_REG(o_2nd_out_offset);

    g_predictor_regs->pid_o_2nd_out_offset.data = g_server.predictor_state.output_offset_2nd;
    // const int cc0 = g_server.predictor_state.output_offset_2nd;
    // const int cc = g_predictor_regs->pid_o_2nd_out_offset.data;
    // const int cc2 = READ_PIO_REG(o_2nd_out_offset);
    g_predictor_regs->pid_o_out_offset.data = g_server.predictor_state.output_offset;
    // const int pcc0 = g_server.predictor_state.output_offset;
    // const int pcc = g_predictor_regs->pid_o_out_offset.data;
    // const int pcc2 = READ_PIO_REG(o_out_offset);

    // const char *addr1 = (char *)&(g_predictor_regs->pid_o_2nd_out_offset.data);
    // const char *addr2 = (char *)&(g_predictor_regs->pid_o_2nd_out_offset);
    // const char *addr3 = (char *)(g_predictor_regs);
    // const int addrD = (addr2 - addr3);

    // const char *baddr1 = (char *)&(g_predictor_regs->pid_version.data);
    // const char *baddr2 = (char *)&(g_predictor_regs->pid_version);
    // const char *baddr3 = (char *)(g_predictor_regs);
    // const int baddrD = (baddr2 - baddr3);

    // printf("------ aabb %d %d %d %d, %d %d (%d), %d %d (%d)\n", aa, bb, aa2, bb2, cc, cc2, cc0, pcc, pcc2, pcc0);
    // printf("------ addr %08x %08x %08x %d \n", (unsigned int)addr1, (unsigned int)addr2, (unsigned int)addr3, addrD);
    // printf("------ addr %08x %08x %08x %d \n", (unsigned int)baddr1, (unsigned int)baddr2, (unsigned int)baddr3, baddrD);
    
    WRITE_PIO_REG(o_y_reference     , g_server.predictor_state.input_offset )  ;
    WRITE_PIO_REG(o_manual_dac_output,g_server.predictor_state.manual_dac_output_value);

    const int num_regs = sizeof(pid_pio_regs_t) / sizeof(pio_base_t);

    WRITE_PIO_REG(o_config, config._w32);

    union dither_config_1  dither_config_1;
    union dither_config_2  dither_config_2;
    union dither_config_3  dither_config_3;

    dither_config_1.w.input_init_count    = g_server.predictor_state.dither_parameters.input_phase_1_count ;
    dither_config_1.w.output_amplitude    = g_server.predictor_state.dither_parameters.output_amplitude    ;
    dither_config_2.w.input_next_count    = g_server.predictor_state.dither_parameters.input_phase_2_count ;
    dither_config_2.w.output_init_count   = g_server.predictor_state.dither_parameters.output_phase_1_count;
    dither_config_3.w.input_init_polarity = g_server.predictor_state.dither_parameters.input_init_polarity ;
    dither_config_3.w.dither_input_enable = g_server.predictor_state.dither_parameters.input_enabled       ;
    dither_config_3.w.dither_output_enable= g_server.predictor_state.dither_parameters.output_enabled      ;

    WRITE_PIO_REG(o_dither_config_1, dither_config_1._w32);
    WRITE_PIO_REG(o_dither_config_2, dither_config_2._w32);
    WRITE_PIO_REG(o_dither_config_3, dither_config_3._w32);

    WRITE_PIO_REG(o_2nd_output, g_server.predictor_state.i2_disabled_value);

    WRITE_PIO_REG(o_dacb_output, (1 << g_server.predictor_state.output_signal_index));


} // function set_predictor(). //


static void send_generic_response(const int s, const communication_packet_t* packet, const uint32_t command, const uint32_t status)
{
    printf("[%d]: sending response status 0x%x to command %d\n",s, status, command);

    // communication_packet_t response;
    // response.sync = packet->sync;
    // response.len  = sizeof(command_response_t);
    // response.type = PACKET_TYPE_RESPONSE;

    command_response_t response_payload;
    response_payload.command = command;
    response_payload.status  = status;

    send_response(s, packet, (uint8_t*)& response_payload, sizeof(response_payload));
    // send_data(s, (uint8_t*)& response           , sizeof(response        ));
    // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));
} // function send_generic_response(). //


static void handle_set_predictor_alpha(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_alpha_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_predictor alpha command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_alpha_payload_t));
        return;
    } // if length is too small then... //

    const set_alpha_payload_t* cmd = (const set_alpha_payload_t*)packet->payload;

    printf("[%d]: setting alpha to --> %g (was %g)\n", s, cmd->alpha, g_server.predictor_state.alpha);

    g_server.predictor_state.alpha = cmd->alpha;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_PREDICTOR_ALPHA, 0);
} // function handle_set_predictor_alpha(). //




static void handle_set_predictor_order(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_order_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_predictor order command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_order_payload_t));
        return;
    } // if length is too small then... //

    const set_order_payload_t* cmd = (const set_order_payload_t*)packet->payload;

    printf("[%d]: setting order to --> %d (was %d)\n", s, cmd->n, g_server.predictor_state.n);

    g_server.predictor_state.n = cmd->n;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_PREDICTOR_ORDER, 0);
} // function handle_set_predictor_order(). //




static void handle_set_averaging_time(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_averaging_time_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_averaging time command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_averaging_time_payload_t));
        return;
    } // if length is too small then... //

    const set_averaging_time_payload_t* cmd = (const set_averaging_time_payload_t*)packet->payload;

    const int32_t delay = cmd->averaging_time_cycles - MINIMAL_AVERAGING_CYCLES;

    if (delay < 0)
    {
        printf("[%d]: cannot set averaging time to %d cycles, as the minimal is %d\n",
               s,
               cmd->averaging_time_cycles,
               MINIMAL_AVERAGING_CYCLES);
        send_generic_response(s, packet, COMMAND_SET_AVERAGING_TIME, STATUS_INVALID_AVERAGING_TIME);
        return;
    }

    printf("[%d]: setting averaging time to %d cycles (%d nsec) (was %d cycles, %d nsec)\n",
           s,
           delay,
           delay * AVERAGING_CYCLE_NSEC,
           g_server.predictor_state.delay,
           g_server.predictor_state.delay * AVERAGING_CYCLE_NSEC
           );

    g_server.predictor_state.delay = delay;
    if (cmd->averaging_time_cycles == 0)
    {
        g_server.predictor_state.averaging_enable = 0;
        printf("[%d] disabling averaging...\n", s);
    } // if zero delay then... //
    else if (!g_server.predictor_state.averaging_enable)
    {
        g_server.predictor_state.averaging_enable = 1;
        printf("[%d] enabling averaging...\n", s);
    }

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_AVERAGING_TIME, 0);
} // function handle_set_averaging_time(). //



static void handle_set_gain(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_gain_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_gain command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_gain_payload_t));
        return;
    } // if length is too small then... //

    const set_gain_payload_t* cmd = (const set_gain_payload_t*)packet->payload;

    printf("[%d]: setting integrator gains to %d (i2 %d) was %d (i2 %d), proportional gain %f (was %f)\n",
           s,
           cmd->i_gain,
           cmd->i2_gain,
           g_server.predictor_state.i_gain,
           g_server.predictor_state.i2_gain,
           cmd->p_gain,
           g_server.predictor_state.p_gain);


    g_server.predictor_state.i_gain  = cmd->i_gain ;
    g_server.predictor_state.i2_gain = cmd->i2_gain;
    g_server.predictor_state.p_gain  = cmd->p_gain ;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_GAIN, 0);
} // function handle_set_gain(). //



static void handle_set_output_shift(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_output_shift_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_output_shift command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_output_shift_payload_t));
        return;
    } // if length is too small then... //

    const set_output_shift_payload_t* cmd = (const set_output_shift_payload_t*)packet->payload;

    printf("[%d]: setting output_shifts:  normal %d (was %d) i0 %d (was %d), i0 2nd %d (was %d)\n",
           s,

           cmd->output_shift                        ,
           g_server.predictor_state.output_shift    ,
           cmd->i0_shift                            ,
           g_server.predictor_state.i0_shift        ,
           cmd->i0_2nd_shift                        ,
           g_server.predictor_state.i0_2nd_shift    );


    g_server.predictor_state.output_shift = cmd->output_shift   ;
    g_server.predictor_state.i0_shift     = cmd->i0_shift       ;
    g_server.predictor_state.i0_2nd_shift = cmd->i0_2nd_shift   ;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_OUTPUT_SHIFT, 0);
} // function handle_set_output_shift(). //



static void handle_set_output_offset(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_output_offset_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_output_offset command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_output_offset_payload_t));
        return;
    } // if length is too small then... //

    const set_output_offset_payload_t* cmd = (const set_output_offset_payload_t*)packet->payload;

    printf("[%d]: setting output_offset: (N) standard %d (was %d) 2nd %d (was %d)\n",
           s,
           cmd->output_offset                           ,
           g_server.predictor_state.output_offset       ,
           cmd->output_offset_2nd                       ,
           g_server.predictor_state.output_offset_2nd   );


    g_server.predictor_state.output_offset     = cmd->output_offset     ;
    g_server.predictor_state.output_offset_2nd = cmd->output_offset_2nd ;

    set_predictor();

    const int num_regs = sizeof(pid_pio_regs_t) / sizeof(pio_base_t);

    send_generic_response(s, packet, COMMAND_SET_OUTPUT_OFFSET, 0);
} // function handle_set_output_offset(). //


static void handle_set_input_select(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_input_select_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_input_select command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_input_select_payload_t));
        return;
    } // if length is too small then... //

    const set_input_select_payload_t* cmd = (const set_input_select_payload_t*)packet->payload;

    uint32_t input_select = cmd->input_select;
    if (    (input_select != INPUT_SELECT_A         )
        &&  (input_select != INPUT_SELECT_B         )
        &&  (input_select != INPUT_SELECT_A_MINUS_B )
       )
    {
        printf("[%d]: ***ERROR: input select[%d] is invalid\n"  ,
               s                                                ,
               cmd->input_select                                );
        send_generic_response(s, packet, COMMAND_SET_INPUT_SELECT, STATUS_INVALID_INPUT_SELECT);
        return;
    } // if bad input select then... //


    printf("[%d]: setting input_select output_offset:  standard %d (was %d)\n",
           s,
           cmd->input_select                            ,
           g_server.predictor_state.input_select        );

    //
    // If using difference, mark it separately, otherwise, specify which input.
    //
    if (input_select == INPUT_SELECT_A_MINUS_B)
    {
        g_server.predictor_state.input_select      = 0;
        g_server.predictor_state.input_diff_enable = 1;
    } // if use the difference... //
    else
    {
        g_server.predictor_state.input_select      = input_select;
        g_server.predictor_state.input_diff_enable = 0;
    } // else---> use the selected input... //


    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_INPUT_SELECT, 0);
} // function handle_set_input_select(). //




static void handle_set_input_offset(const int s,const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_input_offset_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_input_offset command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_input_offset_payload_t));
        return;
    } // if length is too small then... //

    const set_input_offset_payload_t* cmd = (const set_input_offset_payload_t*)packet->payload;

    printf("[%d]: setting input_offset:  standard %d (was %d) \n",
           s                                            ,
           cmd->input_offset                            ,
           g_server.predictor_state.input_offset        );


    g_server.predictor_state.input_offset      = cmd->input_offset      ;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_INPUT_OFFSET, 0);
} // function handle_set_input_offset(). //




static void handle_set_input_averaging_enable(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_input_averaging_enable_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_input_averaging command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_input_averaging_enable_payload_t));
        return;
    } // if length is too small then... //

    const set_input_averaging_enable_payload_t* cmd = (const set_input_averaging_enable_payload_t*)packet->payload;

    printf("[%d]: setting input_averaging:  standard %d (was %d) \n",
           s                                            ,
           cmd->input_averaging_enable                  ,
           g_server.predictor_state.averaging_enable    );


    g_server.predictor_state.averaging_enable = cmd->input_averaging_enable;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_INPUT_AVERAGING_ENABLE, 0);
} // function handle_set_input_averaging_enable(). //



static void handle_get_single_register_n_samples(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(get_single_register_n_samples_payload_t))
    {
        printf("[%d]***ERROR: unexpected get_single_register_n_samples command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(get_single_register_n_samples_payload_t));
        return;
    } // if length is too small then... //

    const get_single_register_n_samples_payload_t* cmd = (const get_single_register_n_samples_payload_t*) packet->payload;

    if (cmd->register_index > pid_num_regs)
    {
        printf("[%d]: ***ERROR: register index [%d] out of bounds\n", s, cmd->register_index);
        send_generic_response(s, packet, COMMAND_GET_SINGLE_REGISTER_N_SAMPLES, STATUS_INVALID_REGISTER_INDEX);
        return;
    } // if register index is bad then... //

    if (cmd->num_samples > MAX_SAMPLES)
    {
        printf("[%d]: ***ERROR: num samples [%d] exceeds maximum allowed %d\n", s, cmd->num_samples, MAX_SAMPLES);
        send_generic_response(s, packet, COMMAND_GET_SINGLE_REGISTER_N_SAMPLES, STATUS_INVALID_NUM_SAMPLES);
        return;
    } // if register index is bad then... //

    const volatile uint32_t* register_address = NULL;


    struct timespec sleep_ts;
    int             should_sleep = 0;

    if (cmd->interval_msec != 0.)
    {
        sleep_ts.tv_sec  = (__time_t         )( cmd->interval_msec / 1000.                                 );
        sleep_ts.tv_nsec = (__syscall_slong_t)((cmd->interval_msec - (sleep_ts.tv_sec  *1000.) ) * 1000000.);
        should_sleep = 1;
    } // if sleep interval given then... //

    int samples_left = cmd->num_samples;

    while (samples_left > 0) {

        int samepls_to_send = samples_left <= SLICE_SAMPLES ? samples_left : SLICE_SAMPLES;
        samples_left -= samepls_to_send;

        //
        // Translate register index to actual address.
        // Each register is implemnented using PIO module, which has 4 actual reg per register, all other 3 are not
        // used (in/out selection, irq enable, etc).
        //
        register_address = GET_PIO_REG_ADDR_BY_INDEX(cmd->register_index);

        uint32_t result_buffer_size =   sizeof(get_single_register_n_samples_response_t) +
                                        samepls_to_send * sizeof(get_single_register_n_samples_response_entry_t);

        uint32_t result_buffer_size_for_allocation =   sizeof(get_two_registers_n_samples_response_t) +
                                        SLICE_SAMPLES * sizeof(get_two_registers_n_samples_response_entry_t);

        char* result_buffer_bytes = malloc(result_buffer_size_for_allocation);
        if (result_buffer_bytes == NULL)
        {
            printf("[%d]: ***ERROR: result buffer size %d bytes too big\n", s, result_buffer_size);
            send_generic_response(s, packet, COMMAND_GET_SINGLE_REGISTER_N_SAMPLES, STATUS_ERROR_NO_MEMORY);
            return;
        } // if failed to malloc() then... //

        //printf("[%d]: about to sample register index %d for #%d samples, interval %lf msec\n", s, cmd->register_index, cmd->num_samples, cmd->interval_msec);

        get_single_register_n_samples_response_t* response = (get_single_register_n_samples_response_t*)result_buffer_bytes;

        response->num_samples = samepls_to_send;
        for (uint32_t i = 0 ; i < samepls_to_send ; i++)
        {
            struct timespec ts;

            const uint32_t value = register_address[0];

            clock_gettime(CLOCK_REALTIME, & ts);

            //
            // Convert to flat 64 bit usec count
            //
            const uint64_t timestamp_u64 = (uint64_t)ts.tv_sec*1000000 + ((uint64_t)ts.tv_nsec / 1000);

            //
            // 32 bits of usec is large enough...
            //
            const uint32_t timestamp_u32 = (uint32_t)timestamp_u64;

            response->samples[i].sample = value;
            response->samples[i].time_stamp = timestamp_u32;

            if (should_sleep)
            {
                nanosleep(& sleep_ts,NULL);
            }
        } // for i loop. //

        // communication_packet_t response_header;
        // response_header.sync = packet->sync;
        // response_header.len  = sizeof(command_response_t) + result_buffer_size ;
        // response_header.type = PACKET_TYPE_RESPONSE;

        command_response_t response_payload;
        response_payload.command = COMMAND_GET_SINGLE_REGISTER_N_SAMPLES;
        response_payload.status  = samples_left == 0 ? 1 : 0;

        send_response2(s, packet, (uint8_t*)& response_payload, sizeof(response_payload), (uint8_t*)result_buffer_bytes, result_buffer_size);
        // send_data(s, (uint8_t*)& response_header    , sizeof(response_header ));
        // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));
        // send_data(s, (uint8_t*)  result_buffer_bytes, result_buffer_size      );

        free(result_buffer_bytes);
        result_buffer_bytes = NULL;
    }

    return;
} // static function handle_get_single_register_n_samples(). //





static void handle_get_two_registers_n_samples(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(get_two_registers_n_samples_payload_t))
    {
        printf("[%d]***ERROR: unexpected get_two_registers_n_samples command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(get_two_registers_n_samples_payload_t));
        return;
    } // if length is too small then... //

    const get_two_registers_n_samples_payload_t* cmd = (const get_two_registers_n_samples_payload_t*) packet->payload;

    if (cmd->register_index_1 > pid_num_regs || cmd->register_index_2 > pid_num_regs)
    {
        printf("[%d]: ***ERROR: register index 1[%d] or register index 2[%d] out of bounds\n", s, cmd->register_index_1, cmd->register_index_2);
        send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_N_SAMPLES, STATUS_INVALID_REGISTER_INDEX);
        return;
    } // if register index is bad then... //

    if (cmd->num_samples > MAX_SAMPLES)
    {
        printf("[%d]: ***ERROR: num samples [%d] exceeds maximum allowed %d\n", s, cmd->num_samples, MAX_SAMPLES);
        send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_N_SAMPLES, STATUS_INVALID_NUM_SAMPLES);
        return;
    } // if register index is bad then... //

    const volatile uint32_t* register_address_1 = GET_PIO_REG_ADDR_BY_INDEX(cmd->register_index_1);
    const volatile uint32_t* register_address_2 = GET_PIO_REG_ADDR_BY_INDEX(cmd->register_index_2);

    struct timespec sleep_ts;
    int             should_sleep = 0;

    if (cmd->interval_msec != 0.)
    {
        sleep_ts.tv_sec  = (__time_t         )( cmd->interval_msec / 1000.                                 );
        sleep_ts.tv_nsec = (__syscall_slong_t)((cmd->interval_msec - (sleep_ts.tv_sec  *1000.) ) * 1000000.);
        should_sleep = 1;
    } // if sleep interval given then... //

    int samples_left = cmd->num_samples;

    int n_allocated_buffers = 0;
    int i_next_buffer = 0;
    char *result_buffer_bytes[MAX_NUM_OF_BUFFESRS];


    while (samples_left > 0) {

        int samepls_to_send = samples_left <= SLICE_SAMPLES ? samples_left : SLICE_SAMPLES;
        samples_left -= samepls_to_send;
    
        uint32_t result_buffer_size =   sizeof(get_two_registers_n_samples_response_t) +
                                        samepls_to_send * sizeof(get_two_registers_n_samples_response_entry_t);
        uint32_t result_buffer_size_for_allocation =   sizeof(get_two_registers_n_samples_response_t) +
                                        SLICE_SAMPLES * sizeof(get_two_registers_n_samples_response_entry_t);

        if (i_next_buffer >= n_allocated_buffers) {
            result_buffer_bytes[i_next_buffer] = malloc(result_buffer_size_for_allocation);
            if (result_buffer_bytes[i_next_buffer] == NULL)
            {
                printf("[%d]: ***ERROR: result buffer size %d bytes too big\n", s, result_buffer_size);
                send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_N_SAMPLES, STATUS_ERROR_NO_MEMORY);
                return;
            } // if failed to malloc() then... //
            n_allocated_buffers = i_next_buffer;
        }

        //printf("[%d]: about to sample registers 1: %d, 2: %d, for #%d samples, interval %lf msec\n", s, cmd->register_index_1, cmd->register_index_2, cmd->num_samples, cmd->interval_msec);

        get_two_registers_n_samples_response_t* response = (get_two_registers_n_samples_response_t*)result_buffer_bytes[i_next_buffer];

        response->num_samples        = samepls_to_send;
        response->register_index_1   = cmd->register_index_1;
        response->register_index_2   = cmd->register_index_2;
        response->register_address_1 = (uint32_t)register_address_1;
        response->register_address_2 = (uint32_t)register_address_2;

        for (uint32_t i = 0 ; i < samepls_to_send ; i++)
        {
            struct timespec ts;

            const uint32_t value_1 = register_address_1[0];
            const uint32_t value_2 = register_address_2[0];

            clock_gettime(CLOCK_REALTIME, & ts);

            //
            // Convert to flat 64 bit usec count
            //
            const uint64_t timestamp_u64 = (uint64_t)ts.tv_sec*1000000 + ((uint64_t)ts.tv_nsec / 1000);

            //
            // 32 bits of usec is large enough...
            //
            const uint32_t timestamp_u32 = (uint32_t)timestamp_u64;

            response->samples[i].sample_1 = value_1;
            response->samples[i].sample_2 = value_2;
            response->samples[i].time_stamp = timestamp_u32;

            // if (should_sleep)
            // {
            //     nanosleep(& sleep_ts,NULL);
            // }
        } // for i loop. //

        // if (    (register_address_1 == GET_PIO_REG_ADDR(o_y_n))
        //     ||  (register_address_1 == GET_PIO_REG_ADDR(o_z_n)) )
        // {
        //     for (uint32_t i = 0 ; i < cmd->num_samples ; i++)
        //     {
        //         const uint32_t sample = response->samples[i].sample_1;
        //         const int32_t  ssample = (const int32_t)sample;
        //         if (ssample > 0xFFFF || ssample < -0xFFFF)
        //         {
        //             printf("WARNING: register 1 %d, sample [%d] = 0x%8.8x (signed: %d)\r\n", cmd->register_index_1, i, sample, ssample);
        //         }
        //     }
        // }

        // communication_packet_t response_header;
        // response_header.sync = packet->sync;
        // response_header.len  = sizeof(command_response_t) + result_buffer_size ;
        // response_header.type = PACKET_TYPE_RESPONSE;

        command_response_t response_payload;
        response_payload.command = COMMAND_GET_TWO_REGISTERS_N_SAMPLES;
        response_payload.status  = samples_left == 0 ? 1 : 0;
        if (samepls_to_send == 300 || samepls_to_send == 100) {
            // support old api
            response_payload.status  = 0;
        }

        send_response2(s, packet, (uint8_t*)& response_payload, sizeof(response_payload), (uint8_t*)result_buffer_bytes[i_next_buffer], result_buffer_size);
        // send_data(s, (uint8_t*)& response_header    , sizeof(response_header ));
        // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));
        // send_data(s, (uint8_t*)  result_buffer_bytes, result_buffer_size      );

        i_next_buffer = (i_next_buffer + 1) % MAX_NUM_OF_BUFFESRS;
    }
    
    for (i_next_buffer = 0; i_next_buffer < n_allocated_buffers; i_next_buffer++) {
        free(result_buffer_bytes[i_next_buffer]);
        result_buffer_bytes[i_next_buffer] = NULL;
    }
    
    return;
} // static function handle_get_two_registers_n_samples(). //

static void handle_get_two_registers_quick_samples(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(get_two_registers_quick_samples_payload_t))
    {
        printf("[%d]***ERROR: unexpected get_two_registers_quick_samples command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(get_two_registers_quick_samples_payload_t));
        return;
    } // if length is too small then... //

    const get_two_registers_quick_samples_payload_t* cmd = (const get_two_registers_quick_samples_payload_t*) packet->payload;

    if (cmd->register_index_1 > 63 || cmd->register_index_2 > 63)
    {
        printf("[%d]: ***ERROR: register index 1[%d] or register index 2[%d] out of bounds\n", s, cmd->register_index_1, cmd->register_index_2);
        send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_QUICK_SAMPLES, STATUS_INVALID_REGISTER_INDEX);
        return;
    } // if register index is bad then... //

    if (cmd->interval_samples > MAX_SAMPLES_INTERVAL)
    {
        printf("[%d]: ***ERROR: interval samples [%d] exceeds maximum allowed %d\n", s, cmd->interval_samples, MAX_SAMPLES_INTERVAL);
        send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_QUICK_SAMPLES, STATUS_INVALID_INTERVAL_SAMPLES);
        return;
    } // if samples interval is bad then... //

    const int rec_index_A = cmd->register_index_1;
    const int rec_index_B = cmd->register_index_2;

    const int samepls_to_send = 1024;
    uint32_t result_buffer_size =   sizeof(get_two_registers_quick_samples_response_t) +
                                        samepls_to_send * sizeof(get_two_registers_quick_samples_response_entry_t);
    char* result_buffer_bytes = malloc(result_buffer_size);
    if (result_buffer_bytes == NULL)
    {
        printf("[%d]: ***ERROR: result buffer size %d bytes too big\n", s, result_buffer_size);
        send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_QUICK_SAMPLES, STATUS_ERROR_NO_MEMORY);
        return;
    } // if failed to malloc() then... //


    get_two_registers_quick_samples_response_t* response = (get_two_registers_quick_samples_response_t*)result_buffer_bytes;

    response->num_samples        = samepls_to_send;
    response->register_index_1 = (uint32_t)rec_index_A;
    response->register_index_2 = (uint32_t)rec_index_B;


    // 1. Configure which registers to capture and set interval
    WRITE_PIO_REG(recorder_i3_cs,      0x0001);          // select CS[0]
    WRITE_PIO_REG(recorder_i3_write,   1);
    WRITE_PIO_REG(recorder_i3_data_in, (rec_index_B << 6) | rec_index_A | (1 << 12));
    //                     ^^^ this also sets enable=1, starts capture
    WRITE_PIO_REG(recorder_i3_write,   0);
    // 2. Set sampling interval (e.g. every 10th clock tick)
    WRITE_PIO_REG(recorder_i3_cs,      0x0004);          // CS[2] = interval register
    WRITE_PIO_REG(recorder_i3_write,   1);
    WRITE_PIO_REG(recorder_i3_data_in, 10);
    WRITE_PIO_REG(recorder_i3_write,   0);

    // 3. Poll status until enable drops to 0 (buffer full)
    while (READ_PIO_REG(recorder_o3_status) & 0x1) {
        usleep(100);
    }


    uint32_t status = READ_PIO_REG(recorder_o3_status);
    printf("status: enable=%d counter=%d\n", status & 1, (status >> 1) & 0x3FF);

    // 4. Read back samepls_to_send words from each buffer
    WRITE_PIO_REG(recorder_i3_cs,    0x0002);            // CS[1] = set read address
    for (int i = 0; i < samepls_to_send; i++) {
        WRITE_PIO_REG(recorder_i3_data_in, i);
        WRITE_PIO_REG(recorder_i3_write,   1);
        WRITE_PIO_REG(recorder_i3_write,   0);
        WRITE_PIO_REG(recorder_i3_cs,   0x0008);         // CS[3] = read buf_A
        WRITE_PIO_REG(recorder_i3_read, 1);
        WRITE_PIO_REG(recorder_i3_read, 0);
        usleep(1);
        response->samples[i].sample_1 = READ_PIO_REG(recorder_o3_data_out);
        if (i < 32) {
            printf("sampleA %d: 0x%8.8x\r\n", i, response->samples[i].sample_1);
        }
        WRITE_PIO_REG(recorder_i3_cs,   0x0010);         // CS[4] = read buf_B
        WRITE_PIO_REG(recorder_i3_read, 1);
        WRITE_PIO_REG(recorder_i3_read, 0);
        usleep(1);
        response->samples[i].sample_2 = READ_PIO_REG(recorder_o3_data_out);
        if (i <32) {
            printf("sampleB %d: 0x%8.8x\r\n", i, response->samples[i].sample_2);
        }
        
    }

    command_response_t response_payload;
    response_payload.command = COMMAND_GET_TWO_REGISTERS_QUICK_SAMPLES;
    response_payload.status  = 1;

    send_response2(s, packet, (uint8_t*)& response_payload, sizeof(response_payload), (uint8_t*)result_buffer_bytes, result_buffer_size);
    
}


static void handle_set_2nd_integrator_enable(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_2nd_integrator_enable_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_2nd_integrator_enable_payload_t command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_2nd_integrator_enable_payload_t));
        return;
    } // if length is too small then... //

    const set_2nd_integrator_enable_payload_t* cmd = (const set_2nd_integrator_enable_payload_t*) packet->payload;

    printf("[%d]: setting 2nd integrator enable to[%d] (value if disabled[%d])\n", s, cmd->is_enabled, cmd->disabled_value);

    g_server.predictor_state.i2_enabled        = cmd->is_enabled    ;
    g_server.predictor_state.i2_disabled_value = cmd->disabled_value;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_2ND_INTEGRATOR_ENABLE, 0);
} // static function handle_set_2nd_integrator_enable(). //




static void handle_set_manual_dac_output(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_manual_dac_output_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_manual_dac_output_payload_t command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_manual_dac_output_payload_t));
        return;
    } // if length is too small then... //

    const set_manual_dac_output_payload_t* cmd = (const set_manual_dac_output_payload_t*) packet->payload;

    printf("[%d]: setting manual dac output enable to[%d] (value if manual[%d])\n", s, cmd->is_manual_dac_output_enabled, cmd->manual_dac_output_value);

    g_server.predictor_state.manual_dac_output_enable = cmd->is_manual_dac_output_enabled;
    g_server.predictor_state.manual_dac_output_value  = cmd->manual_dac_output_value;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_MANUAL_DAC_OUTPUT, 0);
} // static function handle_set_2nd_integrator_enable(). //


static void handle_set_register_and_get_two_register_samples(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_register_and_get_two_registers_samples_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_register_and_get_two_registers_samples_payload_t command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_register_and_get_two_registers_samples_payload_t));
        return;
    } // if length is too small then... //

    const set_register_and_get_two_registers_samples_payload_t* cmd = (const set_register_and_get_two_registers_samples_payload_t*) packet->payload;

    //
    // Verify that the given # of samples is precisely consistent with the packet length
    //
    const uint32_t expected_packet_len = sizeof(set_register_and_get_two_registers_samples_payload_t)
                                         +
                                         (cmd->num_samples * sizeof(cmd->samples[0]))
                                         ;

    if (packet->len != expected_packet_len)
    {
        printf("[%d]: ***ERROR: for # of samples %d expected length %d while having %d\n",
                   s,
                   cmd->num_samples,
                   expected_packet_len,
                   packet->len);
        send_generic_response(s, packet, COMMAND_SET_REGISTER_AND_GET_TWO_REGISTER_SAMPLES, STATUS_INVALID_PACKET_LENGTH);
        return;
    } // if packet length inconsistent then... //

    //
    // Verify register indices are ok
    //
    if (    cmd->set_register_index     > pid_num_regs
        ||  cmd->get_register_index_1   > pid_num_regs
        ||  cmd->get_register_index_2   > pid_num_regs)
    {
        printf("[%d]: ***ERROR: set register index[%d] or get register index 1[%d] or get register index 2[%d] out of bounds\n",
                   s,
                   cmd->set_register_index,
                   cmd->get_register_index_1,
                   cmd->get_register_index_2);
        send_generic_response(s, packet, COMMAND_SET_REGISTER_AND_GET_TWO_REGISTER_SAMPLES, STATUS_INVALID_REGISTER_INDEX);
        return;
    } // if register index is bad then... //

    //
    // Not handling too many samples
    //
    if (cmd->num_samples > MAX_SAMPLES)
    {
        printf("[%d]: ***ERROR: num samples [%d] exceeds maximum allowed %d\n", s, cmd->num_samples, MAX_SAMPLES);
        send_generic_response(s, packet, COMMAND_SET_REGISTER_AND_GET_TWO_REGISTER_SAMPLES, STATUS_INVALID_NUM_SAMPLES);
        return;
    } // if register index is bad then... //

    volatile uint32_t*       set_register_address   = GET_PIO_REG_ADDR_BY_INDEX(cmd->set_register_index); // Note: this is not const! We must write to this!
    const volatile uint32_t* get_register_address_1 = GET_PIO_REG_ADDR_BY_INDEX(cmd->get_register_index_1);
    const volatile uint32_t* get_register_address_2 = GET_PIO_REG_ADDR_BY_INDEX(cmd->get_register_index_2);;


    struct timespec sleep_ts;
    int             should_sleep = 0;

    if (cmd->interval_msec != 0.)
    {
        sleep_ts.tv_sec  = (__time_t         )( cmd->interval_msec / 1000.                                 );
        sleep_ts.tv_nsec = (__syscall_slong_t)((cmd->interval_msec - (sleep_ts.tv_sec  *1000.) ) * 1000000.);
        should_sleep = 1;
    } // if sleep interval given then... //

    //
    // The result buffer is the same as the one for the command GET_TWO_REGISTERS...
    //
    uint32_t result_buffer_size =       sizeof(get_two_registers_n_samples_response_t)
                                    +     cmd->num_samples
                                        * sizeof(get_two_registers_n_samples_response_entry_t)
                                        ;

    char* result_buffer_bytes = malloc(result_buffer_size);
    if (result_buffer_bytes == NULL)
    {
        printf("[%d]: ***ERROR: result buffer size %d bytes too big\n", s, result_buffer_size);
        send_generic_response(s, packet, COMMAND_GET_TWO_REGISTERS_N_SAMPLES, STATUS_ERROR_NO_MEMORY);
        return;
    } // if failed to malloc() then... //

    printf("[%d]: about to set register [%d] and sample registers 1: %d, 2: %d, for #%d samples, interval %lf msec\n",
               s,
               cmd->set_register_index,
               cmd->get_register_index_1,
               cmd->get_register_index_2,
               cmd->num_samples,
               cmd->interval_msec);

    get_two_registers_n_samples_response_t* response = (get_two_registers_n_samples_response_t*)result_buffer_bytes;

    response->num_samples        = cmd->num_samples;
    response->register_index_1   = cmd->get_register_index_1;
    response->register_index_2   = cmd->get_register_index_2;
    response->register_address_1 = (uint32_t)get_register_address_1;
    response->register_address_2 = (uint32_t)get_register_address_2;
    for (uint32_t i = 0 ; i < cmd->num_samples ; i++)
    {
        struct timespec ts;

        //
        // 1st, write the sample.
        //
        set_register_address[0] = cmd->samples[i];

        //
        // To allow stabilization, we read the registers after the interval.
        //
        if (should_sleep)
        {
            nanosleep(& sleep_ts,NULL);
        }

        //
        // Now read both registers
        //
        const uint32_t value_1 = get_register_address_1[0];
        const uint32_t value_2 = get_register_address_2[0];

        clock_gettime(CLOCK_REALTIME, & ts);

        //
        // Convert to flat 64 bit usec count
        //
        const uint64_t timestamp_u64 = (uint64_t)ts.tv_sec*1000000 + ((uint64_t)ts.tv_nsec / 1000);

        //
        // 32 bits of usec is large enough...
        //
        const uint32_t timestamp_u32 = (uint32_t)timestamp_u64;

        response->samples[i].sample_1 = value_1;
        response->samples[i].sample_2 = value_2;
        response->samples[i].time_stamp = timestamp_u32;

    } // for i loop. //

    // communication_packet_t response_header;
    // response_header.sync = packet->sync;
    // response_header.len  = sizeof(command_response_t) + result_buffer_size ;
    // response_header.type = PACKET_TYPE_RESPONSE;

    command_response_t response_payload;
    response_payload.command = COMMAND_SET_REGISTER_AND_GET_TWO_REGISTER_SAMPLES;
    response_payload.status  = 0;

    send_response2(s, packet, (uint8_t*)& response_payload, sizeof(response_payload), (uint8_t*)result_buffer_bytes, result_buffer_size);
    // send_data(s, (uint8_t*)& response_header    , sizeof(response_header ));
    // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));
    // send_data(s, (uint8_t*)  result_buffer_bytes, result_buffer_size      );

    free(result_buffer_bytes);
    result_buffer_bytes = NULL;

    return;
} // static function handle_set_register_and_get_two_register_samples(). //


static void handle_select_output_signal(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(select_output_signal_payload_t))
    {
        printf("[%d]***ERROR: unexpected select_output_signal_payload command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(select_output_signal_payload_t));
        return;
    } // if length is too small then... //

    const select_output_signal_payload_t* cmd = (const select_output_signal_payload_t*) packet->payload;

    if (cmd->signal_index > 15)
    {
        printf("[%d]: ***ERROR: signal index[%d]  out of bounds\n", s, cmd->signal_index);
        send_generic_response(s, packet, COMMAND_SELECT_OUTPUT_SIGNAL, STATUS_INVALID_SIGNAL_INDEX);
        return;
    }

    printf("[%d]: selecting signal index %d\n", s, cmd->signal_index);

    g_server.predictor_state.output_signal_index = cmd->signal_index;
    set_predictor();

    send_generic_response(s, packet, COMMAND_SELECT_OUTPUT_SIGNAL, 0);
} // static function handle_set_output_signal(). //



static void handle_scan_begin(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(scan_begin_payload_t))
    {
        printf("[%d]***ERROR: unexpected scan_begin_payload command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(scan_begin_payload_t));
        return;
    } // if length is too small then... //

    const scan_begin_payload_t* cmd = (const scan_begin_payload_t*) packet->payload;

    hex_dump("scan begin dump", (const char *)cmd, sizeof(*cmd));

    scan_state_apply(& g_server.predictor_state.scan_state, cmd);

    send_generic_response(s, packet, COMMAND_SCAN_BEGIN, 0);
} // static function handle_scan_begin(). //



static void handle_scan_end(const int s, const communication_packet_t* packet)
{
    printf("scan end command received\n");
    scan_state_disable(& g_server.predictor_state.scan_state);

    send_generic_response(s, packet, COMMAND_SCAN_END, 0);
} // static function handle_scan_end(). //




static void handle_set_dithering_parameters(const int s, const communication_packet_t* packet)
{
    if (packet->len < sizeof(set_dithering_parameters_payload_t))
    {
        printf("[%d]***ERROR: unexpected set_dither_parameters command len %d (expecting %d)\n",
               s,
               packet->len,
               sizeof(set_dithering_parameters_payload_t));
        return;
    } // if length is too small then... //

    const set_dithering_parameters_payload_t* cmd = (const set_dithering_parameters_payload_t *)packet->payload;


    printf("[%d]: dither enable: output %d (was %d), input %d (was %d)\n"
           "      output amplitude %d (was %d), output count %d (was %d)\n"
           "      input phase 1 count %d (was %d), phase 2 count %d (was %d)\n"
           "      input init polarity %d (was %d)\n",
           s,
           cmd->output_enable       , g_server.predictor_state.dither_parameters.output_enabled       ,
           cmd->input_enable        , g_server.predictor_state.dither_parameters.input_enabled        ,
           cmd->output_amplitude    , g_server.predictor_state.dither_parameters.output_amplitude     ,
           cmd->output_phase_1_count, g_server.predictor_state.dither_parameters.output_phase_1_count ,
           cmd->input_phase_1_count , g_server.predictor_state.dither_parameters.input_phase_1_count  ,
           cmd->input_phase_2_count , g_server.predictor_state.dither_parameters.input_phase_2_count  ,
           cmd->input_init_polarity , g_server.predictor_state.dither_parameters.input_init_polarity
           );


    g_server.predictor_state.dither_parameters.output_enabled       = cmd->output_enable       ;
    g_server.predictor_state.dither_parameters.input_enabled        = cmd->input_enable        ;
    g_server.predictor_state.dither_parameters.output_amplitude     = cmd->output_amplitude    ;
    g_server.predictor_state.dither_parameters.output_phase_1_count = cmd->output_phase_1_count;
    g_server.predictor_state.dither_parameters.input_phase_1_count  = cmd->input_phase_1_count ;
    g_server.predictor_state.dither_parameters.input_phase_2_count  = cmd->input_phase_2_count ;
    g_server.predictor_state.dither_parameters.input_init_polarity  = cmd->input_init_polarity ;

    set_predictor();

    send_generic_response(s, packet, COMMAND_SET_DITHERING_PARAMETERS, 0);
} // function handle_set_dithering_parameters(). //




static void handle_stop(const int s, const communication_packet_t* packet)
{
    printf("[%d]: about to stop predictor...\n",s);

    if (!g_server.predictor_state.started)
    {
        printf("[%d]: Warning: already stopped!\n",s);
        send_generic_response(s, packet, COMMAND_STOP, STATUS_WARNING_ALREADY_STOPPED);
        return;
    }

    g_server.predictor_state.started = 0;

    set_predictor();

    send_generic_response(s, packet, COMMAND_STOP, 0);
} // function handle_stop(). //


static void handle_start(const int s, const communication_packet_t* packet)
{
    printf("[%d]: about to start predictor...\n",s);

    if (g_server.predictor_state.started)
    {
        printf("[%d]: Warning: already started!\n",s);
        send_generic_response(s, packet, COMMAND_STOP, STATUS_WARNING_ALREADY_STARTED);
        return;
    }

    g_server.predictor_state.started = 1;

    set_predictor();

    send_generic_response(s, packet, COMMAND_START, 0);
} // function handle_start(). //




static void handle_reboot(const int s, const communication_packet_t* packet)
{
    printf("[%d]: !!!!about to reboot!!!!...\n",s);

    send_generic_response(s, packet, COMMAND_REBOOT, 0);
    close(s);

    sleep(1);
    printf("[%d]: !!!!rebooting now!!!!...\n",s);

    int rc = system("reboot");
    if (rc) {
        printf("[%d]: failed to reboot, rc=%d, errno=%d\n", s, rc, errno);
    } else {
        printf("rebooting...\n");
    }
} // function handle_start(). //





static int handle_command(const int s, const communication_packet_t* packet)
{
    int rc = 0;

    const command_payload_t* cmd = (const command_payload_t*)packet->payload;

    switch(cmd->command)
    {
        case COMMAND_ERROR:
            printf("[%d]***ERROR: unexpected command error!\n", s);
            return -1;

        case COMMAND_SET_PREDICTOR_ALPHA:
            handle_set_predictor_alpha(s,packet);
            return 0;

        case COMMAND_SET_PREDICTOR_ORDER:
            handle_set_predictor_order(s,packet);
            return 0;

        case COMMAND_SET_AVERAGING_TIME:
            handle_set_averaging_time(s,packet);
            return 0;

        case COMMAND_SET_GAIN:
            handle_set_gain(s,packet);
            return 0;

        case COMMAND_SET_OUTPUT_SHIFT:
            handle_set_output_shift(s,packet);
            return 0;

        case COMMAND_SET_OUTPUT_OFFSET:
            handle_set_output_offset(s,packet);
            return 0;

        case COMMAND_SET_INPUT_SELECT:
            handle_set_input_select(s,packet);
            return 0;

        case COMMAND_SET_INPUT_OFFSET:
            handle_set_input_offset(s,packet);
            return 0;

        case COMMAND_SET_INPUT_AVERAGING_ENABLE:
            handle_set_input_averaging_enable(s,packet);
            return 0;

        case COMMAND_SET_DITHERING_PARAMETERS:
            handle_set_dithering_parameters(s,packet);
            return 0;

        case COMMAND_GET_SINGLE_REGISTER_N_SAMPLES:
            handle_get_single_register_n_samples(s,packet);
            return 0;

        case COMMAND_GET_TWO_REGISTERS_N_SAMPLES:
            handle_get_two_registers_n_samples(s,packet);
            return 0;

        case COMMAND_GET_TWO_REGISTERS_QUICK_SAMPLES:
            handle_get_two_registers_quick_samples(s,packet);
            return 0;

        case COMMAND_STOP:
            handle_stop(s,packet);
            return 0;

        case COMMAND_START:
            handle_start(s,packet);
            return 0;

        case COMMAND_REBOOT:
            handle_reboot(s,packet);
            return 0;

        case COMMAND_SET_REGISTER_AND_GET_TWO_REGISTER_SAMPLES:
            handle_set_register_and_get_two_register_samples(s,packet);
            return 0;

        case COMMAND_SET_2ND_INTEGRATOR_ENABLE:
            handle_set_2nd_integrator_enable(s,packet);
            return 0;

        case COMMAND_SET_MANUAL_DAC_OUTPUT:
            handle_set_manual_dac_output(s,packet);
            return 0;

        case COMMAND_SELECT_OUTPUT_SIGNAL:
            handle_select_output_signal(s,packet);
            return 0;

        case COMMAND_GET_VERSION:
        {
            printf("[%d]: get version command\n",s);
            // communication_packet_t response;
            // response.sync = packet->sync;
            // response.len  = sizeof(version_string) + sizeof(command_response_t);
            // response.type = PACKET_TYPE_RESPONSE;

            command_response_t response_payload;
            response_payload.command = COMMAND_GET_VERSION;
            response_payload.status  = 0;

            send_response2(s, packet, (uint8_t*)&response_payload, sizeof(response_payload), (uint8_t*)version_string, sizeof(version_string  ));

            // send_data(s, (uint8_t*)& response           , sizeof(response        ));
            // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));
            // send_data(s, (uint8_t*)version_string       , sizeof(version_string  ));
        }
        break;

        case COMMAND_GET_REGISTER_DUMP:
        {
            //printf("[%d]: get register dump\n",s);
            handle_register_dump_response(s, packet);
        }
        break;

        case COMMAND_PING:
        {
            printf("[%d]: ping\n",s);

            // communication_packet_t response;
            // response.sync = packet->sync;
            // response.len  = sizeof(command_response_t);
            // response.type = PACKET_TYPE_RESPONSE;

            command_response_t response_payload;
            response_payload.command = COMMAND_PING;
            response_payload.status  = 0;

            send_response(s, packet, (uint8_t*)&response_payload, sizeof(response_payload));
            // send_data(s, (uint8_t*)& response           , sizeof(response        ));
            // send_data(s, (uint8_t*)& response_payload   , sizeof(response_payload));
        }
        break;

        case COMMAND_SCAN_BEGIN:
        {
            handle_scan_begin(s,packet);

        }
        break;

        case COMMAND_SCAN_END:
        {
            handle_scan_end(s,packet);
        }
        break;


        default:
        {
            printf("[%d] command[%d] not handled yet\n", s, cmd->command);
        }
    }

    return rc;
} // function handle_command(). //



static int handle_packet(const int s, const communication_packet_t* packet)
{
    int rc = 0;

    switch(packet->type)
    {
        case PACKET_TYPE_ERROR:
            printf("[%d]***ERROR: unexpected command error!\n", s);
            return -1;

        case PACKET_TYPE_COMMAND:
            return handle_command(s, packet);

        default:
        {
            printf("[%d] command[%d] not handled yet\n", s, packet->type);
        }
    }

    return rc;
} // function handle_packet(). //


static void* client_thread_main(void* arg)
{
    const int client_socket = (int)arg;

    printf("client_thread 1.1: socket[%d] starting...\n", client_socket);
    for (int i = 0; i < 300; i++) {
        printf("=");
        if (i % 50 == 49) {
            printf("\n");
        }
    }


    int should_continue = 1;
    int count = 0;

    while(should_continue)
    {
        count ++ ;
        if ((count & 0xFFF) == 0)
        {
            printf("socket[%d] accepted %d commands\n", client_socket, count);
        }
        printf("command count %d, getting packet header\n", count);
        communication_packet_t packet_header = get_packet_header(client_socket);
        printf("got packet len %d <<<<<<<< ", packet_header.len);

        if (packet_header.type == COMMAND_ERROR)
        {
            printf("[%d]***ERROR: error in reception of header, aborting\n", client_socket);
            should_continue = 0;
        }
        else
        {
            const size_t payload_size = packet_header.len;
            const size_t packet_size  = sizeof(packet_header) + payload_size;
            uint8_t*    packet = malloc(packet_size);
            if (packet == NULL)
            {
                printf("[%d]***ERROR: failed to allocate [%d] bytes, errno %d\n", client_socket, packet_size, errno);
                should_continue = 0;
            }
            else
            {
                memcpy(packet, & packet_header, sizeof(packet_header));

                const size_t remaining_size = get_data(      client_socket                      ,
                                                           & packet[sizeof(packet_header)]      ,
                                                           payload_size                         );



                if (remaining_size == -1)
                {
                    printf("[%d]***ERROR: failed to retrieve payload\n", client_socket);
                    free(packet);
                }
                else if (remaining_size != payload_size)
                {
                    printf("[%d]***ERROR: expected [%d] bytes but got less[%d]\n", client_socket, payload_size, remaining_size);
                    free(packet);
                }
                else
                {
                    //
                    // packet received ok, now handle it...
                    //
                    for (int i = 0; i < ((communication_packet_t*)packet)->len; i++) {
                        printf(" %02x", ((communication_packet_t*)packet)->payload[i]);
                    }
                    printf(" >>>>>>>>\n");

                    int rc = handle_packet(client_socket, (communication_packet_t*)packet);

                    free(packet);

                    if (rc != 0)
                    {
                        printf("[%d]***ERROR: failed to handle packet\n", client_socket);
                    }
                    else if ((count & 0xFFF) == 0)
                    {
                        printf("[%d]packet[%d] handled ok (count %d)\n", client_socket, packet_header.type, count);
                    }
                } // else--> remaining data got ok... //
            } // else--> got a packet to receive into... //
        }
    }

    printf("client [%d] thread ending... disabling scan if was active\n", client_socket);
    scan_state_disable(& g_server.predictor_state.scan_state);

    return NULL;
} // function  client_thread_main(). //



static
void handle_client( server_t* server, const int client_socket, const struct sockaddr_in* client_addr, const socklen_t client_addr_len)
{
    const char* client_addr_str = inet_ntoa(client_addr->sin_addr);

    printf("new client (fd:%d), from addr[%s:%d]\n", client_socket, client_addr_str, client_addr->sin_port);

    pthread_t      client_thread;
    pthread_attr_t client_thread_attr = {0};

    pthread_attr_init(&client_thread_attr);

    int rc = pthread_create(& client_thread, & client_thread_attr, client_thread_main, (void*) client_socket);

    if (rc != 0)
    {
        printf("**ERROR: failed to create thread for client, errno %d\n", errno);
    }
} // function handle_client(). //




void* server_thread_main(void* arg)
{
    server_t* server = (server_t*)arg;
    printf("server: starting....\n");

    struct sockaddr_in  server_addr;


    server->socket = socket(AF_INET, SOCK_STREAM, 0);

    if (server->socket < 0)
    {
        printf("***ERROR: failed to open socket, errno %d\n", errno);
        exit(-1);
        return NULL;
    }

    memset(& server_addr, 0x00, sizeof(server_addr));

    server_addr.sin_addr.s_addr = 0;
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(SERVER_PORT);

    int rc = bind(server->socket, & server_addr, sizeof(server_addr));

    if (rc != 0)
    {
        printf("***ERROR: bind() failed, errno %d\n", errno);
        goto end;
    }

    rc = listen(server->socket, 5);
    if (rc != 0)
    {
        printf("***ERROR: listen() failed, errno %d\n", errno);
    }

    int should_continue = 1;
    while (should_continue)
    {
        struct sockaddr_in  client_addr;
        socklen_t           client_addr_len = sizeof(client_addr);

        printf("server 1.1 accepts on socket[%d] on port[%d]\n", g_server.socket, SERVER_PORT);
        fflush(stdout);
        int client_socket = accept(server->socket, & client_addr, &client_addr_len);

        if (client_socket < 0)
        {
            printf("***ERROR: accept() failed, errno %d\n", errno);
            should_continue = 0;
        } // if accept failed then... //
        else
        {
            handle_client(server, client_socket, &client_addr, client_addr_len);
        } // else --> all is well... //
    } // while loop. //
end:
    close(g_server.socket);
    return NULL;
} // function server_thread_main(). //


static
void init_predictor(server_t* server)
{
    server->predictor_state.alpha               = 1.0;
    server->predictor_state.averaging_enable    = 1;
    server->predictor_state.input_select        = 0;
    server->predictor_state.delay               = 0;
    server->predictor_state.i2_gain             = 0;
    server->predictor_state.i_gain              = 0;
    server->predictor_state.p_gain              = 1;
    server->predictor_state.input_offset        = 8192;
    server->predictor_state.output_offset       = -8192;
    server->predictor_state.n                   = 4;
    server->predictor_state.output_shift        = 0;
    server->predictor_state.started             = 0;

    union predictor_hw_reg_o_config_u  config;
    union dacb_output_config_u         dacb_config;

    config._w32 = 0;
    config.c.input_averaging_enable = server->predictor_state.averaging_enable  ;
    config.c.input_select           = server->predictor_state.input_select      ;
    config.c.continuous             = server->predictor_state.started           ;
    config.c.do_delay               = (server->predictor_state.delay != 0)      ;
    config.c.output_shift           = server->predictor_state.output_shift      ;

    dacb_config._w32 = 0;
    dacb_config.c.predictor_z_n = 1;

    printf("------ offsets %d %d\n", server->predictor_state.output_offset, server->predictor_state.output_offset_2nd);
    WRITE_PIO_REG(o_config        , config._w32                           ) ;
    WRITE_PIO_REG(o_dacb_output   , dacb_config._w32                      ) ;
    WRITE_PIO_REG(o_delay_count   , server->predictor_state.delay         ) ;
    WRITE_PIO_REG(o_i0            , server->predictor_state.i_gain        ) ;
    WRITE_PIO_REG(o_out_offset    , server->predictor_state.output_offset ) ;
    WRITE_PIO_REG(o_2nd_out_offset, server->predictor_state.output_offset_2nd) ;
    WRITE_PIO_REG(o_y_reference   , server->predictor_state.input_offset  ) ;

    set_alpha(server->predictor_state.alpha, server->predictor_state.i_gain, server->predictor_state.n);

} // function init_predictor() //


void server_main(void)
{
    printf("server: starting....\n");

    pthread_t      server_thread;
    pthread_attr_t server_thread_attr = {0};

    init_predictor(&g_server);

    pthread_attr_init(&server_thread_attr);

    int rc = pthread_create(& server_thread, & server_thread_attr, server_thread_main, &g_server);

    sleep(1);
    if (rc != 0)
    {
        printf("**ERROR: failed to create thread for server, errno %d\n", errno);
    }
    else
    {
        printf("server main thread started...\n");
    }
} // function server_main(). //
