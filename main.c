#define _GNU_SOURCE
#define soc_cv_av

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
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

#include "registers.h"
#include "predictor_api.h"
#include "version.h"

volatile uint32_t g_time_stub = 0;

void *virtual_base = NULL;

void *virtual_base_axi = NULL;


//#include "pid_simple.h"

const volatile uint32_t* pio_a2d_data;
volatile       uint32_t* pio_d2a_data;

const volatile uint32_t* pio_axi_a2d_data;
volatile       uint32_t* pio_axi_d2a_data;

volatile uint32_t   g_test;
volatile       uint32_t* pio_d2a_data_test = &g_test;

//pid_hw_regs_t*        pid_hw_regs       ;
//predictor_hw_regs_t*   predictor_hw_regs ;
//predictor2_hw_regs_t*  predictor2_hw_regs ;


pid_pio_regs_t  *g_predictor_regs = NULL;

//#define __MIN__(x,y)    (((x) > (y)) ? (y) : (x))
//#define __MAX__(x,y)    (((x) > (y)) ? (x) : (y))

#define __MIN__(x,y)    fmin(x,y)
#define __MAX__(x,y)    fmax(x,y)


static void print_time_diff(const char* title, const struct timespec *t0, const struct timespec* t1, const int32_t n)
{
//    printf("diff: t0: sec %d, nsec %d    ---    t1: sec %d, nsec %d\n",
//           (int)t0->tv_sec,
//           (int)t0->tv_nsec,
//           (int)t1->tv_sec,
//           (int)t1->tv_nsec);
//
    const static int64_t  ns_in_sec = 1000000000;

    const int64_t t0_ns = (int64_t)t0->tv_sec * ns_in_sec + (int64_t)t0->tv_nsec;
    const int64_t t1_ns = (int64_t)t1->tv_sec * ns_in_sec + (int64_t)t1->tv_nsec;
    const int64_t diff  = t1_ns - t0_ns ;
    const float   f_diff= (float)diff;

    printf("%s: %d steps : %g ns per step\n", title, n, f_diff/(float)n);
} // function print_time_diff(). //


void hex_dump(const char *title, const char *buffer, const size_t size)
{
    printf("%s: starting dump of address %p length %u\n", title, buffer, size);
    const static size_t bytes_per_line = 16;
    const static char *printables = "ABCDEFGHIJKLMNOPQRSTUVWYZ"
                                   "abcdefghijklmnopqrstuvwyz"
                                   "0123456789"
                                   "!@#$%^&*()-=_+[]{};':,.<>/?";

    int64_t remainder = (int64_t)size;
    const char* offset = buffer;
    while (remainder >= 0) {
        printf("%p: (0x%8.8x) ", offset, (offset - buffer));
        const uint32_t *offset_w32 = (const uint32_t *)offset;
        for (int i = 0 ; i < bytes_per_line/sizeof(uint32_t) ; i++)
        {
            if (remainder - i*sizeof(uint32_t) > 0)
            {
                const uint32_t word32 = offset_w32[i];
                printf(" 0x%8.8x", word32);
            }
            else
            {
                printf(" __________");
            }
        }

        printf(" | ");

        for (int i = 0 ; i < bytes_per_line ; i++)
        {
            if (remainder - i > 0)
            {
                const uint8_t byte = offset[i];

                if (byte && strchr(printables, (char)byte))
                {
                    printf("%c", byte);
                }
                else
                {
                    printf(".");
                }
            }
            else
            {
                printf("_");
            }
        }
        printf(" |\n");
        remainder -= bytes_per_line;
        offset += bytes_per_line;
    }
}


volatile uint32_t k = 0;
uint32_t peek(const uint32_t offset)
{
    if (virtual_base != NULL)
    {
        const volatile uint32_t* addr = virtual_base;

        const volatile uint32_t* actual  = &addr[offset];
        const uint32_t value = actual[0];

        printf("Returning addr 0x%p = 0x%8.8x\n", actual, value);

        return value;
    }
    else
    {
        printf("mmap not opened?\n");
        return -1;
    }
}

uint32_t write_dac(const uint32_t v)
{
    if (pio_axi_d2a_data!= NULL)
    {
        pio_axi_d2a_data[0] = v;
        printf("Written 0x%8.8x to %p\n", v, pio_axi_d2a_data);
        return  0;
    }
    else
    {
        printf("mmap not opened?\n");
        return -1;
    }
}




//
// read_adc_to_file()
// Write samples to text file using the following format:
// uint64_t  timestamp (usec resolution)     uint32_t sample
//
void     read_adc_to_file(  const char*     filename    ,
                            const uint32_t  num_samples )
{
    uint64_t timestamp;
    FILE*    fp = fopen(filename, "wt");

    if (fp == NULL)
    {
        const int le = errno;
        printf("***ERROR: failed to open file %s for write, errno %d\n", filename, le);
        return;
    }

    uint32_t i ;

    for (i = 0 ; i < num_samples ; i++)
    {
        struct timespec ts;

        clock_gettime(CLOCK_REALTIME, & ts);

        timestamp = (uint64_t)ts.tv_sec*1000000 + ((uint64_t)ts.tv_nsec / 1000);

        const uint32_t sample = pio_axi_a2d_data[0];

        fprintf(fp, "%lld  %d\n",timestamp, sample);

    }
    fclose(fp);

} // function read_adc_to_file(). //



uint32_t read_dac(void)
{
    if (pio_axi_a2d_data != NULL)
    {
        const uint32_t v = pio_axi_a2d_data[0];
        printf("A2D: %p ==> 0x%8.8x \n",pio_axi_a2d_data,v);
        return  v;
    }
    else
    {
        printf("mmap not opened?\n");
        return -1;
    }
}



void check_dac_performance(void)
{
    printf("checking performance.\n");

    struct timespec resolution;
    const int res_rc = clock_getres(CLOCK_REALTIME, &resolution);

    if (res_rc != 0)
    {
        printf("clock_getres() failed, error = %d\n", errno);
        return;
    } // if failed then... //

    printf("clock_getres() result: resolution: sec %lld, nsec %ld\n", resolution.tv_sec, resolution.tv_nsec);

    printf("axi base: %p, virtual axi base %p, pio_axi_d2a_data %p, offset 0x%x\n",
           (void*)0xC0000000,
           virtual_base_axi,
           pio_axi_d2a_data,
           (uint32_t)pio_axi_d2a_data - (uint32_t)virtual_base_axi);


    struct timespec t0, t1, t2;
    int clk_rc = clock_gettime(CLOCK_REALTIME, & t0);

    if (clk_rc != 0)
    {
        printf("clock_gettime() for t0 failed, errno %d\n", errno);
        return;
    } // if failed then... //

    int i = 0 ;

    uint32_t data = 0;
    for (i = 0 ; i < 1000000; i++)
    {
        pio_axi_d2a_data[0] = data;
        data = ~data;
    }

    clk_rc = clock_gettime(CLOCK_REALTIME, & t1);
    if (clk_rc != 0)
    {
        printf("clock_gettime() for t1 failed, errno %d\n", errno);
        return;
    } // if failed then... //

    for (i = 0 ; i < 1000000; i++)
    {
        pio_d2a_data_test[0] = data;
        data = ~data;
    }

    clk_rc = clock_gettime(CLOCK_REALTIME, & t2);
    if (clk_rc != 0)
    {
        printf("clock_gettime() for t1 failed, errno %d\n", errno);
        return;
    } // if failed then... //

    printf("t0: sec %lld, nsec %ld, t1: sec %lld, nsec %ld\n", t0.tv_sec, t0.tv_nsec, t1.tv_sec, t1.tv_nsec);
    printf("t2: sec %lld, nsec %ld, \n", t2.tv_sec, t2.tv_nsec);

    int32_t nsec_diff = t1.tv_nsec - t0.tv_nsec;
    int32_t sec_diff  = 0;
    if (nsec_diff < 0)
    {
        sec_diff -- ;
        nsec_diff += 1000000000 ; // take a carry (1 sec = 10^9 nsec) and add it to the nsec diff.
    }

    sec_diff += t1.tv_sec - t0.tv_sec ;

    int32_t nsec_diff_2 = t2.tv_nsec - t1.tv_nsec;
    int32_t sec_diff_2  = 0;
    if (nsec_diff_2 < 0)
    {
        sec_diff_2 -- ;
        nsec_diff_2 += 1000000000 ; // take a carry (1 sec = 10^9 nsec) and add it to the nsec diff.
    }

    sec_diff_2 += t2.tv_sec - t1.tv_sec;

    printf("diff: sec %d, nsec %d\n", sec_diff, nsec_diff);
    printf("diff2: sec %d, nsec %d\n", sec_diff_2, nsec_diff_2);

    int64_t diff1_nsec = sec_diff  *1000000000 + nsec_diff  ;
    int64_t diff2_nsec = sec_diff_2*1000000000 + nsec_diff_2;

    float  iteration_len_nsec_1 = (float)diff1_nsec / 1000000 ;
    float  iteration_len_nsec_2 = (float)diff2_nsec / 1000000 ;

    printf("diff1 (nsec) %g, iteration len %g\n", (float)diff1_nsec, iteration_len_nsec_1);
    printf("diff2 (nsec) %g, iteration len %g\n", (float)diff2_nsec, iteration_len_nsec_2);

    return;
}  // function check_dac_performance(). //




uint32_t get_new_value(const char* title, const uint32_t old_value)
{
    printf("%s: old value %d (0x%x) : new value ===> ",title, old_value, old_value);

    uint32_t new_value;

    int scanf_rc = scanf("%i", & new_value);
    if (scanf_rc == EOF) {
        printf(" [ERROR] failed to scan for value [%s]\n", title);
    }
    if (scanf_rc != 1) {
        printf(" [ERROR] could not get value [%s], mismatch in type?\n", title);
    }

    printf(" %d ... ", new_value);

    return new_value;
} // function get_new_value(). //




uint32_t  change_value(const char* title, volatile uint32_t* value)
{
    //const int was_continuous = pid_stop_continuous();

    printf("About to change %s (old value %d)...\n", title, *value);

    *value = get_new_value(title,*value);

    //pid_restore_continuous(was_continuous);
    printf("changed %s to %d\n", title, *value);

    return *value;
} // function change_value() //



void set_predictor_delay()
{
    if (g_predictor_regs == NULL)
    {
        printf("ERROR: g_predictor_regs is null!\n");
        return;
    }

    printf("setting predictor delay (current delay = %d, current count %d, is delay enabled? %d):\n"
           "Please enter delay value in units of 10ns (0 for no delay) ==> ",
           READ_PIO_REG(o_delay_count),
           READ_PIO_REG(o_delay_counter),
           READ_PIO_REG(o_config) & (1<< 9));

    int  delay_count;

    int scanf_rc = scanf("%d",&delay_count);
    if (scanf_rc == EOF || scanf_rc != 1) {
        printf("[ERROR] failed to read delay from input\n");
        return;
    }


    printf("\n delay_count ==> %d\n", delay_count);

    if (delay_count <= 0)
    {
        printf("DISABLING DELAY!!\n");


        union predictor_hw_reg_o_config_u config;
        config._w32 = READ_PIO_REG(o_config);

        config.c.do_delay = 0;

        WRITE_PIO_REG(o_config, config._w32);
        WRITE_PIO_REG(o_delay_count, 0);
    }
    else
    {
        printf("ENABLING DELAY!!\n");

        WRITE_PIO_REG(o_delay_count, delay_count);
        union predictor_hw_reg_o_config_u config;
        config._w32 = READ_PIO_REG(o_config);

        config.c.do_delay = 1;

        WRITE_PIO_REG(o_config, config._w32);
    }

} // function set_predictor_delay(). //



// void get_and_set_alpha_old_method(predictor_hw_regs_t* predictor_hw_regs)
// {
//     if (predictor_hw_regs == NULL)
//     {
//         printf("ERROR: predctor_hw_regs is null!\n");
//         return;
//     }

//     double alpha = 0.0;
//     double g0    = 1.0;

//     printf("About to enter 2 parameters:\n\n"
//            "1) alpha = ratio DT/dt,  \n"
//            "   where DT = prediction interval and dt = sampling interval (20ns for now)\n"
//            "2) proportional gain     \n");

//     printf("Please enter alpha ==> ");
//     int scanf_rc = scanf("%lf", &alpha);
//     if (scanf_rc == EOF || scanf_rc != 1) {
//         printf("[ERROR] failed to read 'alpha' from input\n");
//     }

//     printf("=== alpha ==> %lf\n", alpha);

//     printf("Please enter proportional gain (g0) ==> ");
//     scanf_rc = scanf("%lf", &g0);
//     if (scanf_rc == EOF || scanf_rc != 1) {
//         printf("[ERROR] failed to read 'gain' from input\n");
//     }

//     printf("=== g0 ==> %lf\n", g0);


//     const double alpha2 = alpha*alpha;
//     const double alpha3 = alpha2*alpha;

//     const double p0_d = g0 * ( (1. + (15./8.)*alpha + alpha2 + alpha3/6.)      ) ;
//     const double p1_d = g0 * ( -( (25./8.)*alpha + (5./2.)*alpha2 + alpha3/2.) ) ;
//     const double p2_d = g0 * ( (13./8.)*alpha + 2*alpha2 + alpha3/2.           ) ;
//     const double p3_d = g0 * ( - 0.5*alpha*(0.75 + alpha + alpha2/3.)          ) ;

//     printf("from alpha %lf/g0 = %lf ==> float values: p0 %lf, p1 %lf, p2 %lf, p3 %lf\n", alpha, g0, p0_d, p1_d, p2_d, p3_d);

//     const int32_t p0_sign = ((p0_d >= 0.) ? 1 : -1);
//     const int32_t p1_sign = ((p1_d >= 0.) ? 1 : -1);
//     const int32_t p2_sign = ((p2_d >= 0.) ? 1 : -1);
//     const int32_t p3_sign = ((p3_d >= 0.) ? 1 : -1);

//     const int32_t p0 = p0_sign * (int32_t)(fabs(p0_d) + 0.5);
//     const int32_t p1 = p1_sign * (int32_t)(fabs(p1_d) + 0.5);
//     const int32_t p2 = p2_sign * (int32_t)(fabs(p2_d) + 0.5);
//     const int32_t p3 = p3_sign * (int32_t)(fabs(p3_d) + 0.5);

//     printf("from alpha %lf ==> int values: p0 %d, p1 %d, p2 %d, p3 %d\n", alpha, p0, p1, p2, p3);

// //    predictor_hw_regs->w.i_q0 = p0;
// //    predictor_hw_regs->w.i_q1 = p1;
// //    predictor_hw_regs->w.i_q2 = p2;
// //    predictor_hw_regs->w.i_q3 = p3;
// //
// //    printf("alpha set to %lf, g0 to %lf, regs: q0 %d, q1 %d, q2 %d, q3 %d\n",
// //               alpha,
// //               g0,
// //               predictor_hw_regs->w.i_q0,
// //               predictor_hw_regs->w.i_q1,
// //               predictor_hw_regs->w.i_q2,
// //               predictor_hw_regs->w.i_q3);

//     return;
// } // function get_and_set_alpha(). //





//
// Access a flat vector as a 2D array of row size n
// (C must have the row size a compile time constant, and therefore, not suitable
//  for runtime row size specification (the value n specifies the matrix size n x n))
//

#define __M_ELEMENT(M,n,i,j)            \
        ((double*)M)[(i)*n + (j)]

void set_alpha(const double alpha, const double g0, const int n)
{
#define max_n 6
    double  matrix_M_area[max_n][max_n]                 ;
    double* matrix_M_vec = (double*)&matrix_M_area      ;
    double vector_b[max_n] = {1.0, 0, 0, 0, 0, 0 };

    printf("set alpha = %f, overall gain g0 = %f, order = %d\n", alpha, g0, n);

    int32_t x_i32[max_n];

    int i;
    int j;
    for ( i = 0 ; i < max_n ; i++)
    {
        x_i32[i] = 0;
    }

#define M_ELEMENT(n,i,j)    __M_ELEMENT(matrix_M_vec,n,i,j)

    if (n > max_n)
    {
        printf("***ERROR: n=%d > max n=%d\n", n, max_n);
        return;
    }

    if (n < 2)
    {
        x_i32[0] = (int32_t)(g0 + 0.5);
        // printf("***ERROR: n too small %d\n", n);
        // return;
        printf("==== small n = %d ==== \n",n);

    } else {

        printf("==== n = %d ==== \n",n);

        for (i = 0 ; i < max_n ; i++)
        {
            for (j = 0 ; j < max_n ; j++)
            {
                matrix_M_area[i][j] = 0.;
            } // for j loop. //
        } // for i loop. //

        // 
        // M_{ij} = (-1)^{i} (alpha + j)^{i} / i!
        //
        // 1st row s just 1's as i=0.
        //
        for (i = 0 ; i < n ; i++)
        {
            M_ELEMENT(n,0,i) = 1.;
        } // for i loop. //

        //
        // 2nd row and onwards:
        double D    = 1;
        double sign = 1;

        for (i = 1 ; i < n ; i++)
        {
            D = D * i;  // denominator D = i!
            sign = -sign;
            for (j = 0 ; j < n ; j++)
            {
                int k;
                double x = alpha + (double)j;
                double xn= x;
                for (k = 1; k < i ; k++)
                {
                    xn = xn*x;
                }
                M_ELEMENT(n,i,j) = sign*xn/ D;
            } // for j loop. //
        } // for i loop. //

        printf("matrix M * b = \n");
        for (i = 0 ; i < n ; i++)
        {
            printf(" | ");

            for (j = 0 ; j < n ; j++ )
            {
                printf("  %10.10g", M_ELEMENT(n,i,j));
            } // for j loop //

            printf("  |  %10.10g  \n", vector_b[i]);
        } // for i loop. //

        gsl_matrix_view m = gsl_matrix_view_array(&matrix_M_area[0][0],n,n);
        gsl_vector_view v = gsl_vector_view_array(vector_b,n);
        gsl_vector*     x = gsl_vector_alloc(n);
        int             s;

        // printf("M as gsl sees it\n");
        // gsl_matrix_fprintf(stdout, &m.matrix, "%10.10g");
        // printf("b as gsl sees it\n");
        // gsl_vector_fprintf(stdout, &v.vector, "%10.10g");

        gsl_permutation* p = gsl_permutation_alloc(n);

        int rc = gsl_linalg_LU_decomp(&m.matrix, p, &s);
        printf("LU decomposition rc=%d, s=%d\n", rc, s);
        double det = gsl_linalg_LU_det(&m.matrix,s);
        printf("|matrix| = %g\n", det);
        rc = gsl_linalg_LU_solve(&m.matrix, p, &v.vector, x);
        printf("LU solve rc=%d\n", rc);

        printf("resultant x= %g %g %g %g\n", x->data[0], x->data[1], x->data[2], x->data[3]);
        //gsl_vector_fprintf(stdout, x, "%g");



        for (i = 0 ; i < n ; i++)
        {
            if (x->data[i] > 0)
            {
                x_i32[i] = (int32_t)(x->data[i]*g0 + 0.5);
            }
            else if(x->data[i] < 0)
            {
                x_i32[i] = (int32_t)(x->data[i]*g0 - 0.5);
            }
        }

        gsl_permutation_free(p);
        gsl_vector_free(x);
    }

    uint32_t q0_q4 = (x_i32[0] & 0xFFFF) | ((x_i32[4] & 0xFFFF) << 16) ;
    uint32_t q1_q5 = (x_i32[1] & 0xFFFF) | ((x_i32[5] & 0xFFFF) << 16) ;
    uint32_t q2_q6 = (x_i32[2] & 0xFFFF) | 0;
    uint32_t q3_q7 = (x_i32[3] & 0xFFFF) | 0;

    WRITE_PIO_REG(o_q0_q4, q0_q4);
    WRITE_PIO_REG(o_q1_q5, q1_q5);
    WRITE_PIO_REG(o_q2_q6, q2_q6);
    WRITE_PIO_REG(o_q3_q7, q3_q7);

    return;
} // function get_and_set_alpha(). //



void get_and_set_alpha()
{
    if (g_predictor_regs == NULL)
    {
        printf("ERROR: predctor_hw_regs is null!\n");
        return;
    }

    double alpha = 0.0;
    int    n     = 6  ;

    printf("About to enter 2 parameters:\n\n"
           "1) alpha = ratio DT/dt,  \n"
           "   where DT = prediction interval and dt = sampling interval (20ns for now)\n"
           "2) proportional gain     \n");

    printf("Please enter n (2..6) ==> ");
    int scanf_rc = scanf("%d", &n);
    if (scanf_rc == EOF || scanf_rc != 1) {
        printf("[ERROR] failed to read 'n' from input\n");
        return;
    }

    printf("Please enter alpha ==> ");
    scanf_rc = scanf("%lf", &alpha);
    if (scanf_rc == EOF || scanf_rc != 1) {
        printf("[ERROR] failed to read 'alpha' from input\n");
        return;
    }

    printf("=== alpha ==> %lf\n", alpha);

    set_alpha(alpha, 1.0, n);
    return;
} // function get_and_set_alpha(). //



void predictor_measure_performance(void)
{
    //
    // measuring h/w performance:
    // take the count at a given time, then at a later time,
    // and assess the # of samples in that period
    //
    if (g_predictor_regs == NULL)
    {
        printf("ERROR: g_predictor_regs is null!\n");
        return;
    } // if no H/W predictor registers pointer init then... //
    if ((READ_PIO_REG(o_config) & 0x2) == 0)
    {
        printf("**ERROR: predictor not in continuous operation, nothing to measure\n");
        return;
    } // if not in a good state then... //

    struct timespec t0, t1;
    const uint32_t c0 = READ_PIO_REG(o_count);
    clock_gettime(CLOCK_REALTIME, & t0);

    sleep(3);

    const uint32_t c1 = READ_PIO_REG(o_count);
    clock_gettime(CLOCK_REALTIME, & t1);

    print_time_diff("predictor h/w performance", &t0, &t1, c1 - c0);

} // function measure_performance(). //


void run_dacb_menu(void)
{
    if (g_predictor_regs == NULL)
    {
        printf("ERROR: g_predictor_regs is null!\n");
        return;
    } // if no H/W predictor registers pointer init then... //

    printf("=====DAC B debug output configuration======\n");

    int continue_loop = 1;

    union dacb_output_config_u dacb ;

    //    /* Bit [    0] */ unsigned int  pid_out              :   1   ;
    //    /* Bit [    1] */ unsigned int  adc_da               :   1   ;
    //    /* Bit [    2] */ unsigned int  predictor_y_n        :   1   ;
    //    /* Bit [    3] */ unsigned int  predictor_y_n_1      :   1   ;
    //    /* Bit [    4] */ unsigned int  predictor_d1_n_1     :   1   ;
    //    /* Bit [    5] */ unsigned int  predictor_d2_n_1     :   1   ;
    //    /* Bit [    6] */ unsigned int  predictor_d3_n_1     :   1   ;
    //    /* Bit [    7] */ unsigned int  predictor_z_n        :   1   ;
    //    /* Bit [    8] */ unsigned int  pid_count            :   1   ;
    //    /* Bit [    9] */ unsigned int  predictor_count      :   1   ;
    dacb._w32 = READ_PIO_REG(o_dacb_output);

    while (continue_loop)
    {

        printf( /* 0 */"1) DACB value 0x%8.8x: \n"
                /* 1 */" Bit [ 0] adc_da                        %d\n"
                /* 2 */" Bit [ 1] value 0x2000                  %d\n"
                /* 3 */" Bit [ 2] predictor_y_n                 %d\n"
                /* 4 */" Bit [ 3] predictor_y_n_1               %d\n"
                /* 5 */" Bit [ 4] predictor_y_n_2               %d\n"
                /* 6 */" Bit [ 5] predictor_y_n_3               %d\n"
                /* 7 */" Bit [ 6] predictor_y_n_4               %d\n"
                /* 8 */" Bit [ 7] predictor_z_n                 %d\n"
                /* 9 */" Bit [ 8] predictor_y_n_5               %d\n"
                /*10 */" Bit [ 9] predictor_count               %d\n"
                /*11 */" Bit [10] predictor_y_n_6               %d\n"
                /*12 */" Bit [11] predictor_y_n_7               %d\n"
                /*13 */" Bit [12] dither_input_counter          %d\n"
                /*14 */" Bit [13] predictor_z_n_no_integral     %d\n"
                /*15 */" Bit [14] predictor_y_input             %d\n"
                /*16 */" Bit [15] predictor_integral_sum        %d\n"
                /*17 */" Bit [16] 0                             %d\n"
                /*18 */" Bit [17] 0                             %d\n",
                /* 0 */ dacb._w32                            ,
                /* 1 */ dacb.c.adc_da                        ,
                /* 2 */ dacb.c.pid_output                    ,
                /* 3 */ dacb.c.predictor_y_n                 ,
                /* 4 */ dacb.c.predictor_y_n_1               ,
                /* 5 */ dacb.c.predictor_y_n_2               ,
                /* 6 */ dacb.c.predictor_y_n_3               ,
                /* 7 */ dacb.c.predictor_y_n_4               ,
                /* 8 */ dacb.c.predictor_z_n                 ,
                /* 9 */ dacb.c.predictor_y_n_5               ,
                /*10 */ dacb.c.predictor_count               ,
                /*11 */ dacb.c.predictor_y_n_6               ,
                /*12 */ dacb.c.predictor_y_n_7               ,
                /*13 */ dacb.c.dither_input_counter          ,
                /*14 */ dacb.c.predictor_z_n_no_integral     ,
                /*15 */ dacb.c.predictor_y_input             ,
                /*16 */ dacb.c.predictor_integral_sum        ,
                /*17 */ dacb.c.zero_16                       ,
                /*18 */ dacb.c.zero_17
              );

        printf("Select:\n"
                "<0> adc_da                        \n"
                "<1> value 0x2000                  \n"
                "<2> predictor_y_n                 \n"
                "<3> predictor_y_n_1               \n"
                "<4> predictor_y_n_2               \n"
                "<5> predictor_y_n_3               \n"
                "<6> predictor_y_n_4               \n"
                "<7> predictor_z_n                 \n"
                "<8> predictor_y_n_5               \n"
                "<9> predictor_count               \n"
                "<a> predictor_y_n_6               \n"
                "<b> predictor_y_n_7               \n"
                "<c> dither_input_counter          \n"
                "<d> predictor_z_n_no_integral     \n"
                "<e> predictor_y_input             \n"
                "<f> predictor_integral_sum        \n"
                "<g> zero[16]                      \n"
                "<h> zero[17]                      \n"
                "<z> clear                         \n"
                "\n"
                "<y> Return to previous menu\n"
                );


        char b;
        int scanf_rc = scanf("%c", &b );
        if (scanf_rc == EOF || scanf_rc != 1) {
            printf("[ERROR] failed to read selection from input\n");
            return;
        }

        printf("%c ... (%d)\n", b, b);


        switch (b)
        {
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
            {
                printf("selected %c\n", b);

                const uint32_t bit = 1 << (uint32_t)(b - '0');

                dacb._w32 = bit;
                WRITE_PIO_REG(o_dacb_output,dacb._w32);
            }
            break;

            case 'a':
            case 'b':
            case 'c':
            case 'd':
            case 'e':
            case 'f':
            case 'g':
            case 'h':
            {
                const uint32_t bit_index = (int)(b - 'a') + 10;
                printf("selected %c ==> bit # %d \n", b, bit_index);

                const uint32_t bit = 1 << bit_index;

                dacb._w32 = bit;
                WRITE_PIO_REG(o_dacb_output,dacb._w32);
            }
            break;

            case 'z':
            {
                printf("clearing\n");

                dacb._w32 = 0;
                WRITE_PIO_REG(o_dacb_output,dacb._w32);
            }
            break;

            case 'y':
            {
                printf("returning to previous menu\n");

                continue_loop = 0;
            }
            break;

            default:
                break;
        } // switch //
    } // while loop. //

    printf("leaving dacb menu....\n\n"); fflush(stdout);
} // function run_dacb_menu(). //



void print_dither_registers()
{
    const uint32_t dither_config_1 = READ_PIO_REG(o_dither_config_1);
    const uint32_t dither_config_2 = READ_PIO_REG(o_dither_config_2);
    const uint32_t dither_config_3 = READ_PIO_REG(o_dither_config_3);
    const uint32_t dither_count_1  = READ_PIO_REG(o_dither_count_1 );
    const uint32_t dither_count_2  = READ_PIO_REG(o_dither_count_2 );
    const uint32_t dither_count_3  = READ_PIO_REG(o_dither_count_3 );

    union dither_config_1 dc_1 = {._w32 = dither_config_1 };
    union dither_config_2 dc_2 = {._w32 = dither_config_2 };
    union dither_config_3 dc_3 = {._w32 = dither_config_3 };

    union dither_output_1 do_1 = {._w32 = dither_count_1 };
    union dither_output_2 do_2 = {._w32 = dither_count_2 };

    printf("current dither configuration\n");
    printf("(quick dump: cfg1 0x%8.8x, cfg2 0x%8.8x, cfg3 0x%8.8x, cnt1 0x%8.8x, cnt2 0x%8.8x, cnt3 0x%8.8x)\n",
           dither_config_1 ,
           dither_config_2 ,
           dither_config_3 ,
           dither_count_1  ,
           dither_count_2  ,
           dither_count_3  );
    printf("dither 1: output amplitude %d, input init count %d \n", dc_1.r.output_amplitude , dc_1.r.input_init_count   );
    printf("dither 2: input next count %d, output init count %d\n", dc_2.r.input_next_count , dc_2.r.output_init_count  );
    printf("dither 3: dither input enable? %d, dither output enable? %d, input init polarity %d\n"
                                                                                            , dc_3.r.dither_input_enable, dc_3.r.dither_output_enable    , dc_3.r.input_init_polarity);
    printf("dither output 1: input count? %d, output count %d  \n", do_1.r.input_count      , do_1.r.output_count       );
    printf("dither output 2: input polarity %d, ouptut polarity %d, input state %d\n"
                                                                  , do_2.r.input_polarity   , do_2.r.output_polarity    , do_2.r.input_state);
    printf("dither output 3: out_offset_plus_dither_amplitude %d\n",dither_count_3 );

} // function print_dither_registers(). //



void dither_config_menu()
{
    int quit = 0;

    while (!quit)
    {
        printf("Dither configuration menu:\n");

        print_dither_registers();

        printf( "\n\n Please select:\n"
                "<o> output amplitude \n"
                "<c> output init count\n"
                "<i> input init count\n"
                "<n> input next count\n"
                "<d> dither output enable\n"
                "<D> dither input enable\n"
                "<p> dither init polarity\n"
                "<q> quit\n"
               );

        char b;
        printf("selection ==> ");
        int scanf_rc = scanf("%c", &b );
        if (scanf_rc == EOF || scanf_rc != 1) {
            printf("[ERROR] failed to read selection from input\n");
            return;
        }

        printf("%c",b );

        const uint32_t dither_config_1 = READ_PIO_REG(o_dither_config_1);
        const uint32_t dither_config_2 = READ_PIO_REG(o_dither_config_2);
        const uint32_t dither_config_3 = READ_PIO_REG(o_dither_config_3);

        union dither_config_1 dc_1 = {._w32 = dither_config_1 };
        union dither_config_2 dc_2 = {._w32 = dither_config_2 };
        union dither_config_3 dc_3 = {._w32 = dither_config_3 };


        switch (b)
        {
            case 'p':
            {
                uint32_t input_init_polarity = dc_3.r.input_init_polarity;

                change_value("dither input init polarity enable", & input_init_polarity);

                if(input_init_polarity == 0)
                {
                    printf("\n INPUT INIT POLARITY LOW... \n");
                    dc_3.w.input_init_polarity = 0;
                    WRITE_PIO_REG(o_dither_config_3,dc_3._w32);
                }
                else
                {
                    printf("\n INPUT INIT POLARITY HIGH... \n");
                    dc_3.w.input_init_polarity = 1;
                    WRITE_PIO_REG(o_dither_config_3,dc_3._w32);
                }
            }
            break;

            case 'd':
            {
                uint32_t dither_output_enable = dc_3.r.dither_output_enable;

                change_value("dither output enable", & dither_output_enable);

                if(dither_output_enable == 0)
                {
                    printf("\n DITHER OUTPUT DISABLED... \n");
                    dc_3.w.dither_output_enable = 0;
                    WRITE_PIO_REG(o_dither_config_3,dc_3._w32);
                }
                else
                {
                    printf("\n DITHER OUTPUT ENABLED... \n");
                    dc_3.w.dither_output_enable = 1;
                    WRITE_PIO_REG(o_dither_config_3,dc_3._w32);
                }
            }
            break;

            case 'D':
            {
                uint32_t dither_input_enable = dc_3.r.dither_input_enable;

                change_value("dither input enable", & dither_input_enable);

                if(dither_input_enable == 0)
                {
                    printf("\n DITHER INPUT DISABLED... \n");
                    dc_3.w.dither_input_enable = 0;
                    WRITE_PIO_REG(o_dither_config_3,dc_3._w32);
                }
                else
                {
                    printf("\n DITHER INPUT ENABLED... \n");
                    dc_3.w.dither_input_enable = 1;
                    WRITE_PIO_REG(o_dither_config_3,dc_3._w32);
                }
            }
            break;

            case 'n':
            {
                uint32_t input_next_count = dc_2.r.input_next_count;

                change_value("dither input next count", & input_next_count);

                dc_2.w.input_next_count = input_next_count;

                WRITE_PIO_REG(o_dither_config_2,dc_2._w32);
            }
            break;

            case 'c':
            {
                uint32_t output_init_count = dc_2.r.output_init_count;

                change_value("dither output init count", & output_init_count);

                dc_2.w.output_init_count = output_init_count;

                WRITE_PIO_REG(o_dither_config_2,dc_2._w32);
            }
            break;

            case 'o':
            {
                uint32_t output_amplitude = dc_1.r.output_amplitude;

                change_value("dither output amplitude", & output_amplitude);


                dc_1.w.output_amplitude = output_amplitude;

                printf("chaning dither output amplitude to --> %d (reg value 0x%8.8x)\n", output_amplitude, dc_1._w32);
                WRITE_PIO_REG(o_dither_config_1,dc_1._w32);
            }
            break;

            case 'i':
            {
                uint32_t input_init_count = dc_1.r.input_init_count;

                change_value("dither input init count", & input_init_count);

                dc_1.w.input_init_count = input_init_count ;

                WRITE_PIO_REG(o_dither_config_1,dc_1._w32);
            }
            break;

            case 'q':
            {
                printf("\n QUITTING dither menu\n");
                quit = 1;
            }
            break;

            default:
                printf("\n selection %c (%d) unsupported\n", b,(int)b);
        } // switch ..//

    } // while loop. //

} // function dither_config_menu(). //






void run_predictor_menu(void)
{
    if (g_predictor_regs == NULL)
    {
        printf("ERROR: g_predictor_regs is null!\n");
        return;
    } // if no H/W predictor registers pointer init then... //

    printf("=====Predictor H/W registers configuration menu======\n");

    printf("NOW ABOUT TO READ FROM PREDICTOR HW...\n");

    int continue_loop = 1;

    while (continue_loop)
    {
        const uint32_t y_n    = READ_PIO_REG(o_y_n)  ;
        const uint32_t y_n_1  = 0xFFFF; // y_n_1 and y_n_2 are currently not given by H/W
        const uint32_t y_n_2  = 0xFFFF;
        const uint32_t y_n_3  = READ_PIO_REG(o_y_n_3);
        const uint32_t y_n_4  = READ_PIO_REG(o_y_n_4);
        const uint32_t y_n_5  = READ_PIO_REG(o_y_n_5);
        const uint32_t y_n_6  = READ_PIO_REG(o_y_n_6);
        const uint32_t y_n_7  = READ_PIO_REG(o_y_n_7);

        const uint32_t z_n = READ_PIO_REG(o_z_n);
        const uint32_t z_n_14b = z_n & 0x3FFF;

        printf("1) READING: z_n %d (14bit: %d, 0x%x), y_n %d, y_n_1 %d, y_n_2 %d, y_n_3 %d, y_n_4 %d, y_n_5 %d, y_n_6 %d, y_n_7 %d \n",
                           (int  )(z_n    ),
                           (int  )(z_n_14b),
                           (int  )(z_n_14b),
                           (int  )(y_n    ),
                           (int  )(y_n_1  ),
                           (int  )(y_n_2  ),
                           (int  )(y_n_3  ),
                           (int  )(y_n_4  ),
                           (int  )(y_n_5  ),
                           (int  )(y_n_6  ),
                           (int  )(y_n_7  )
                        );

        const uint32_t q0_q4 = READ_PIO_REG(o_q0_q4);
        const uint32_t q1_q5 = READ_PIO_REG(o_q1_q5);
        const uint32_t q2_q6 = READ_PIO_REG(o_q2_q6);
        const uint32_t q3_q7 = READ_PIO_REG(o_q3_q7);
        const uint32_t q0    = q0_q4 & 0xFFFF;
        const uint32_t q1    = q1_q5 & 0xFFFF;
        const uint32_t q2    = q2_q6 & 0xFFFF;
        const uint32_t q3    = q3_q7 & 0xFFFF;
        const uint32_t q4    = q0_q4 >> 16   ;
        const uint32_t q5    = q1_q5 >> 16   ;
        const uint32_t q6    = q2_q6 >> 16   ;
        const uint32_t q7    = q3_q7 >> 16   ;

        const uint32_t debug1 = READ_PIO_REG(o_debug_reg_1);

        printf("2) READING: magix 0x%8.8x, config 0x%8.8x, count %d, q0 %d, q1 %d, q2 %d, q3 %d, q4 %d, q5 %d, q6 %d, q7 %d\n",
                           (int  )(READ_PIO_REG(o_magic )),
                           (int  )(READ_PIO_REG(o_config)),
                           (int  )(READ_PIO_REG(o_count )),
                           (int  )(q0                    ),
                           (int  )(q1                    ),
                           (int  )(q2                    ),
                           (int  )(q3                    ),
                           (int  )(q4                    ),
                           (int  )(q5                    ),
                           (int  )(q6                    ),
                           (int  )(q7                    )
                        );

        union predictor_hw_reg_i_config_u config_u;
        config_u._w32 = READ_PIO_REG(o_config);
        printf("2.1) CONFIG DUMP: 0x%x, \n"
               "   [  0  ]  sw_reset:                   %d\n"
               "   [  1  ]  continuous:                 %d\n"
               "   [  2  ]  input select:               %d\n"
               "   [  3  ]  pre-dither manual enable:   %d\n"
               "   [ 8: 4]  output precision size:      %d\n"
               "   [  9  ]  adc align enable:           %d\n"
               "   [ 10  ]  do delay:                   %d\n"
               "   [ 11  ]  manual dac output enable:   %d\n"
               "   [ 12  ]  invert y_n:                 %d\n"
               "   [ 13  ]  invert output:              %d\n"
               "   [ 14  ]  output precision enable:    %d\n"
               "   [ 15  ]  input averaging enable:     %d\n"
               "   [21:16]  output shift:               %d\n"
               "   [ 22  ]  kick predictor:             %d\n"
               "   [ 23  ]  reserved_23:                %d\n"
               "   [28:24]  output precision shift:     %d\n"
               "   [ 29  ]  y input diff enable:        %d\n"
               "   [ 30  ]  2nd integrator output enable:%d\n"
               "   [ 31  ]  2nd integrator enable:      %d\n",
               config_u._w32,
               config_u.c.sw_reset,
               config_u.c.continuous,
               config_u.c.input_select,
               config_u.c.pre_dither_manual_enable,
               config_u.c.output_precision_size,
               config_u.c.adc_align_enable,
               config_u.c.do_delay,
               config_u.c.manual_dac_output_enable,
               config_u.c.invert_y_n,
               config_u.c.invert_output,
               config_u.c.output_precision_enable,
               config_u.c.input_averaging_enable,
               config_u.c.output_shift,
               config_u.c.kick_predictor,
               config_u.c.reserved_23,
               config_u.c.output_precision_shift,
               config_u.c.y_input_diff_enable,
               config_u.c.second_integrator_output_enable,
               config_u.c.second_integrator_enable);

        printf("3) READING: out_offset 0x%8.8x  y_reference %d, i0 %d\n",
                       (int  )(READ_PIO_REG(o_out_offset )),
                       (int  )(READ_PIO_REG(o_y_reference)),
                       (int  )(READ_PIO_REG(o_i0         )));

        printf("4) READING: delay_count %d,  delay_counter %d, is delay enabled? %d\n",
                       (int  )(READ_PIO_REG(o_delay_count    )) ,
                       (int  )(READ_PIO_REG(o_delay_counter  )) ,
                       (int  )(READ_PIO_REG(o_config)&  (1<<9)) );

        printf("5) predictor register dump: \n"
                        "y_n          : 0x%lx\n"
                        "y_n_1        : 0x%lx\n"
                        "y_n_2        : 0x%lx\n"
                        "y_n_3        : 0x%lx\n"
                        "y_n_4        : 0x%lx\n"
                        "y_n_5        : 0x%lx\n"
                        "y_n_6        : 0x%lx\n"
                        "y_n_7        : 0x%lx\n"
                        "q0_q4        : 0x%lx\n"
                        "q1_q5        : 0x%lx\n"
                        "q2_q6        : 0x%lx\n"
                        "q3_q7        : 0x%lx\n"
                        "config       : 0x%lx\n"
                        "y_reference  : 0x%lx\n"
                        "i0           : 0x%lx\n"
                        "z_n          : 0x%lx\n"
                        "count        : 0x%lx\n"
                        "delay_count  : 0x%lx\n"
                        "delay_counter: 0x%lx\n"
                        "out_offset   : 0x%lx\n"
                        "magic        : 0x%lx\n"
                        "dacb_output  : 0x%lx\n"
            ,
            (long)(READ_PIO_REG(o_y_n)           ),
            0xFFFFL /* (long)(predictor2_hw_regs->r.o_y_n_1         ) */,  // for now y_n_1 and y_n_2 are not provided by H/W
            0xFFFFL /* (long)(predictor2_hw_regs->r.o_y_n_2         ) */,
            (long)(READ_PIO_REG(o_y_n_3         )),
            (long)(READ_PIO_REG(o_y_n_4         )),
            (long)(READ_PIO_REG(o_y_n_5         )),
            (long)(READ_PIO_REG(o_y_n_6         )),
            (long)(READ_PIO_REG(o_y_n_7         )),
            (long)(READ_PIO_REG(o_q0_q4         )),
            (long)(READ_PIO_REG(o_q1_q5         )),
            (long)(READ_PIO_REG(o_q2_q6         )),
            (long)(READ_PIO_REG(o_q3_q7         )),
            (long)(READ_PIO_REG(o_config        )),
            (long)(READ_PIO_REG(o_y_reference   )),
            (long)(READ_PIO_REG(o_i0            )),
            (long)(READ_PIO_REG(o_z_n           )),
            (long)(READ_PIO_REG(o_count         )),
            (long)(READ_PIO_REG(o_delay_count   )),
            (long)(READ_PIO_REG(o_delay_counter )),
            (long)(READ_PIO_REG(o_out_offset    )),
            (long)(READ_PIO_REG(o_magic         )),
            (long)(READ_PIO_REG(o_dacb_output   )));

        //
        // printing predictor config register
        //
        {
            union predictor_hw_reg_i_config_u config;

            config._w32 = READ_PIO_REG(o_config);

            printf("6) CONFIG: 0x%8.8x ......output shift: %d, hpo shift %d, hpo size %d, \n%s, \n%s, \n%s, \n%s, \n%s, \n%s, \n%s, \n%s, \n%s, \n%s \n%s \n................. |\n" ,
                   config._w32,
                   config.c.output_shift,
                   config.c.output_precision_shift,
                   config.c.output_precision_size,
                   (config.c.input_averaging_enable ? "input averaging enabled" : "no input averaging"      ),
                   (config.c.output_precision_enable? "output precision enabled": "no output precision"     ),
                   (config.c.continuous             ? "continuous"              : "sw step"                 ),
                   (config.c.input_select           ? "input A"                 : "input B"                 ),
                   (config.c.adc_align_enable       ? "aligned to ADC clock"    : "unaligned to ADC clock"  ),
                   (config.c.pre_dither_manual_enable
                                                    ? "normal output to dither" : "pre-dither manual mode"  ),
                   (config.c.do_delay               ? "do delay"                : "no delay"                ),
                   (config.c.manual_dac_output_enable
                                                    ? "manual-dac-output"       : "standard dac-output"     ),
                   (config.c.invert_y_n             ? "y_n-inverted"            : "no-y-n-inversion"        ),
                   (config.c.invert_output          ? "output-inverted"         : "no-output-inversion"     ),
                   (config.c.kick_predictor         ? "kick-predictor"          : "no-kick"                 )
                   );


        }

        printf("7) debug1 register: 0x%8.8x, 2nd integrator enabled? %d, 2nd integrator output enabled? %d, magic: 0x%4.4x\n",
                debug1,
                debug1 & 0x01,
                debug1 & 0x02,
                debug1 >> 16
                );

        print_dither_registers();

        //printf("4) S/W predictor state:\n");
        //predictor_print(&g_predictor_sw);

        printf("Select:\n"
                        "<c> toggle continuous operation\n"
                        "<0> change q0\n"
                        "<1> change q1\n"
                        "<2> change q2\n"
                        "<3> change q3\n"
                        "<4> change q4\n"
                        "<5> change q5\n"
                        "<6> change q6\n"
                        "<7> change q7\n"
                        "<i> change i0\n"
                        "<a> change alpha (and proportional gain g0)\n"
                        "<s> change output shift\n"
                        "<n> toggle input select\n"
                        "<r> set y_reference\n"
                        "<A> toggle pre-dither manual mode\n"
                        "<R> issue RESET\n"
                        "<m> measure predictor performance\n"
                        "<t> set output offset\n"
                        "<D> Set DACB output\n"
                        "<P> toggle high output precision enable/disable\n"
                        "<I> toggle input averaging enable/disable\n"
                        "<O> set output precision shift\n"
                        "<Q> set output precision size\n"
                        "<d> set delay and enable/disable\n"
                        "<u> toggle manual dac outut enable\n"
                        "<w> toggle y_n    inversion\n"
                        "<z> toggle output inversion\n"
                        "<x> dither menu\n"
                        "<T> scan trace dump\n"
                        "\n"
                        "<q> Return to main menu\n"
            );

        char b;
        int scanf_rc = scanf("%c", &b );
        if (scanf_rc == EOF || scanf_rc != 1) {
            printf("[ERROR] failed to read selection from input - retrying in 120 seconds\n");
            sleep(120);
            continue;
        }

        printf("%c ... (%d)\n", b, b);

        union predictor_hw_reg_i_config_u  config_reg;

        config_reg._w32 = READ_PIO_REG(o_config);

        switch (b)
        {
            case 'c':
            {
                printf("Toggling continuous operation %d => %d\n", config_reg.c.continuous , !config_reg.c.continuous);

                config_reg.c.continuous = !config_reg.c.continuous ;

                WRITE_PIO_REG(o_config, config_reg._w32);
            }
            break;

            case 'a':
            {
                printf("set alpha (ratio of predicted time step to sample time step of 20ns)\n");
                get_and_set_alpha();
            }
            break;

            case 'n':
            {
                printf("Toggling input select %d => %d\n", config_reg.c.input_select, !config_reg.c.input_select);

                config_reg.c.input_select = !config_reg.c.input_select;
                WRITE_PIO_REG(o_config, config_reg._w32);
            }
            break;

            case 'r':
            {
                change_value("predictor y_reference", GET_PIO_REG_ADDR(o_y_reference));
            }
            break;

            case 'i':
            {
                change_value("predictor i0", GET_PIO_REG_ADDR(o_i0));
            }
            break;

            case '0':
            {
                uint32_t q0_q4 = READ_PIO_REG(o_q0_q4);
                uint32_t q0 = q0_q4 & 0xFFFF;
                change_value("predictor q0", &q0);
                q0_q4 = ((q0_q4 & 0xFFFF0000) + (q0 & 0xFFFF));
                WRITE_PIO_REG(o_q0_q4, q0_q4);
            }
            break;

            case '1':
            {
                uint32_t q1_q5 = READ_PIO_REG(o_q1_q5);
                uint32_t q1 = q1_q5 & 0xFFFF;
                change_value("predictor q1", &q1);
                q1_q5 = ((q1_q5 & 0xFFFF0000) + (q1 & 0xFFFF));
                WRITE_PIO_REG(o_q1_q5, q1_q5);
            }
            break;

            case '2':
            {
                uint32_t q2_q6 = READ_PIO_REG(o_q2_q6);
                uint32_t q2 = q2_q6 & 0xFFFF;
                change_value("predictor q2", &q2);
                q2_q6 = ((q2_q6 & 0xFFFF0000) + (q2 & 0xFFFF));
                WRITE_PIO_REG(o_q2_q6, q2_q6);
            }
            break;

            case '3':
            {
                uint32_t q3_q7 = READ_PIO_REG(o_q3_q7);
                uint32_t q3 = q3_q7 & 0xFFFF;
                change_value("predictor q3", &q3);
                q3_q7 = ((q3_q7 & 0xFFFF0000) + (q3 & 0xFFFF));
                WRITE_PIO_REG(o_q3_q7, q3_q7);
            }
            break;

            case '4':
            {
                uint32_t q0_q4 = READ_PIO_REG(o_q0_q4);
                uint32_t q4 = q0_q4 >> 16;
                change_value("predictor q4", &q4);
                q0_q4 = ((q0_q4 & 0x0000FFFF) + (q4 << 16));
                WRITE_PIO_REG(o_q0_q4, q0_q4);
            }
            break;

            case '5':
            {
                uint32_t q1_q5 = READ_PIO_REG(o_q1_q5);
                uint32_t q5 = q1_q5 >> 16;
                change_value("predictor q5", &q5);
                q1_q5 = ((q1_q5 & 0x0000FFFF) + (q5 << 16));
                WRITE_PIO_REG(o_q1_q5, q1_q5);
            }
            break;

            case '6':
            {
                uint32_t q2_q6 = READ_PIO_REG(o_q2_q6);
                uint32_t q6 = q2_q6 >> 16;
                change_value("predictor q4", &q6);
                q2_q6 = ((q2_q6 & 0x0000FFFF) + (q6 << 16));
                WRITE_PIO_REG(o_q2_q6, q2_q6);
            }
            break;

            case '7':
            {
                uint32_t q3_q7 = READ_PIO_REG(o_q3_q7);
                uint32_t q7 = q3_q7 >> 16;
                change_value("predictor q4", &q7);
                q3_q7 = ((q3_q7 & 0x0000FFFF) + (q7 << 16));
                WRITE_PIO_REG(o_q3_q7, q3_q7);
            }
            break;

            case 't':
            {
                change_value("output offset", GET_PIO_REG_ADDR(o_out_offset));
            }
            break;

            case 'm':
            {
                printf("measuring predictor performance...\n");
                predictor_measure_performance();
                printf("measuring predictor performance complete.\n");
            }
            break;

            case 'd':
            {
                set_predictor_delay();
            }
            break;

            case 'D':
            {
                printf("Setting DACB output (debug)\n");
                run_dacb_menu();
            }
            break;

            case 'x':
            {
                printf("Dither menu selected\n");
                dither_config_menu();
            }
            break;

            case 'A':
            {
                config_reg.c.pre_dither_manual_enable = !config_reg.c.pre_dither_manual_enable;

                WRITE_PIO_REG(o_config, config_reg._w32);

                printf("toggling pre-dither manual mode to ==> %s\n",
                       config_reg.c.pre_dither_manual_enable ? "normal mode" : "pre-dither manual mode");
            }
            break;

            case 'E':
            {
                config_reg.c.adc_align_enable = !config_reg.c.adc_align_enable;

                WRITE_PIO_REG(o_config, config_reg._w32);

                printf("toggling ADC alignment enabled to ==> %s\n",
                       config_reg.c.adc_align_enable ? "ADC clock alignment enabled" : "ADC clock alignment disabled");
            }
            break;

            case 'P':
            {
                config_reg.c.output_precision_enable = !config_reg.c.output_precision_enable;

                WRITE_PIO_REG(o_config, config_reg._w32);

                printf("toggling output precision enable ==> %s\n",
                       config_reg.c.output_precision_enable ? "ADC clock alignment enabled" : "ADC clock alignment disabled");
            }
            break;

            case 'I':
            {
                config_reg.c.input_averaging_enable = !config_reg.c.input_averaging_enable ;

                WRITE_PIO_REG(o_config, config_reg._w32);

                printf("toggling input averaging enable ==> %s\n",
                       config_reg.c.input_averaging_enable  ? "input averaging enabled" : "input averaging disabled");
            }
            break;

            case 'O':
            {
                uint32_t output_precision_shift = config_reg.c.output_precision_shift;

                change_value("output precision shift", & output_precision_shift);
                config_reg.c.output_precision_shift = output_precision_shift;

                WRITE_PIO_REG(o_config, config_reg._w32);

                printf("setting output precision shift ==> %d\n",
                       config_reg.c.output_precision_shift);
            }
            break;

            case 'Q':
            {
                uint32_t output_precision_size = config_reg.c.output_precision_size;

                change_value("output precision size", & output_precision_size);
                config_reg.c.output_precision_size= output_precision_size;

                WRITE_PIO_REG(o_config, config_reg._w32);

                printf("setting output precision size==> %d\n",
                       config_reg.c.output_precision_size);
            }
            break;


            case 's':
            {
                uint32_t shift_value = config_reg.c.output_shift;

                change_value("shift", & shift_value);

                config_reg.c.output_shift = shift_value;

                WRITE_PIO_REG(o_config, config_reg._w32);

            }
            break;

            case 'R':
            {
                printf("Resetting predictor\n");
                uint32_t config = READ_PIO_REG(o_config);

                config |= 1;

                WRITE_PIO_REG(o_config, config_reg._w32);

            }
            break;

            case 'S':
            {
                printf("Writing S/W value for y_n\n");
                uint32_t sw_value;

                change_value("sw value of y_n", & sw_value);

                WRITE_PIO_REG(o_y_n /* same place as i_sw_yn) */, sw_value);

            }
            break;

            case 'u':
            {
                union predictor_hw_reg_i_config_u config;

                config._w32 = READ_PIO_REG(o_config);

                printf("toggling manual dac outut from %d\n", config.c.manual_dac_output_enable);

                config.c.manual_dac_output_enable = !config.c.manual_dac_output_enable;

                WRITE_PIO_REG(o_config, config._w32);
            }
            break;

            case 'w':
            {
                union predictor_hw_reg_i_config_u config;

                config._w32 = READ_PIO_REG(o_config);

                printf("toggling y_n inversion from %d\n", config.c.invert_y_n);

                config.c.invert_y_n = !config.c.invert_y_n;

                WRITE_PIO_REG(o_config, config._w32);
            }
            break;

            case 'z':
            {
                union predictor_hw_reg_i_config_u config;

                config._w32 = READ_PIO_REG(o_config);

                printf("toggling output inversion from %d\n", config.c.invert_output);

                config.c.invert_output = !config.c.invert_output;

                WRITE_PIO_REG(o_config, config._w32);
            }
            break;


            case 'q':
            {
                printf("returning to previous menu\n");

                continue_loop = 0;
            }
            break;

            case 'T':
            {
                printf("dumping scan trace (if compiled)\n");
                extern void scan_trace_dump(void);

                scan_trace_dump();
            }

            default:
                break;
        } // switch //
    } // while loop. //

    printf("leaving predictor menu....\n\n"); fflush(stdout);

    struct timespec ts;
    ts.tv_nsec = 200*1000*1000;
    ts.tv_sec  = 0;
    nanosleep(&ts, NULL);
} // function run_predictor_menu(). //






void pid_hw_run(void)
{
    if (g_predictor_regs == NULL)
    {
        printf("ERROR: predictor global pointer not set?\n");
        return;
    }

    printf("Setting default alpha = 0\n");
    set_alpha(/* alpha = */ 0.0, /* g0 (unused) */ 1.0, /* n = */ 6);

    union dither_config_3 dither_config_3 = {0};


    union predictor_hw_reg_o_config_u predictor_init_config;

    predictor_init_config._w32                     = 0;
    predictor_init_config.c.continuous             = 0; // default = manual mode
    predictor_init_config.c.adc_align_enable       = 0;
    predictor_init_config.c.do_delay               = 0;
    predictor_init_config.c.input_select           = 1; // "1" = A, "0" = B
    predictor_init_config.c.input_averaging_enable = 0;
    predictor_init_config.c.manual_dac_output_enable
                                                   = 0;
    predictor_init_config.c.invert_output          = 0;
    predictor_init_config.c.invert_y_n             = 0;
    predictor_init_config.c.output_precision_enable= 0;
    predictor_init_config.c.output_shift           = 0;

    WRITE_PIO_REG(o_config, predictor_init_config._w32);
    WRITE_PIO_REG(o_dither_config_3, dither_config_3._w32);

    printf("=== predictor config 0x%8.8x, dither config 0x%8.8x ==== \n", READ_PIO_REG(o_config), READ_PIO_REG(o_dither_config_3));


    run_predictor_menu();

    printf("\n leaving main menu \n\n\n"); fflush(stdout);
} // function pid_hw_run(). //


char check_buffer[] = "this is a check buffer....";

uint32_t dbg_reg(const int num)
{
    const volatile pio_base_t* p = (const volatile pio_base_t*)g_predictor_regs;

    return p[num].data;
}

int main()
{
    int fd;


    printf("main: %s:%s:%s PARALLEL PID starting \n",__FILE__, __DATE__, __TIME__); fflush(stdout);
    printf("main: build_time %s\n", BUILD_TIME);

    hex_dump("check buffer dump", check_buffer, sizeof(check_buffer));

    printf("main: initialized S/W predictor, moving on...\n"); fflush(stdout);

    if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 )
    {
        printf( "ERROR: could not open \"/dev/mem\"...\n" );
        return( 1 );
    }

    virtual_base = mmap                                    // Map /dev/mem to our own process space virtual address
                    (
                        NULL                        ,    // Hint: proposed virtual address - we couldn't give a shit...
                        HW_REGS_SPAN                ,    // Length
                        ( PROT_READ | PROT_WRITE )  ,   // Protection flags: can read/write
                        MAP_SHARED                  ,   // Writes immediately visible to all others
                        fd                          ,   // File-descriptor to map (that of /dev/mem)
                        HW_REGS_BASE                    // Offset within the file.
                    );

    if( virtual_base == MAP_FAILED )
    {
        printf( "ERROR: mmap() failed...\n" );
        close( fd );
        return( 1 );
    }

    g_predictor_regs = (typeof(g_predictor_regs))virtual_base;
    printf("main: predictor regs base at %p\n", g_predictor_regs);

    const uint32_t magic = READ_PIO_REG(magic);
    if (magic  != PID_MAGIC) {
        printf("main: ***ERROR: MAGIC 0x%8.8x not as expected 0x%8.8x! aborting\n", magic, PID_MAGIC);
        exit(-10);
    }
    printf("main: [[MAGIC is good at %p]]\n", (void *)magic);

    printf("main: \n\n\n\n\n\n\n\n ============YOSSI VERSION STARTING 1.2 ======== \n\n\n\n\n\n\n\n\n ");
    printf("main: starting tcp server...\n");

    extern void server_main(void);

    server_main();

    // pid_run();
    pid_hw_run();

    printf("main: ending\n");

    if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
        printf( "ERROR: munmap() failed...\n" );
        close( fd );
        return( 1 );
    }

    close( fd );


    return( 0 );
}
