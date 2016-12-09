/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 *  
 *  
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 	* Redistributions of source code must retain the above copyright 
 * 	  notice, this list of conditions and the following disclaimer.
 * 
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	  notice, this list of conditions and the following disclaimer in the 
 * 	  documentation and/or other materials provided with the   
 * 	  distribution.
 * 
 * 	* Neither the name of Texas Instruments Incorporated nor the names of
 * 	  its contributors may be used to endorse or promote products derived
 * 	  from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <am335x/pru_cfg.h>
#include "resource_table_empty.h"

volatile register uint32_t __R30;

/* Shared PID data structure - ensure both shared stucts match PRU 1 */
struct pid_data {
    /* PID tunings */
    int Kp_f, Ki_f, Kd_f;

    /* PID controls */
    int setpoint;
    int int_err;
    int input, output, last_output;
    int min_output, max_output;
};

/* Shared memory block struct */
struct shared_mem {
    volatile char init_flag;
    volatile unsigned int pwm_out;
    volatile int enc_rpm;
    volatile struct pid_data pid;
};

/* 'Float' conversion */
#define SHIFT    0x0E

/* Function prototypes */
void update_pid(volatile struct pid_data* pid);
void init_pid(volatile struct pid_data* pid);

/* Shared memory setup */
#pragma DATA_SECTION(share_buff, ".share_buff")
volatile far struct shared_mem share_buff;

#define LOOPS (*((volatile unsigned int *)0x4A301000))

/*
 * main.c
 */
void main(void) {
    LOOPS = 0;
    //  Blink the LED to show PRU0 is alive.
    __R30 = 0x0000;
    __delay_cycles(10000000);
    __R30 = 0xFFFF;
    __delay_cycles(10000000);
    __R30 = 0x0000;
    __delay_cycles(10000000);

    /* Allow for PRU core 1 to initialize */
    while (!share_buff.init_flag == 1);
    __R30 = 0xFFFF;  //  Turn on LED if initialization happens.

    /* Initialize PIDs */
    init_pid(&share_buff.pid);

    /* Set default PID tunings */
    share_buff.pid.Kp_f    = 500;
    share_buff.pid.Ki_f    = 200;
    share_buff.pid.Kd_f    = 200;

    share_buff.pid.max_output = 0x1000; // Decimal 4096.  This is duty cycle.
    share_buff.pid.min_output = 0;

    share_buff.pid.setpoint = 3000;

    /* Main loop */
	while(1) {
	    update_pid(&share_buff.pid);
	    LOOPS += 1;
	}
}

/*
 * update_pid
 */
void update_pid(volatile struct pid_data* pid) {
    unsigned int p_f, d_f;
    int output_f, output;

    /* Calculate error */
    int error = (pid->input - pid->setpoint);

    /* Calculate P term */
    p_f = pid->Kp_f * error;

    /* Integrate I term */
    pid->int_err += (pid->Ki_f * error) >> SHIFT;

    /* Calculate D term */
    d_f = pid->Kd_f * (pid->output - pid->last_output);

    /* Sum PID output */
    output_f = p_f + pid->int_err + d_f;
    output = output_f >> SHIFT;

    /* Set output_f, check min/max output */
    if (output < pid->min_output) output = pid->min_output;
    if (output > pid->max_output) output = pid->max_output;

    pid->last_output = pid->output;
    pid->output = pid->max_output - output;
}

/*
 * init_pid
 */
void init_pid(volatile struct pid_data* pid) {
    pid->input = 0;
    pid->output = 0;

    pid->setpoint = 0;
    pid->int_err = 0;
}
