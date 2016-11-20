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
#include <limits.h>
#include <am335x/pru_cfg.h>
#include <am335x/pru_ecap.h>
#include <am335x/sys_pwmss.h>
#include <am335x/pru_intc.h>
#include <rsc_types.h>
#include <pru_virtqueue.h>
#include <pru_rpmsg.h>
//#include <sys_mailbox.h>
#include "resource_table_1.h"

/* Shared PID data structure - ensure both shared stucts match PRU 0 */
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

/* RPMsg data struct */
struct rpmsg_unit {
    char cmd;
    unsigned int msg;
};

/* PRU GPIO Registers */
volatile register uint32_t __R30;
volatile register uint32_t __R31;

/* Non-CT register defines */
#define CM_PER_EPWMSS1 (*((volatile unsigned int *)0x44E000CC))

/* PWM generation definitions*/
#define PERIOD_CYCLES       0x1000

/* Encoder definitions */
#define TICKS_PER_REV       16
#define SAMPLES_PER_SEC     12
#define SEC_PER_MIN         60

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT            ((uint32_t) 1 << 31) 

//  The PRU-ICSS system events used for RPMsg are defined in the Linux device
//  tree.
//  PRU0 uses system event 16 (to ARM) and 17 (from ARM)
//  PRU1 uses system event 18 (to ARM) and 19 (from ARM)
#define TO_ARM_HOST 18
#define FROM_ARM_HOST 19

/*
 * The name 'rpmsg-pru' corresponds to the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME           "rpmsg-pru"

#define CHAN_DESC           "Channel 30"
#define CHAN_PORT           30

#define CHAN_DESC_2         "Channel 31"
#define CHAN_PORT_2         31

/* Prototypes */
void init_pwm();
void init_eqep();
int get_enc_rpm();
void init_rpmsg(struct pru_rpmsg_transport* transport);
void rpmsg_interrupt(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst, uint16_t len);
void rpmsg_isr(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst);
void int_to_payload(uint8_t *payload, int data);
unsigned int payload_to_int(uint8_t *payload);

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

/* RPMsg buffer, global to reduce stack size */
uint8_t payload[RPMSG_BUF_SIZE];

/* Shared memory setup */
#pragma DATA_SECTION(share_buff, ".share_buff")
volatile far struct shared_mem share_buff;

/*
 * main.c
 */
void main(void) {
    /* RPMsg variables */
    struct pru_rpmsg_transport transport;
    uint16_t src, dst, len;
//    volatile uint8_t *status;

    /* allow OCP master port access by the PRU so the PRU can read external memories */
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Initialize peripheral fuctions */
    init_eqep();
    init_pwm();

    /* Initialize RPMsg */
    init_rpmsg(&transport);

    /* Set init flag to signal PID to continue initializing */
    share_buff.init_flag = 1;

    while (1) {
        /* Get userspace messages */
        rpmsg_interrupt(&share_buff.pid, &transport, payload, dst, src, len);

        /* Write PWM speed (ACMP) */
        CT_ECAP.CAP2_bit.CAP2 = share_buff.pid.output;

        /* Save write cycles by waiting until unit time event */
        if (PWMSS1.EQEP_QFLG & 0x0800) {
            PWMSS1.EQEP_QCLR |= 0x0800;
            share_buff.pid.input = get_enc_rpm();
        }
    }
}

/*
 * Init APWM
 */
void init_pwm() {
    /* Set PWM memory to 0 */
    share_buff.pwm_out = 0;

    /* Enable APWM mode and enable asynchronous operation; set polarity to active high */
    CT_ECAP.ECCTL2 = 0x02C0;

    /* Set number of clk cycles in a PWM period (APRD) */
    CT_ECAP.CAP1 = PERIOD_CYCLES;

    /* Enable ECAP PWM Freerun counter */
    CT_ECAP.ECCTL2 |= 0x0010;
}

/*
 * Init eQEP
 */
void init_eqep() {
    /* Set RPM memory to 0 */
    share_buff.enc_rpm = 0;

    /* Enable PWMSS1 clock signal generation */
    while (!(CM_PER_EPWMSS1 & 0x2))
        CM_PER_EPWMSS1 |= 0x2;

    /* Set to defaults in quadriture mode */
    PWMSS1.EQEP_QDECCTL = 0x00;

    /* Enable unit timer
     * Enable capture latch on unit time out
     * Enable quadrature position counter
     * Enable software loading of position counter
     * Reset position counter on unit time event to gauge RPM
     */
    PWMSS1.EQEP_QEPCTL = 0x308E;

    /* Set prescalers for EQEP Capture timer and UPEVNT */
    /* Note: EQEP Capture unit must be disabled before changing prescalar */
    PWMSS1.EQEP_QCAPCTL = 0x0073;

    /* Enable EQEP Capture */
    PWMSS1.EQEP_QCAPCTL |= 0x8000;

    /* Enable unit time out interupt */
    PWMSS1.EQEP_QEINT |= 0x0800;

    /* Clear encoder count */
    PWMSS1.EQEP_QPOSCNT_bit.QPOSCNT = 0x00000000;

    /* Set max encoder count */
    PWMSS1.EQEP_QPOSMAX_bit.QPOSMAX = UINT_MAX;

    /* Clear timer */
    PWMSS1.EQEP_QUTMR_bit.QUTMR = 0x00000000;

    /* Set unit timer period count */
    /*  QUPRD = Period * 100MHz */
    PWMSS1.EQEP_QUPRD_bit.QUPRD = 0x007FFFFF; // (~1/12s) @ 100MHz

    /* Clear all interrupt bits */
    PWMSS1.EQEP_QCLR = 0xFFFF;
}

/*
 * get_enc_rpm()
 */
int get_enc_rpm() {
    int rpm = 0;

    /* Check for overrun/overflow errors, flash LED and show RPM as 0 */
    if (PWMSS1.EQEP_QEPSTS &= 0x0C) {
        PWMSS1.EQEP_QEPSTS |= 0x0C;
        __R30 = 0xFF;
        rpm = 0;
    } else {
        __R30 = 0x00;
        rpm = (PWMSS1.EQEP_QPOSLAT * SAMPLES_PER_SEC * SEC_PER_MIN) / TICKS_PER_REV;
    }

    return rpm;
}

/*
 * init_rpmsg
 */
void init_rpmsg(struct pru_rpmsg_transport* transport) {
	volatile uint8_t *status;

    /* clear the status of event MB_INT_NUMBER (the mailbox event) and enable the mailbox event */
    CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
//    CT_MBX.IRQ[MB_USER].ENABLE_SET |= 1 << (MB_FROM_ARM_HOST * 2);

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

    /* Initialize pru_virtqueue corresponding to vring0 (PRU to ARM Host direction) */
    pru_rpmsg_init(transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

    /* Initialize pru_virtqueue corresponding to vring1 (ARM Host to PRU direction) */
//    pru_rpmsg_init(transport, &resourceTable.rpmsg_vring1, &resourceTable.rpmsg_vring0, TO_ARM_HOST, FROM_ARM_HOST);

    /* Create the RPMsg channels between the PRU and ARM user space using the transport structure. */
    while (pru_rpmsg_channel(RPMSG_NS_CREATE, transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
//    while (pru_rpmsg_channel(RPMSG_NS_CREATE, transport, CHAN_NAME, CHAN_DESC_2, CHAN_PORT_2) != PRU_RPMSG_SUCCESS);
}

/*
 * rpmsg_interrupt
 */
void rpmsg_interrupt(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst, uint16_t len) {
    /* Check bit 31 of register R30 to see if the mailbox interrupt has occurred */
    if(__R31 & HOST_INT){

        /* Clear the mailbox interrupt */
 //       CT_MBX.IRQ[MB_USER].STATUS_CLR |= 1 << (MB_FROM_ARM_HOST * 2);
        /* Clear the event status, event MB_INT_NUMBER corresponds to the mailbox interrupt */
        CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

        /* Use a while loop to read all of the current messages in the mailbox */
        //while(CT_MBX.MSGSTATUS_bit[MB_FROM_ARM_HOST].NBOFMSG > 0)
            /* Check to see if the message corresponds to a receive event for the PRU */
        //    if(CT_MBX.MESSAGE[MB_FROM_ARM_HOST] == 1)
                /* Receive the message */
                if(pru_rpmsg_receive(transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS){
                    /* Service the new interrupt */
                    rpmsg_isr(pid, transport, payload, src, dst);
                }}}

/*
 * rpmsg_isr
 */
void rpmsg_isr(volatile struct pid_data* pid, struct pru_rpmsg_transport *transport, uint8_t *payload,
        uint16_t src, uint16_t dst) {
    struct rpmsg_unit* rpunit;

    rpunit = (struct rpmsg_unit *) payload;

    /* Check command */
    switch(rpunit->cmd) {
    /* Set setpoint */
    case ('s'):
        pid->setpoint = rpunit->msg;
        rpunit->msg = pid->setpoint;
        break;
    /* Set Kp */
    case ('p'):
        pid->Kp_f = rpunit->msg;
        rpunit->msg = pid->Kp_f;
        break;
    /* Set Ki */
    case ('i'):
        pid->Ki_f = rpunit->msg;
        rpunit->msg = pid->Ki_f;
        break;
    /* Set Kd */
    case ('d'):
        pid->Kd_f = rpunit->msg;
        rpunit->msg = pid->Kd_f;
        break;
    /* Readback output PWM */
    case ('o'):
        pid->output = rpunit->msg;
        rpunit->msg = pid->output;
        break;
    /* Readback setpoint */
    case ('r'^'s'):
        rpunit->msg = pid->setpoint;
        break;
    /* Readback Kp */
    case ('r'^'p'):
        rpunit->msg = pid->Kp_f;
        break;
    /* Readback Ki */
    case ('r'^'i'):
        rpunit->msg = pid->Ki_f;
        break;
    /* Readback Kd */
    case ('r'^'d'):
        rpunit->msg = pid->Kd_f;
        break;
    /* Readback encoder RPM */
    case ('r'^'e'):
        rpunit->msg = pid->input;
        break;
    /* Readback output PWM */
    case ('r'^'o'):
        rpunit->msg = pid->output;
        break;
    }
    /* Send message back to host */
    pru_rpmsg_send(transport, dst, src, &rpunit->msg, 4);
}
