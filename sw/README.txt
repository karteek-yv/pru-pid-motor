PRU-ICSS PID Motor Demo Software Folder

DESCRIPTION
This folder contains the software necessary to run the
PRU-ICSS PID Motor Demo. A top-level Makefile is
provided in order to build all three binaries at one time.
In order to use the Makefile you will need to export three
environment variables:

ARM Cross-Compiler Toolchain:
export ARM_CCT=${HOME}/ti-processor-sdk-linux-am335x-evm-02.00.02.11/linux-devkit/sysroots/x86_64-arago-linux/usr/bin 

PRU Code Generation Tools:
export PRU_CGT=${HOME}/ti-processor-sdk-linux-am335x-evm-02.00.02.11/linux-devkit/sysroots/x86_64-arago-linux/usr/share/ti/cgt-pru

PRU Software Support Package:
export PRU_SSP=${HOME}/ti-processor-sdk-linux-am335x-evm-02.00.02.11/example-applications/pru-icss-4.0.2

Once these three environment variables are exported you can
use 'make' and 'make clean'. Generated files and output
binaries will be placed in a directory named 'gen' inside
each of the three project folders.

WHAT'S INCLUDED?
	This package includes the following resources:

	DIRECTORY	CONTENTS
	---------	--------
	binaries	Contains pre-built binaries
	dtsi		devicetree include file for pin muxing
	prumsg		ARM user space application for sending messages
	prupid_fw_0	PRU0 firmware for implementing the PID algorithm
	prupid_fw_1	PRU1 firmware that reads the encoder, controls the PWM, and handles RPMsg	
	prupid_server	PHP webpage to control the PID set point along with a small server
