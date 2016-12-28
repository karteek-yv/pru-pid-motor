    Beaglebone Green PRU PID Motor Speed Control Project
    
    Copyright (C) 2016  GREGORY RAVEN

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    
    This is a revised implementation of a project published by Texas
    Instruments:
    
    http://www.ti.com/lit/ug/tidubj6/tidubj6.pdf
    
    The above project is based on an earlier version of the PRU 
    RemoteProc framework which used the "mailbox" system on the SOC.  
    The latest framework has changed to system interrupts, and the code
    has been revised for compatibility with these changes.
    
    The project uses the "Beaglebone Green" (BBG) development board:
    
    http://beagleboard.org/green
    
    A youtube video demonstrates the project:
    
    https://youtu.be/wzNGHVjAqL8
    
    This device includes two "Programmable Real-time Units" which are 
    32-bit RISC processors which are intended to offload real-time tasks
    from the embedded Linux running on the host ARM processor.
    
    The system implements a "Proportional Integral Derivative" type
    digital feedback controller.  The controlled parameter is the
    rotational speed of a DC motor.  The DC motor is fitted with a
    "quadrature encoder" which provides both RPM and directional data.
    
    The DC motor/encoder recommended is different than the one shown in
    the TI documentation.  A motor/encoder obtained from eBay provided
    excellent results in this system.
    
    The quadrature encoder output is connected to the P8 header on the
    BBG.  The DC motor is controlled via "Pulse Width Modulation" which
    is sourced from header P9.  The PWM peripheral resides on PRU1, 
    while the quadrature timing/decoding peripheral is located outside
    the PRU and is accessed via the internal bus.
    
    A user-space C program provides simple control of the PID loop via
    character devices instantiated with the RemoteProc Messaging driver.
    
    An HTTP server capability allows the graphing of control loop
    behavior in a web page.  This is functional and can easily be set
    up to serve the web page from the BBG.  Capability to control RPM
    and PID loop parameters is possible from the web page, but it was
    not successfully demonstrated.
    
    Regards,
    Greg
    
