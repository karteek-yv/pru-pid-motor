##
 # Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 #  
 #  
 # Redistribution and use in source and binary forms, with or without 
 # modification, are permitted provided that the following conditions 
 # are met:
 # 
 # 	* Redistributions of source code must retain the above copyright 
 # 	  notice, this list of conditions and the following disclaimer.
 # 
 # 	* Redistributions in binary form must reproduce the above copyright
 # 	  notice, this list of conditions and the following disclaimer in the 
 # 	  documentation and/or other materials provided with the   
 # 	  distribution.
 # 
 # 	* Neither the name of Texas Instruments Incorporated nor the names of
 # 	  its contributors may be used to endorse or promote products derived
 # 	  from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 # A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ##
# Clear out old data
rm data.csv
rm data_tail.csv
rm data_tail_tmp.csv
while [ 1 ]; do
    # Log PID output data to CSV
    echo $(date +%H:%M:%S),$(prumsg 31 ro),$(prumsg 31 re),$(prumsg 31 rs) | tee -a data.csv
    # Create readable CSV header
    echo "Time,PWM,RPM,Setpoint" > data_tail_tmp.csv
    # Add last 500 lines to CSV (exclude last line to ensure it is fully written)
    tail -501 data.csv | head -500 >> data_tail_tmp.csv
    # Rename for access by user interface to ensure graph does not attempt to load incomplete data
    mv data_tail_tmp.csv data_tail.csv
done
