<!--
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
-->
<html>
  <head>
    <?php
      $setpoint=0;
      $kp=0;
      $ki=0;
      $kd=0;
      $return_val=0;

      /* Load current control values from PRU */
      exec("prumsg 30 rs", $setpoint, $return_val);
      $setpoint = $setpoint[0] + 0;
      exec("prumsg 30 rp", $kp, $return_val);
      $kp = $kp[0] + 0;
      exec("prumsg 30 ri", $ki, $return_val);
      $ki = $ki[0] + 0;
      exec("prumsg 30 rd", $kd, $return_val);
      $kd = $kd[0] + 0;

      /* Write control values if set in URL */
      if (isset($_GET['speed'])) {
        $setpoint = $_GET['speed'];
        exec( "prumsg 30 s $setpoint" );
      }

      if (isset($_GET['Kp'])) {
        $kp = $_GET['Kp'];
        exec( "prumsg 30 p $kp" );
      }

      if (isset($_GET['Ki']))
      {
        $ki = $_GET['Ki'];
        exec( "prumsg 30 i $ki" );
      }

      if (isset($_GET['Kd']))
      {
        $kd = $_GET['Kd'];
        exec( "prumsg 30 d $kd" );
      }
    ?>

    <!-- Load c3.css for graph -->
    <link href="js/c3/c3.css" rel="stylesheet" type="text/css">
    <!-- Load d3.js and c3.js for graph -->
    <script src="js/d3/d3.min.js" charset="utf-8"></script>
    <script src="js/c3/c3.min.js"></script>

    <title>PID Web Control Interface</title>

    <style type="text/css">
      body { font-family: sans-serif; font-size: 20px; font-weight: bold;}
      p { margin: 0; padding: 0 }
      button { margin: 2px auto; }
    </style>
  </head>
    <body>
    <div id="control_header" style="float: left; width: 27%;">
      <h3>PID Web Control Interface</h2>
      <br><br>
      <div>
          <p>PID Setpoint:</p>
          <input type="range" name="setpoint" min="0" max="6000" style="width: 100%;"
          onchange="updateSetpoint(this.value);"  value="<?php echo $setpoint ?>">
          <input type="text" id="setpoint_text" value="<?php echo $setpoint ?>">
          <button type="button" onclick="location.href='index.php?speed='
          + document.getElementById('setpoint_text').value">Update Setpoint</button>
      </div>

      <br><br>

      <div>
          <p>PID Tuning:</p>
          Kp: <input type="text" id="Kp_text" value="<?php echo $kp ?>">
          <button type="button" onclick="location.href='index.php?Kp='
          + document.getElementById('Kp_text').value">Update Kp</button>
          <br>
          Ki: <input type="text" id="Ki_text" value="<?php echo $ki ?>">
          <button type="button" onclick="location.href='index.php?Ki='
          + document.getElementById('Ki_text').value">Update Ki</button>
          <br>
          Kd: <input type="text" id="Kd_text" value="<?php echo $kd ?>">
          <button type="button" onclick="location.href='index.php?Kd='
          + document.getElementById('Kd_text').value">Update Kd</button>
      </div>
    </div>

    <div id="chart" style="margin-left: 27%;"></div>

    <script type="text/javascript">
      /* Fill page height with chart */
      var chart_height = (document.body.clientHeight - 25);

      /* Set graph parameters */
      var chart = c3.generate({
        bindto: '#chart',
        data: {
          url: 'data_tail.csv',
          hide: ['Time'],
          axes: {
            PWM: 'y2'
          }
        },
        axis : {
          x : {
            label: "Time (s)",
            tick: {
              count: 50,
              // Convert axis values to integers
              format: function (x) { return x | 0; }
            }
          },
          y : {
            label: "RPM",
            max: 7000,
            min: 0
          },
          y2 : {
            show: true,
            label: "PWM",
            max: 4096,
            min: 0
          }
        },
        size: {
          height: chart_height
        },
        point: {
          show: false
        },
        tooltip: {
            show: false
        },
        subchart: {
          show: true
        }
      });

      /* Load fresh data every 750 ms */
      setInterval(function () {
        chart.load({
          url: 'data_tail.csv'
        });
      }, 750);

      /* Update setpoint text box when slider has been dragged */
      function updateSetpoint(val) {
        document.getElementById('setpoint_text').value = val;
      }

      function onLoad() {
        var val = document.getelementById('setpoint_text').value;

      }
    </script>

  </body>
</html>
