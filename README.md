# Fuzzy-PID
<h4>STM Code implementing Fuzzy Logic for PID to drive motor.</br>
Involves UART Communication between STM32F407VG and FPGA.</h4>

Work Flow:</br>
Recieves the number of Ticks as input from FPGA. </br>
Computes the appropriate PWM Value using Fuzzy Logic in STM.</br>
Transmits the computed PWM to FPGA which in turn drives the Motor.</br>
