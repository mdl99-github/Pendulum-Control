# Cotrolling a pendulum with a rotating arm

This repo contains files to provide insight on how to control a rotating arm-pendulum system using state-space feedback. Using an Arduino board, a command signal is sent to a servomotor that moves an arm attached to
the pendulum. MATLAB files and simulations are also provided, in order to test controllers before using them on the real model. System identification was already perfomed, but the following document can provide enough
guidance so that the reader can perform the task by themselves https://www.st.com/content/dam/AME/2019/Educational%20Curriculums/motor-control/Introduction_to_Integrated_Rotary_Inverted_Pendulum_v2.pdf.
Bear in mind that this document uses a DC motor, that operates differently from the one used in this project.

## List of materials
- Arduino UNO board
- 5V - 3A power source
- 10k potentiometer (to measure arm angle)
- MPU6050 IMU (to measure pendulum angle)
- MG996R 12kg servomotor
- 626ZZ 19mm x 6mm x 6mm ball bearing
- 1 M3 15mm screw
- 1 M3 20mm screw
- 1 M3 25mm screw
- 2 M3 35mm screws
- 5 self-locking nuts
- 1 M3 flat washer
- Elastic band (to reduce potencial high frequency oscilation from arm)
- 3D printed parts
