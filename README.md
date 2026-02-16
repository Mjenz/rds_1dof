# rds_1dof
Code for RDS one DOF prototype, communicating with Odrive pro over CAN.
This repo is intended for Teensy 4.1 but will also work on Teensy 4.0 with some adjustments.

### Start Up 
0. Use [Odrive WebGui](https://gui.odriverobotics.com/configuration) to configure odrive for torque control
1. Clone repo into a PlatformIO workspace. 
2. Build and upload to teensy 4.1
3. Use Teleplot to get real time feedback on controller performance

### RDS Group Notes
We are using [this](https://aifitlab.com/products/dji-robomaster-m2006-p36-brushless-dc-gear-motor) motor
With [this](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/AS5048B-ADAPTERBOARD/3188613) SPI encoder
