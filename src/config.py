import odrive
from odrive.enums import *
import time
import math

# Find and connect to the ODrive
odrv0 = odrive.find_any()
print("Connected to ODrive with serial number:", odrv0.serial_number)

odrv = odrv0
odrv.config.dc_bus_overvoltage_trip_level = 25
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf
odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
odrv.axis0.config.motor.pole_pairs = 7
odrv.axis0.config.motor.torque_constant = 0.25091019417475724
odrv.axis0.config.motor.current_soft_max = 10
odrv.axis0.config.motor.current_hard_max = 23
odrv.axis0.config.motor.calibration_current = 1
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 10
odrv.axis0.motor.motor_thermistor.config.enabled = False
odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.POS_FILTER
odrv.axis0.controller.config.vel_limit = 10
odrv.axis0.controller.config.vel_limit_tolerance = 1.2
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.controller.config.input_filter_bandwidth = 100
odrv.can.config.protocol = Protocol.SIMPLE
odrv.can.config.baud_rate = 250000
odrv.axis0.config.can.node_id = 63
odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv.axis0.config.can.encoder_msg_rate_ms = 10
odrv.axis0.config.can.iq_msg_rate_ms = 10
odrv.axis0.config.can.torques_msg_rate_ms = 10
odrv.axis0.config.can.error_msg_rate_ms = 10
odrv.axis0.config.can.temperature_msg_rate_ms = 10
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 10
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.load_encoder = EncoderId.SPI_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.SPI_ENCODER0
odrv.spi_encoder0.config.mode = SpiEncoderMode.AMS
odrv.spi_encoder0.config.ncs_gpio = 17
odrv.config.enable_uart_a = False