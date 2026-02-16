import odrive
from odrive.enums import *
import time
import math

def clear_errors(odrv):
    """Clear all errors on the ODrive"""
    print("Clearing errors...")
    odrv.clear_errors()
    time.sleep(0.5)

def dump_errors(odrv):
    """Print all errors"""
    print("\n=== Error Status ===")
    try:
        # Try newer firmware structure first
        print(f"System errors: {hex(odrv.system.error) if hasattr(odrv, 'system') else 'N/A'}")
    except:
        pass
    
    try:
        print(f"Axis0 errors: {hex(odrv.axis0.error)}")
        print(f"Motor errors: {hex(odrv.axis0.motor.error)}")
        print(f"Controller errors: {hex(odrv.axis0.controller.error)}")
        print(f"Encoder errors: {hex(odrv.axis0.encoder.error)}")
        print(f"SPI Encoder0 errors: {hex(odrv.spi_encoder0.error)}")
    except AttributeError as e:
        print(f"Could not read some error fields: {e}")
    
    print("===================\n")

def configure_odrive(odrv):
    """Apply all configuration settings"""
    print("\n=== Configuring ODrive ===")
    
    # DC Bus Configuration
    print("Configuring DC bus...")
    odrv.config.dc_bus_overvoltage_trip_level = 35
    odrv.config.dc_bus_undervoltage_trip_level = 10.5
    odrv.config.dc_max_positive_current = 10
    odrv.config.dc_max_negative_current = -3.0
    
    # Motor Configuration
    print("Configuring motor...")
    odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
    odrv.axis0.config.motor.pole_pairs = 7
    odrv.axis0.config.motor.torque_constant = 0.25091019417475724
    odrv.axis0.config.motor.current_soft_max = 10
    odrv.axis0.config.motor.current_hard_max = 23
    odrv.axis0.config.motor.calibration_current = 1
    odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.motor.motor_thermistor.config.enabled = False
    
    # Controller Configuration
    print("Configuring controller...")
    odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
    odrv.axis0.controller.config.vel_limit = 50
    odrv.axis0.controller.config.vel_limit_tolerance = 2
    odrv.axis0.config.torque_soft_min = -math.inf
    odrv.axis0.config.torque_soft_max = math.inf
    
    # CAN Configuration
    print("Configuring CAN bus...")
    odrv.can.config.protocol = Protocol.SIMPLE
    odrv.can.config.baud_rate = 250000
    odrv.axis0.config.can.node_id = 0
    odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
    odrv.axis0.config.can.encoder_msg_rate_ms = 10
    odrv.axis0.config.can.iq_msg_rate_ms = 10
    odrv.axis0.config.can.torques_msg_rate_ms = 10
    odrv.axis0.config.can.error_msg_rate_ms = 10
    odrv.axis0.config.can.temperature_msg_rate_ms = 10
    odrv.axis0.config.can.bus_voltage_msg_rate_ms = 10
    
    # Encoder Configuration
    print("Configuring encoder...")
    odrv.axis0.config.load_encoder = EncoderId.SPI_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.SPI_ENCODER0
    odrv.spi_encoder0.config.mode = SpiEncoderMode.AMS
    odrv.spi_encoder0.config.ncs_gpio = 17
    
    # Watchdog and UART
    odrv.axis0.config.enable_watchdog = False
    odrv.config.enable_uart_a = False
    
    print("Configuration complete!")

def save_configuration(odrv):
    """Save configuration and reboot"""
    print("\nSaving configuration...")
    try:
        odrv.save_configuration()
        print("Configuration saved! ODrive will reboot...")
        time.sleep(3)
        return True
    except Exception as e:
        print(f"Error saving configuration: {e}")
        return False

def calibrate_motor(odrv):
    """Run motor calibration"""
    print("\n=== Starting Motor Calibration ===")
    print("WARNING: Motor will move during calibration!")
    print("Ensure motor is free to rotate...")
    time.sleep(2)
    
    clear_errors(odrv)
    
    print("Running motor calibration...")
    odrv.axis0.requested_state = AxisState.MOTOR_CALIBRATION
    
    # Wait for calibration to complete
    while odrv.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)
    
    time.sleep(0.5)
    dump_errors(odrv)
    
    if odrv.axis0.motor.is_calibrated:
        print("✓ Motor calibration successful!")
        print(f"  Phase resistance: {odrv.axis0.motor.config.phase_resistance:.6f} Ω")
        print(f"  Phase inductance: {odrv.axis0.motor.config.phase_inductance:.9f} H")
        return True
    else:
        print("✗ Motor calibration failed!")
        return False

def calibrate_encoder(odrv):
    """Run encoder calibration"""
    print("\n=== Starting Encoder Calibration ===")
    print("WARNING: Motor will rotate during calibration!")
    time.sleep(2)
    
    clear_errors(odrv)
    
    print("Running encoder offset calibration...")
    odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    
    # Wait for calibration to complete
    while odrv.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)
    
    time.sleep(0.5)
    dump_errors(odrv)
    
    if odrv.axis0.encoder.is_ready:
        print("✓ Encoder calibration successful!")
        return True
    else:
        print("✗ Encoder calibration failed!")
        return False

def set_precalibrated(odrv):
    """Mark motor and encoder as pre-calibrated to skip calibration on startup"""
    print("\nMarking motor and encoder as pre-calibrated...")
    odrv.axis0.config.motor.pre_calibrated = True
    odrv.axis0.config.encoder.pre_calibrated = True
    print("Pre-calibrated flags set!")

def test_closed_loop(odrv):
    """Test closed loop control"""
    print("\n=== Testing Closed Loop Control ===")
    print("Entering closed loop control...")
    
    clear_errors(odrv)
    
    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
    time.sleep(1)
    
    if odrv.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
        print("✓ Closed loop control active!")
        print("Motor should now respond to torque commands.")
        
        # Test small torque command
        print("\nTesting small torque command (0.1 Nm for 2 seconds)...")
        odrv.axis0.controller.input_torque = 0.1
        time.sleep(2)
        odrv.axis0.controller.input_torque = 0.0
        
        print("Returning to idle...")
        odrv.axis0.requested_state = AxisState.IDLE
        time.sleep(0.5)
        
        dump_errors(odrv)
        return True
    else:
        print("✗ Failed to enter closed loop control!")
        dump_errors(odrv)
        return False

def main():
    print("=" * 50)
    print("ODrive Configuration and Calibration Script")
    print("=" * 50)
    
    # Find and connect to ODrive
    print("\nSearching for ODrive...")
    try:
        odrv = odrive.find_any()
        print(f"✓ Connected to ODrive (S/N: {odrv.serial_number})")
    except Exception as e:
        print(f"✗ Failed to connect to ODrive: {e}")
        return
    
    # Show initial state
    dump_errors(odrv)
    clear_errors(odrv)
    
    # Configure ODrive
    configure_odrive(odrv)
    
    # Save and reboot
    if not save_configuration(odrv):
        print("Failed to save configuration. Exiting.")
        return
    
    # Reconnect after reboot
    print("\nReconnecting to ODrive...")
    time.sleep(1)
    try:
        odrv = odrive.find_any()
        print("✓ Reconnected!")
    except Exception as e:
        print(f"✗ Failed to reconnect: {e}")
        return
    
    # Run calibrations
    if not calibrate_motor(odrv):
        print("\nMotor calibration failed. Please check connections and try again.")
        return
    
    if not calibrate_encoder(odrv):
        print("\nEncoder calibration failed. Please check encoder connections.")
        return
    
    # Set pre-calibrated flags
    set_precalibrated(odrv)
    
    # Save calibration results
    print("\nSaving calibration results...")
    odrv.save_configuration()
    print("Calibration saved!")
    
    # Test closed loop
    response = input("\nWould you like to test closed loop control? (y/n): ")
    if response.lower() == 'y':
        test_closed_loop(odrv)
    
    print("\n" + "=" * 50)
    print("Configuration and calibration complete!")
    print("=" * 50)
    print("\nYour ODrive is now configured and calibrated.")
    print("To enter closed loop control, use:")
    print("  odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL")
    print("To set torque, use:")
    print("  odrv.axis0.controller.input_torque = <value>")

if __name__ == "__main__":
    main()