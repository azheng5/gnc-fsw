######### Config #########

# UART
odrv0.config.enable_uart_a = True
odrv0.config.gpio12_mode = GpioMode.UART_A
odrv0.config.gpio13_mode = GpioMode.UART_A

# DC power supply config
odrv0.config.dc_bus_overvoltage_trip_level = 24
odrv0.config.dc_bus_undervoltage_trip_level = 8
odrv0.config.dc_max_positive_current = 5

# Lipo config
bat_n_cells = = 4
odrv0.config.dc_bus_undervoltage_trip_level = 8 # 3 for old bat
odrv0.config.dc_bus_overvoltage_trip_level = 26
odrv0.config.dc_max_positive_current = 5
odrv0.config.dc_max_negative_current = -1

# Motor config
odrv0.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv0.axis0.config.motor.pole_pairs = 7
odrv0.axis0.config.motor.torque_constant = 8.27 / 200
odrv0.axis0.config.motor.calibration_current = 2 # current used for motor calibration
odrv0.axis0.config.motor.resistance_calib_max_voltage = 2.0
odrv0.axis0.config.calibration_lockin.current = 8 # current used for encoder calibration
# odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION

odrv0.axis0.config.motor.current_soft_max = 30 #change?
odrv0.axis0.config.motor.current_hard_max = 40 #change?

# SPI config
odrv0.spi_encoder0.config.mode = SpiEncoderMode.CUI
odrv0.spi_encoder0.config.ncs_gpio = 17
odrv0.axis0.config.load_encoder = EncoderId.SPI_ENCODER0
odrv0.axis0.config.commutation_encoder = EncoderId.SPI_ENCODER0

#AMT212 config
odrv0.amt21_encoder_group0.config.event_driven_mode = True
odrv0.amt21_encoder_group0.config.enable = True
odrv0.axis0.config.load_encoder = EncoderId.AMT21_ENCODER0
odrv0.axis0.config.commutation_encoder = EncoderId.AMT21_ENCODER0

######### Axis states #########

# Measure phase resistance and phase inductance of the motor
# After that you don’t have to run the motor calibration on the next start up.
odrv0.axis0.requested_state = AxisState.MOTOR_CALIBRATION
# Turn the motor in one direction for a few seconds and then back to measure the offset between the encoder position and the electrical phase.
odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
# Run closed loop control. The action depends on the controller.config.control_mode.
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
# IDLE (disable motor pwm)
odrv0.axis0.requested_state = 1 

######## Control modes ########

# POSITION_CONTROL
# Uses the inner torque loop, the velocity control loop, and the outer position control loop.
# Use input_pos to command desired position, input_vel to command velocity feed-forward, and input_torque for torque feed-forward.
odrv0.axis0.controller.config.control_mode = 3


###### Input modes ########
odrv0.axis0.controller.config.input_mode = 1 # passthru
odrv0.axis0.controller.input_pos = 0 # turn
odrv0.axis0.controller.input_vel = 0 # turn/s
odrv0.axis0.controller.input_torque = 0 # Nm

odrv0.axis0.controller.config.input_mode = 3 # position tracking filter

# Tuning
# Current thru motor linealy relates to torque of motor
# Position control is a P controller, velocity control is a PI controller

# The higher it is, the lower the posn steady state error
odrv0.axis0.controller.config.pos_gain = 15

# Increasing it any more would make the motor too jerky (risks too much DC current being pulled)
# Decreasing it any more would increase the posn steady state error
odrv0.axis0.controller.config.vel_gain = 0.1

# Not needed
odrv0.axis0.controller.config.vel_integrator_gain = 0

odrv0.axis0.controller.config.vel_limit = 5 # if velocity exceeds limit will error!!!
odrv0.axis0.controller.config.vel_limit_tolerance = 2

# System monitoring
start_liveplotter(lambda:[odrv0.axis0.pos_vel_mapper.pos_rel, odrv0.axis0.controller.pos_setpoint])
odrv0.axis0.pos_vel_mapper.pos_rel # relative n since startup in turns
odrv0.axis0.pos_vel_mapper.pos_abs # counts
odrv0.axis0.pos_vel_mapper.vel # turn/s
odrv0.axis0.foc.Iq_setpoint # commanded motor current (A)
odrv0.axis0.foc.Iq_measured # measured motor current (A)
# torque (Nm) = 8.27*current/KV

odrv0.save_configuration()

time.sleep(3)
odrv0.axis0.controller.input_pos = 1
time.sleep(1)
odrv0.axis0.controller.input_pos = 0
time.sleep(1)
odrv0.axis0.controller.input_pos = 0.5
time.sleep(0.6)
odrv0.axis0.controller.input_pos = 1
time.sleep(0.6)
odrv0.axis0.controller.input_pos = 0.5
time.sleep(0.6)
odrv0.axis0.controller.input_pos = 0
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0.25
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0.5
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0.75
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 1
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0.75
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0.5
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0.25
time.sleep(0.3)
odrv0.axis0.controller.input_pos = 0