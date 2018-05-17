trex_driver
===========

This is a driver for the [Pololu TReX Jr Dual Motor Controller DMC02](https://www.pololu.com/product/767/resources), which features a serial interface, PWM controller, driver (H-bridges) for 2 (+1) motors and current sensing.
The driver provides inbound topics to set the PWM duty cycle for the motors, and outboud topics for the measured current.

Multiple boards are supported by separate USB devices/UART connections.
The node uses the expanded protocol, but multiple board on one UART line are currently not supported.
The auxilary motor is currently not implemented.

Launch files
------------
- **config-initial.launch**: Use to set initial parameters, such as ID and BAUD rate. Sets the device to ID 0x07 (default) and 115200 BAUD. Kill the node after startup.
- **default.launch**: Simple launch file for one board.
- **multi.launch**: Launch file for multiple boards with external YAML-file.

Topics
------
- **~/pwm[XY]** (inbound): PWM duty cycle [-1,1] and rotation direction for board X and motor Y (0,1), e.g. ~/pwm00 for motor 1 on first board.
- **~/current[XY]** (outbound): Measured current [A] for board X and motor Y (0,1)

Parameters
----------
- **ports** (array): UART devices, e.g. dev/ttyUSB0
- **baud**: BAUD rate
- **devices** (array): Device ID of the board, deafults to 0x07. The array must have the same length as ports.
- **rate**: Polling rate for the state, currently only the current.
- **Controller parameters**: All documented controller parameters can be set when the node starts. The parameters are sent to all connected controllers.
	Parameter names:
	DeviceNumber,
	RequiredChannels,
	IgnoredChannels,
	ReversedChannels,
	ParabolicChannels,
	Motor1DeadbandBrakePWM,
	Motor2DeadbandBrakePWM,
	SerialTimeout,
	UARTErrorShutdown,
	Motor1PWMPrescaler,
	Motor2PWMPrescaler,
	Motor1PWMMaximum,
	Motor2PWMMaximum,
	AuxiliaryMotorPWMMaximum,
	Motor1Acceleration,
	Motor2Acceleration,
	AuxiliaryMotorAcceleration,
	Motor1BrakeDuration,
	Motor2BrakeDuration,
	Motor1CurrentLimit,
	Motor2CurrentLimit,
	Motor1CurrentLimitProportionalityConstantP,
	Motor2CurrentLimitProportionalityConstantP,
	EnableUARTResponseDelay,
	MotorMode,
	ChannelInputSource,
	CRC7Polynomial,
	UARTSettings,
	ResetAllParameterstoFactoryDefaults.
