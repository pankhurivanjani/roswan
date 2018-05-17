#!/usr/bin/env python
import serial, time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

class RosConfig:
	cfg = {
		"devices": [ 0x07 ],
		"ports": [ "/dev/ttyUSB0" ],
		"baud": 19200,
		"rate": 10,
		"scale_i": 0.040, # 40mA per unit, according to datasheet
		#"param_pwm_prescaler1": 0,  #
		#"param_pwm_prescaler2": 0,  #
	}

	dev_params = {
		"DeviceNumber": 0x00,
		"RequiredChannels": 0x01,
		"IgnoredChannels": 0x02,
		"ReversedChannels": 0x03,
		"ParabolicChannels": 0x04,
		"Motor1DeadbandBrakePWM": 0x05,
		"Motor2DeadbandBrakePWM": 0x06,
		"SerialTimeout": 0x07,
		"UARTErrorShutdown": 0x08,
		"Motor1PWMPrescaler": 0x09,
		"Motor2PWMPrescaler": 0x0A,
		"Motor1PWMMaximum": 0x0B,
		"Motor2PWMMaximum": 0x0C,
		"AuxiliaryMotorPWMMaximum": 0x0D,
		"Motor1Acceleration": 0x0E,
		"Motor2Acceleration": 0x0F,
		"AuxiliaryMotorAcceleration": 0x10,
		"Motor1BrakeDuration": 0x11,
		"Motor2BrakeDuration": 0x12,
		"Motor1CurrentLimit": 0x13,
		"Motor2CurrentLimit": 0x14,
		"Motor1CurrentLimitProportionalityConstantP": 0x15,
		"Motor2CurrentLimitProportionalityConstantP": 0x16,
		"EnableUARTResponseDelay": 0x17,
		"MotorMode": 0x7B,
		"ChannelInputSource": 0x7C,
		"CRC7Polynomial": 0x7D,
		"UARTSettings": 0x7E,
		"ResetAllParameterstoFactoryDefaults": 0x7F,
	}

	def __init__(self):
		''' Checks for configuration parameters and writes corresponding values to device.
		    Any of the keys in cfg can be used as a ROS parameter name. '''
		for name in self.cfg.keys():
			pn = '~' + name
			if rospy.has_param(pn):
				val = rospy.get_param(pn)
				self.cfg[name] = val
				rospy.loginfo("Parameter %s=%s" % (name, str(val)))



class PoluluTrex:
	is_open = False
	have_loggederr = False
	pub_cur = [None, None, None]

	def __init__(self, serial, device_id, config, index=0):
		self.ser = serial
		self.device_id = device_id
		self.cfg = config.cfg
		self.dev_params = config.dev_params
		self.index = index
		# Publisher and subscriber
		self.pub_cur[0] = rospy.Publisher("~current%d%d" % (index, 0), Float64, queue_size=10)
		self.pub_cur[1] = rospy.Publisher("~current%d%d" % (index, 1), Float64, queue_size=10)
		self.sub_cmd0 = rospy.Subscriber("~pwm%d%d" % (index, 0), Float64, self.cb_cmd, 0)
		self.sub_cmd1 = rospy.Subscriber("~pwm%d%d" % (index, 1), Float64, self.cb_cmd, 1)

		# Open device
		self.dev_open()

		# Timer
		rospy.Timer(rospy.Duration(1.0 / self.cfg['rate']), self.cb_timer)

	def dev_open(self):
		''' Opens and configures the TReX device '''
		if self.is_open:
			return True
		try:
			self.is_open = True
			self.have_loggederr = False
			# Init
			self.ser.write([0x80, self.device_id, 0x01]) # CMD: get signature
			sig = self.ser.read(9)
			if sig[0:6] != "TReXJr":
				raise Exception("Incorrect signature (%s)" % (sig))
			self.ser.write([0x80, self.device_id, 0x02]) # CMD: get mode
			mode = self.ser.read(1)
			if mode != 'a' and mode != 'r':
				raise Exception("Incorrect mode (%c); change jumpers!" % (mode))
			self.dev_config()
		except Exception as e:
			if not self.have_loggederr:
				rospy.logerr("Port %d TReX %02x: open failed: %s", self.index, self.device_id, e.message)
				#self.have_loggederr = True
			return False

		rospy.loginfo("Port %d TReX %02x: device opened.", self.index, self.device_id)
		return True

	def dev_config(self):
		''' Configures all deveice parameters for which ROS parameters are set '''
		# Go through all device parameters...
		for name in self.dev_params.keys():
			pn = '~' + name
			# ... and check for ROS parameter
			if rospy.has_param(pn):
				param_id = self.dev_params[name]
				val = int(rospy.get_param(pn)) & 0x7f
				# Send to device
				self.ser.write([0x80, self.device_id, 0x2F, param_id, val, 0x55, 0x2A]) # Cmd: Set parameter (0xAF)
				data = self.ser.read(1)
				ok = ord(data) == 0
				rospy.loginfo("Device %d-%02x Parameter %s (0x%02x)=%d: %s" % (self.index, self.device_id, name, param_id, val, "OK" if ok else "FAIL"))


	def cb_timer(self, event):
		''' Timer callback: Read current and publish it '''
		if not self.is_open: return
		self.ser.write([0x80, self.device_id, 0x0f]) # CMD: get motor currents
		curs = self.ser.read(2)
		if len(curs) != 2:
			return
		self.pub_cur[0].publish(Float64(float(ord(curs[0])) * self.cfg['scale_i']))
		self.pub_cur[1].publish(Float64(float(ord(curs[1])) * self.cfg['scale_i']))

	def cb_cmd(self, msg, index):
		''' Callback for command message (PWM duty cycle) '''
		pwm = int(round(msg.data * 0x7f))
		if pwm < -0x7f: pwm = 0x7f
		if pwm >  0x7f: pwm = 0x7f
		rospy.loginfo("Received command for %d-%d: %f->%d" % (self.index, index, msg.data, pwm))
		# Build Command
		cmd = 0
		if index == 0: cmd = 0x40
		if index == 1: cmd = 0x48
		if pwm >= 0: cmd += 2  # CMD: Forward
		if pwm < 0:  cmd += 1   # CMD: Reverse
		self.ser.write([0x80, self.device_id, cmd, abs(pwm)])


def shutdown_hook():
  print "shutdown time!"


if __name__ == '__main__':
	rospy.init_node('trex_driver', anonymous=False)
	roscfg = RosConfig()
	# Open ports
	serials = []
	for port in roscfg.cfg["ports"]:
		ser = serial.Serial(port, roscfg.cfg['baud'], timeout = 0.2)
		serials.append(ser)
	# Create devices
	for (i,device) in enumerate(roscfg.cfg["devices"]):
		# Todo: multiple devices on one port. Need semaphore for port access?
		PoluluTrex(serials[i], device, roscfg, i)
	rospy.on_shutdown(shutdown_hook)
	rospy.spin()
	ser.close()
