#!/usr/bin/env python
from ina219 import INA219
from ina219 import DeviceRangeError
import rospy
from std_msgs.msg import Float32

rospy.init_node('payload_battery_voltage_publisher')
voltage_publisher = rospy.Publisher('payload_battery/voltage', Float32, queue_size=10)

SHUNT_OHMS = 0.1
ina = INA219(SHUNT_OHMS)
ina.configure()

rate = rospy.Rate(2.0)

while not rospy.is_shutdown():

	msg = Float32()
	battery_voltage = 0.0
	try:
		battery_voltage = ina.voltage()
	except DeviceRangeError as e:
		# Current out of device range with specified shunt resister
		print e
	msg.data = battery_voltage
	voltage_publisher.publish(msg)
	rate.sleep()