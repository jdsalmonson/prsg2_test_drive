# testbag.py - script to open and plot contents of a ROS bag from the PRSG2
# Jay Salmonson
# 8/8/2017

import rosbag
import numpy as np

bagfile = "prsg2-sensors_2017-08-10-22-21-16.bag"
#bagfile = "prsg2-sensors_2017-08-09-22-13-48.bag"
#bagfile = "prsg2-sensors_2017-08-07-23-04-16.bag"
#bagfile = "prsg2-sensors_2017-08-07-23-10-04.bag"

bag = rosbag.Bag(bagfile)
#topic, msg, t = bag.read_messages(topics=['/arduino/sensor/battery_voltage']).next()

#: read bag data
voltage = []
tm = []
for topic, msg, t in bag.read_messages(topics=['/arduino/sensor/battery_voltage']):
    voltage.append(msg.value)
    tm.append(msg.header.stamp.to_nsec())

#get a time zero:
tm0 = tm[0]

ir_1 = []
tm_ir_1 = []
for _, msg, t in bag.read_messages(topics=['/arduino/sensor/ir_1']):
    ir_1.append(msg.range)
    tm_ir_1.append(msg.header.stamp.to_nsec())

ir_2 = []
tm_ir_2 = []
for _, msg, t in bag.read_messages(topics=['/arduino/sensor/ir_2']):
    ir_2.append(msg.range)
    tm_ir_2.append(msg.header.stamp.to_nsec())

servo_1 = []
tm_servo_1 = []
for _, msg, t in bag.read_messages(topics=['/arduino/sensor/servo_1']):
    servo_1.append(msg.value)
    tm_servo_1.append(msg.header.stamp.to_nsec())

servo_2 = []
tm_servo_2 = []
for _, msg, t in bag.read_messages(topics=['/arduino/sensor/servo_2']):
    servo_2.append(msg.value)
    tm_servo_2.append(msg.header.stamp.to_nsec())

odom_angz = []
odom_linx = []
tm_odom = []
for _, msg, t in bag.read_messages(topics=['/odom']):
    if msg.twist.twist.linear.z <> 0. or msg.twist.twist.linear.y <> 0.:
        print(msg.twist.twist)
    odom_angz.append(msg.twist.twist.angular.z)
    odom_linx.append(msg.twist.twist.linear.x)
    tm_odom.append(msg.header.stamp.to_nsec())

cmdvel_angz = []
cmdvel_linx = []
tm_cmdvel = []
for _, msg, msg_tm in bag.read_messages(topics=['/cmd_vel']):
    if msg.linear.z <> 0. or msg.linear.y <> 0.:
        print(msg.twist.twist)
    cmdvel_angz.append(msg.angular.z)
    cmdvel_linx.append(msg.linear.x)
    tm_cmdvel.append(msg_tm.to_nsec())

bag.close()

#: plot data
from pylab import plt

f, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
ax1.set_title(bagfile)
ax1.plot((np.array(tm)-tm0)/1.e9, np.array(voltage), 'k', label="voltage [mV]")
ax1.legend(frameon=True, framealpha=0.5)
ax2.plot((np.array(tm_ir_1)-tm0)/1.e9, np.array(ir_1), 'r-', label="ir_1")
ax2.plot((np.array(tm_ir_2)-tm0)/1.e9, np.array(ir_2), 'b-', label="ir_2")
ax2.legend(frameon=True, framealpha=0.5)
ax3.plot((np.array(tm_servo_1)-tm0)/1.e9, np.array(servo_1), 'r-', label="servo_1")
ax3.plot((np.array(tm_servo_2)-tm0)/1.e9, np.array(servo_2), 'b-', label="servo_2")
ax3.legend(frameon=True, framealpha=0.5)
ax4.plot((np.array(tm_odom)-tm0)/1.e9, np.array(odom_linx), 'g-', label="odom.linear.x")
ax4.plot((np.array(tm_odom)-tm0)/1.e9, np.array(odom_angz)/2./np.pi, 'm-', label="odom.angular.z")
ax4.plot((np.array(tm_cmdvel)-tm0)/1.e9, np.array(cmdvel_linx), 'c-', label="cmd_vel.linear.x")
ax4.plot((np.array(tm_cmdvel)-tm0)/1.e9, 2.*np.pi*np.array(cmdvel_angz), 'y-', label="cmd_vel.angular.z")
ax4.legend(frameon=True, framealpha=0.5)
ax4.set_xlabel("time [s]")
f.subplots_adjust(hspace=0)
plt.show()

