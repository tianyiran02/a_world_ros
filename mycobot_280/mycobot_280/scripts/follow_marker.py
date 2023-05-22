#!/usr/bin/env python2
# coding:utf-8
import rospy
from visualization_msgs.msg import Marker
import time
import os

# Type of message communicated with mycobot，与 mycobot 通信的消息类型
from mycobot_communication.msg import MycobotSetAngles, MycobotSetCoords, MycobotPumpStatus


rospy.init_node("gipper_subscriber", anonymous=True)

# Control the topic of mycobot, followed by angle, coordinates, gripper
# 控制 mycobot 的 topic，依次是角度、坐标、夹爪
angle_pub = rospy.Publisher("mycobot/angles_goal",
                            MycobotSetAngles, queue_size=5)
coord_pub = rospy.Publisher("mycobot/coords_goal",
                            MycobotSetCoords, queue_size=5)
# 判断设备：ttyUSB*为M5；ttyACM*为wio，Judging equipment: ttyUSB* is M5；ttyACM* is wio
robot = os.popen("ls /dev/ttyUSB*").readline()

if "dev" in robot:
    Pin = [2, 5]
else:
    Pin = [20, 21]

pump_pub = rospy.Publisher("mycobot/pump_status",
                           MycobotPumpStatus, queue_size=5)

# instantiate the message object，实例化消息对象
angles = MycobotSetAngles()
coords = MycobotSetCoords()
pump = MycobotPumpStatus()

# Deviation value from mycobot's real position,与 mycobot 真实位置的偏差值
x_offset = -20
y_offset = 20
z_offset = 110

# With this variable limit, the fetching behavior is only done once
# 通过该变量限制，抓取行为只做一次
flag = False

# In order to compare whether the QR code moves later,为了后面比较二维码是否移动
temp_x = temp_y = temp_z = 0.0

temp_time = time.time()


def pub_coords(x, y, z, rx=0, ry=0, rz=-35, sp=70, m=2):
    """Post coordinates,发布坐标"""
    coords.x = x
    coords.y = y
    coords.z = z
    coords.rx = rx
    coords.ry = ry
    coords.rz = rz
    coords.speed = sp
    coords.model = m
    # print(coords)
    coord_pub.publish(coords)


def pub_angles(a, b, c, d, e, f, sp):
    """Publishing angle,发布角度"""
    angles.joint_1 = float(a)
    angles.joint_2 = float(b)
    angles.joint_3 = float(c)
    angles.joint_4 = float(d)
    angles.joint_5 = float(e)
    angles.joint_6 = float(f)
    angles.speed = sp
    angle_pub.publish(angles)


def pub_pump(flag, Pin):
    """Publish gripper status,发布夹爪状态"""
    pump.Status = flag
    pump.Pin1 = Pin[0]
    pump.Pin2 = Pin[1]
    pump_pub.publish(pump)


def target_is_moving(x, y, z):
    """Determine whether the target moves"""
    """判断目标是否移动"""
    count = 0
    for o, n in zip((x, y, z), (temp_x, temp_y, temp_z)):
        print(o, n)
        if abs(o - n) < 2:
            count += 1
    print(count)
    if count == 3:
        return False
    return True


def grippercallback(data):
    """callback function,回调函数"""
    global flag, temp_x, temp_y, temp_z
    # rospy.loginfo('gripper_subscriber get date :%s', data)
    #if flag:
    #    print("Exit because flag set!")
    #    return

    # Parse out the coordinate value,解析出坐标值
    # pump length: 88mm
    print("CP1!")
    x = float(format(data.pose.position.x * 1000, ".2f"))
    y = float(format(data.pose.position.y * 1000, ".2f"))
    z = float(format(data.pose.position.z * 1000, ".2f"))

    print("CP5!")
    print(x, y, z)

    # detect heigth + pump height + limit height + offset
    x += x_offset
    y += y_offset
    z = z + 88 + z_offset

    print(x,y,z)
    pub_coords(x, y, z)
    time.sleep(2.5)
    print("CP6!")
    # down
    for i in range(1, 17):
        pub_coords(x, y, z - i * 5, rx=-160, sp=10)
        time.sleep(0.1)
    print("CP7!")
    time.sleep(2)



def main():

    # mark 信息的订阅者,subscribers to mark information
    rospy.Subscriber("visualization_marker", Marker,
                     grippercallback, queue_size=1)

    print("gripper test")
    rospy.spin()


if __name__ == "__main__":
    main()
