#!/usr/bin/env python
import rospy
from Tkinter import *
from geometry_msgs.msg import Vector3


def check_ros():
    if rospy.is_shutdown():
        fen.destroy()
    else:
        fen.after(500, check_ros)


def send_cmd():
    cmd = Vector3()
    cmd.x = rudder_angle.get()
    cmd.y = 1
    cmd_pub.publish(cmd)
    fen.after(500, send_cmd)

rospy.init_node('manual_sailboat')
fen = Tk()

cmd_pub = rospy.Publisher('sailboat/manual', Vector3, queue_size=1)
rudder_angle = DoubleVar()
rudder_slider = Scale(fen, from_=-3.14 / 2, to=3.14 / 2,
                      resolution=0.1, variable=rudder_angle)
rudder_slider.pack()

fen.after(1, send_cmd)
fen.after(1, check_ros)

fen.mainloop()
