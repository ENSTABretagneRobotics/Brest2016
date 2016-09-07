#!/usr/bin/env python


# interface graphique permettant d ajouter des waypoints


import rospy
from Tkinter import *
import tkMessageBox
from process.srv import behavior
from process.msg import BehaviorInfo
from geometry_msgs.msg import PointStamped


def find_coeff(xa, ya, xb, yb):
    a = float(yb - ya) / (xb - xa)
    b = yb - a * xb
    return a, b


def find_larger_ligne(xa, ya, xb, yb, K=2):
    if xb == xa:
        return xa, min(ya, yb) - 100, xb, max(ya, yb) + 100
    a, b = find_coeff(xa, ya, xb, yb)
    mi = max(abs(xb - xa), abs(ya - yb))
    x0, x1 = min(xa, xb) - K * mi, max(xa, xb) + K * mi
    y0, y1 = a * x0 + b, a * x1 + b
    if xa > xb:
        x0, x1, y0, y1 = x1, x0, y1, y0
    return x0, y0, x1, y1

# ###################################################################
# Function for the manager
# ###################################################################


def sendClearSignal():
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='775', f_type='waypoint')
    behavior_sender(info, 'clear_all')


def sendWaypoint(x, y):
    print 'sending waypoint'
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='775', f_type='waypoint',
                        xa=x, ya=y,
                        xb=0, yb=0,
                        K=1, R=1,
                        security='LOW', slowing_R=1,
                        slowing_K=1,
                        effect_range=1)
    behavior_sender(info, 'update')


def sendLigne(xa, ya, xb, yb):
    #  aggrandir la ligne
    # x0, y0, x1, y1 = find_larger_ligne(xa, ya, xb, yb)
    x0, y0, x1, y1 = xa, ya, xb, yb
    # Behavior a envoyer
    info = BehaviorInfo(behavior_id='775', f_type='ligne',
                        xa=x0, ya=y0,
                        xb=x1, yb=y1,
                        K=1, R=10, slowing_R=1,
                        slowing_K=1,
                        effect_range=100)
    behavior_sender(info, 'update')


def calc_distance(msg):
    print 'gps received'
    if play:
        global distance, wp_i, waypoints
        distance = ((waypoints[wp_i][0] - msg.point.x)**2 +
                    (waypoints[wp_i][1] - msg.point.y)**2) ** 0.5
        print distance
        if distance < capture_radius:
            rospy.loginfo('captured')
            print 'captured'
            wp_i += 1
            wp_i %= len(waypoints)

            if type_choice.get() == 'waypoint':
                sendWaypoint(waypoints[wp_i][0], waypoints[wp_i][1])
            elif type_choice.get() == 'ligne' and len(waypoints) > 1:
                sendLigne(waypoints[wp_i][0], waypoints[wp_i][1], waypoints[
                          wp_i - 1][0], waypoints[wp_i - 1][1])
# ###################################################################
# Function for the window
# ###################################################################


def type_chosen(*args):
    print type_choice.get()
    # sendClearSignal()


def remove_all(event=None):
    global play, waypoints
    play = False
    waypoint_list.delete(0, END)
    sendClearSignal()
    waypoints = []


def add_waypoint(event=None):
    x, y = int(xa_entry.get()), int(ya_entry.get())
    text = '{}, {}'.format(x, y)
    waypoint_list.insert(END, text)
    waypoints.append((x, y))


def play_waypoints(event=None):
    global play
    if type_choice.get() == '':
        tkMessageBox.showinfo("Warning", "Choisi un type de suivi")
    elif waypoint_list.size() == 0:
        tkMessageBox.showinfo("Warning", "Aucun waypoint dans la liste")
    elif waypoint_list.size() == 1 and type_choice.get() == 'ligne':
        tkMessageBox.showinfo("Warning", "Pas assez de waypoint dans la liste")
    else:
        play = True
        print type_choice.get()
        if type_choice.get() == 'waypoint':
            sendWaypoint(waypoints[wp_i][0], waypoints[wp_i][1])
        if type_choice.get() == 'ligne' and len(waypoints) > 1:
            sendLigne(waypoints[wp_i][0], waypoints[wp_i][1], waypoints[
                      wp_i - 1][0], waypoints[wp_i - 1][1])


def stop_waypoints(event=None):
    global play, waypoints, wp_i, distance
    play = False
    # sendClearSignal()


# ###################################################################
#
#
# GUI
#
#
# ###################################################################

# *******************************************
# Fenetre
fen = Tk()
fen.bind('<Return>', add_waypoint)
# *******************************************
# Section type de champ
type_frame = Frame(fen, borderwidth=2)
type_title_label = Label(type_frame, text='Type de Suivi')
type_choice = StringVar()
type_choice.trace('w', type_chosen)

waypoint_radio_button = Radiobutton(
    type_frame, text='Waypoint', variable=type_choice, value='waypoint')
ligne_radio_button = Radiobutton(
    type_frame, text='Ligne', variable=type_choice, value='ligne')

# *******************************************
# Section parametres du champ
parameter_frame = Frame(fen, borderwidth=2)
parameter_title_label = Label(parameter_frame, text='Parametres')
xa_label = Label(parameter_frame, text='x')
xa_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=1)

ya_label = Label(parameter_frame, text='y')
ya_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=1)


# *******************************************
# Button de validation
add_waypoint_button = Button(fen, text='Ajouter', command=add_waypoint)
# Button de play
play_stop_frame = Frame(fen, borderwidth=2)
play_buton = Button(fen, text='Play', command=play_waypoints)
stop_buton = Button(fen, text='Stop', command=stop_waypoints)

# *******************************************
# Waypoint List
waypoint_list_frame = Frame(fen, borderwidth=2)
waypoint_list_frame_title_label = Label(waypoint_list_frame, text='History')
# List des behaviors envoyes
waypoint_list = Listbox(waypoint_list_frame)
# Button de suppression des behaviors envoyes
waypoint_list_delete_all_button = Button(
    waypoint_list_frame, text='Clear', command=remove_all)


# *******************************************
type_frame.grid(row=1, column=1)
waypoint_radio_button.pack()
ligne_radio_button.pack()

# *******************************************
parameter_frame.grid(row=2, column=1)
xa_label.grid(row=1, column=1)
xa_entry.grid(row=1, column=2)
ya_label.grid(row=2, column=1)
ya_entry.grid(row=2, column=2)

# *******************************************
waypoint_list_frame.grid(row=1, column=2, rowspan=3)
waypoint_list_frame_title_label.grid(row=1, column=1, columnspan=2)
waypoint_list.grid(row=2, column=1, columnspan=2)
waypoint_list_delete_all_button.grid(row=3, column=1)

# *******************************************
add_waypoint_button.grid(row=3, column=1)

# *******************************************
play_stop_frame.grid(row=3, columnspan=2)
play_buton.grid(column=1)
stop_buton.grid(column=2)


# ###################################################################
# ###################################################################
#
#
# ROS
#
#
# ###################################################################
# ###################################################################

play = False
waypoints = []
distance = 10000
wp_i = 0
capture_radius = 10

rospy.init_node('behavior_waypoint_controller')
# Subscriber to GPS local position
gps_sub = rospy.Subscriber('gps/local_pose', PointStamped, calc_distance)

# Service for sending waypoints
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)


def check_ros():
    if rospy.is_shutdown():
        fen.destroy()
    else:
        fen.after(500, check_ros)
fen.after(1, check_ros)
fen.mainloop()
