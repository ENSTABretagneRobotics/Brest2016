#!/usr/bin/env python
import rospy
from Tkinter import *
from process.msg import BehaviorInfo
from process.srv import behavior


def default_widget_state():
    for child in parameter_frame.winfo_children():
        if child.winfo_class() != 'Label':
            child.configure(state='disable')
    validate_button.configure(state='disable')


def type_chosen(*args):
    print type_choice.get()
    configure_widgets(type_choice.get())


def configure_widgets(type_champ):
    default_widget_state()
    id_entry.configure(state='normal')
    xa_entry.configure(state='normal')
    ya_entry.configure(state='normal')
    validate_button.configure(state='normal')
    if type_champ == 'waypoint':
        s_entry.configure(state='normal')
    elif type_champ == 'ligne':
        xb_entry.configure(state='normal')
        yb_entry.configure(state='normal')
        r_entry.configure(state='normal')
        s_entry.configure(state='normal')
    elif type_champ == 'patrol_circle':
        r_entry.configure(state='normal')
        s_entry.configure(state='normal')
    elif type_champ == 'limite':
        xb_entry.configure(state='normal')
        yb_entry.configure(state='normal')
        r_entry.configure(state='normal')
        s_entry.configure(state='normal')


def getInfos():
    return BehaviorInfo(behavior_id=id_value.get(), type=type_choice.get(),
                        xa=float(xa_entry.get()), ya=float(ya_entry.get()),
                        xb=float(xb_entry.get()), yb=float(yb_entry.get()),
                        s=float(s_entry.get()), r=float(r_entry.get()))


def send_field(event=None):
    info = getInfos()
    confirmation = behavior_sender(info)
    print confirmation

# *******************************************
# Fenetre
fen = Tk()
fen.bind('<Return>', send_field)
# *******************************************
# Section type de champ
type_frame = Frame(fen, borderwidth=2)
type_title_label = Label(type_frame, text='Type de Champ')
type_choice = StringVar()
type_choice.trace('w', type_chosen)

waypoint_radio_button = Radiobutton(
    type_frame, text='Waypoint', variable=type_choice, value='waypoint')
ligne_radio_button = Radiobutton(
    type_frame, text='Ligne attractive', variable=type_choice, value='ligne')
patrol_radio_button = Radiobutton(
    type_frame, text='Patrouille Circulaire',
    variable=type_choice, value='patrol_circle')
limite_radio_button = Radiobutton(
    type_frame, text='Limite', variable=type_choice, value='limite')

# *******************************************
# Section parametres du champ
parameter_frame = Frame(fen, borderwidth=2)
parameter_title_label = Label(parameter_frame, text='Parametres')

id_label = Label(parameter_frame, text='ID')
id_value = StringVar()
id_value.set('001')
id_entry = Entry(parameter_frame, textvariable=id_value, width=10)

xa_label = Label(parameter_frame, text='xa')
xa_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=0.1, format='%.2f')

ya_label = Label(parameter_frame, text='ya')
ya_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=0.1, format='%.2f')

xb_label = Label(parameter_frame, text='xb')
xb_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=0.1, format='%.2f')

yb_label = Label(parameter_frame, text='yb')
yb_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=0.1, format='%.2f')

r_label = Label(parameter_frame, text='r')
r_entry = Spinbox(parameter_frame, from_=1, to=50,
                  increment=0.1, format='%.2f')

s_label = Label(parameter_frame, text='s')
s_entry = Spinbox(parameter_frame, from_=-1, to=1,
                  increment=2)
s_entry.delete(0, "end")
s_entry.insert(0, -1)

# Button de validation
validate_button = Button(fen, text='Envoyer', command=send_field)

# *******************************************
# *******************************************
# Packing and griding

type_frame.pack()
type_title_label.grid(row=1, column=1)
waypoint_radio_button.grid(row=2, column=1)
ligne_radio_button.grid(row=2, column=2)
patrol_radio_button.grid(row=2, column=3)
limite_radio_button.grid(row=2, column=4)

parameter_frame.pack()
parameter_title_label.grid(row=1, column=1)
id_label.grid(row=2, column=1)
id_entry.grid(row=2, column=2)
xa_label.grid(row=3, column=1)
xa_entry.grid(row=3, column=2)
ya_label.grid(row=4, column=1)
ya_entry.grid(row=4, column=2)
xb_label.grid(row=3, column=3)
xb_entry.grid(row=3, column=4)
yb_label.grid(row=4, column=3)
yb_entry.grid(row=4, column=4)
r_label.grid(row=5, column=1)
r_entry.grid(row=5, column=2)
s_label.grid(row=5, column=3)
s_entry.grid(row=5, column=4)

validate_button.pack()

default_widget_state()

# Loop
rospy.init_node('gui_behavior_adder')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)

fen.mainloop()
