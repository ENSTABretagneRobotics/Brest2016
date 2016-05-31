#!/usr/bin/env python
import rospy
from Tkinter import *
from process.msg import BehaviorInfo
from process.srv import behavior


# Redefinition of class Behavior Info
def behavior2string(self):
    return '{} ({})'.format(self.behavior_id, self.f_type)

BehaviorInfo.toString = behavior2string


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
    if type_champ in 'waypoint':
        K_entry.configure(state='normal')
    elif type_champ == 'ligne':
        xb_entry.configure(state='normal')
        yb_entry.configure(state='normal')
        R_entry.configure(state='normal')
        K_entry.configure(state='normal')
        effect_range_entry.configure(state='normal')
    elif type_champ == 'patrol_circle':
        R_entry.configure(state='normal')
        K_entry.configure(state='normal')
    elif type_champ == 'obst_point':
        R_entry.configure(state='normal')
        K_entry.configure(state='normal')
        security_entry.configure(state='normal')
        slowing_R_entry.configure(state='normal')
        slowing_K_entry.configure(state='normal')
    elif type_champ == 'limite':
        xb_entry.configure(state='normal')
        yb_entry.configure(state='normal')
        R_entry.configure(state='normal')
        K_entry.configure(state='normal')
        security_entry.configure(state='normal')
        slowing_R_entry.configure(state='normal')
        slowing_K_entry.configure(state='normal')


def getInfos():
    sec = int(security_entry.get())
    sec = ['LOW', 'MEDIUM', 'HIGH'][sec]
    return BehaviorInfo(behavior_id=id_value.get(), f_type=type_choice.get(),
                        xa=float(xa_entry.get()), ya=float(ya_entry.get()),
                        xb=float(xb_entry.get()), yb=float(yb_entry.get()),
                        K=float(K_entry.get()), R=float(R_entry.get()),
                        security=sec, slowing_R=float(slowing_R_entry.get()),
                        slowing_K=float(slowing_K_entry.get()),
                        effect_range=float(effect_range_entry.get()))


def send_field(event=None):
    # send behavior
    info = getInfos()
    confirmation = behavior_sender(info)
    # Increment the ID
    prev = int(id_value.get())
    nexT = '{:0>3}'.format(prev + 1)
    id_value.set(nexT)
    # Add the sent behavior to the history list
    print info.toString()
    history_list.insert(END, info.toString())
    print confirmation


def remove_all(event=None):
    for el in history_list.get(0, END):
        b = BehaviorInfo(behavior_id=el.split(
            ' ')[0], f_type=el.split(' ')[1][1:-1])
        confirmation = behavior_sender(b)
        print 'Removing', el
        print confirmation
    history_list.delete(0, END)
    print '-' * 10
    print 'Removed all !'


def remove_selected(event=None):
    if len(history_list.curselection()) != 0:
        for el in [history_list.get(k) for k in history_list.curselection()]:
            b = BehaviorInfo(behavior_id=el.split(
                ' ')[0], f_type=el.split(' ')[1][1:-1])
            confirmation = behavior_sender(b)
            print 'Removing', el,
            print confirmation
        history_list.delete(*history_list.curselection())


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
obstacle_point_radio_button = Radiobutton(
    type_frame, text='Obstacle point',
    variable=type_choice, value='obst_point')

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
                   increment=1)

ya_label = Label(parameter_frame, text='ya')
ya_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=1)

xb_label = Label(parameter_frame, text='xb')
xb_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=1)

yb_label = Label(parameter_frame, text='yb')
yb_entry = Spinbox(parameter_frame, from_=-999, to=999,
                   increment=1)

R_label = Label(parameter_frame, text='R')
R_entry = Spinbox(parameter_frame, from_=0, to=50,
                  increment=1)

K_label = Label(parameter_frame, text='K')
K_entry = Spinbox(parameter_frame, from_=0, to=20,
                  increment=0.1, format='%.1f')

slowing_R_label = Label(parameter_frame, text='slowing_R')
slowing_R_entry = Spinbox(parameter_frame, from_=0, to=20,
                          increment=0.1, format='%.1f')

slowing_K_label = Label(parameter_frame, text='slowing_K')
slowing_K_entry = Spinbox(parameter_frame, from_=0, to=50,
                          increment=0.5)

effect_range_label = Label(parameter_frame, text='effect_range')
effect_range_entry = Spinbox(parameter_frame, from_=0, to=999,
                             increment=1)

security_label = Label(parameter_frame, text='security')
security_entry = Spinbox(parameter_frame, from_=0, to=2,
                         increment=1)


# *******************************************
# Section parametres du champ
history_frame = Frame(fen, borderwidth=2)
history_frame_title_label = Label(history_frame, text='History')
# List des behaviors envoyes
history_list = Listbox(history_frame, selectmode=MULTIPLE)
# Button de suppression des behaviors envoyes
history_delete_selected_button = Button(
    history_frame, text='Remove', command=remove_selected)
history_delete_all_button = Button(
    history_frame, text='Clear', command=remove_all)


# *******************************************
# Button de validation
validate_button = Button(fen, text='Envoyer', command=send_field)

# *******************************************
# *******************************************
# Packing and griding

type_frame.grid(row=1, column=1)
type_title_label.grid(row=1, column=1)
waypoint_radio_button.grid(row=2, column=1)
ligne_radio_button.grid(row=2, column=2)
patrol_radio_button.grid(row=2, column=3)
limite_radio_button.grid(row=2, column=4)
obstacle_point_radio_button.grid(row=3, column=1)


parameter_frame.grid(row=2, column=1)
parameter_title_label.grid(row=1, column=1)
id_label.grid(row=2, column=1)
id_entry.grid(row=2, column=2)
xa_label.grid(row=3, column=1)
xa_entry.grid(row=3, column=2)
xb_label.grid(row=3, column=3)
xb_entry.grid(row=3, column=4)
ya_label.grid(row=4, column=1)
ya_entry.grid(row=4, column=2)
yb_label.grid(row=4, column=3)
yb_entry.grid(row=4, column=4)
R_label.grid(row=5, column=1)
R_entry.grid(row=5, column=2)
K_label.grid(row=5, column=3)
K_entry.grid(row=5, column=4)
slowing_R_label.grid(row=6, column=1)
slowing_R_entry.grid(row=6, column=2)
slowing_K_label.grid(row=6, column=3)
slowing_K_entry.grid(row=6, column=4)
effect_range_label.grid(row=7, column=1)
effect_range_entry.grid(row=7, column=2)
security_label.grid(row=7, column=3)
security_entry.grid(row=7, column=4)

validate_button.grid(row=3, column=1)

history_frame.grid(row=1, column=2, rowspan=3)
history_frame_title_label.grid(row=1, column=1, columnspan=2)
history_list.grid(row=2, column=1, columnspan=2)
history_delete_all_button.grid(row=3, column=1)
history_delete_selected_button.grid(row=3, column=2)

default_widget_state()

# Loop
rospy.init_node('gui_behavior_adder')
rospy.wait_for_service('behavior_manager')
behavior_sender = rospy.ServiceProxy('behavior_manager', behavior)


def check_ros():
    if rospy.is_shutdown():
        fen.destroy()
    else:
        fen.after(500, check_ros)
fen.after(1, check_ros)
fen.mainloop()
