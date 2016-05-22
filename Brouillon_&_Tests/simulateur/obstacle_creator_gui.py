#!usr/bin/env python
from Tkinter import *
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from cv2 import filter2D


def update_vector_field():
    global grid, U, V
    if gaussian_interpolation.get():
        kernel = np.ones((3, 3), np.float32) / 25
        B = filter2D(grid, -1, kernel)
        U, V = np.gradient(-B)
    else:
        U, V = np.gradient(-grid)
    # print U, V
    ve.set_UVC(V, -U)
    fig.canvas.draw()


def onclick(event):
    global grid, im
    if event.button == 1:
        grid[round(event.ydata), round(event.xdata)] += 1
    elif event.button == 3:
        grid[round(event.ydata), round(event.xdata)] = reduce_to_zero(
            grid[round(event.ydata), round(event.xdata)])
        # if grid[round(event.ydata), round(event.xdata)] > 0:
        #     grid[round(event.ydata), round(event.xdata)] -= 1
    # grid[event.ydata, event.xdata] %= 2
    update_infos()
    fen.after(5000, lambda: low(
        round(event.ydata), round(event.xdata)))
    # print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
    #       (event.button, event.x, event.y, event.xdata, event.ydata))


def reset():
    global grid
    grid = np.zeros((SIZE, SIZE))
    update_infos()
    fig.canvas.draw()


def update_interpolation():
    global gaussian_interpolation, im
    im.set_data(grid)
    print 'change', gaussian_interpolation.get()
    if gaussian_interpolation.get():
        im.set_interpolation('gaussian')
    else:
        im.set_interpolation('none')
    update_vector_field()
    fig_can.draw()


def test():
    global grid
    grid = reduce_to_zero_m(grid)
    update_infos()
    fig_can.draw()


def reduce_to_zero(x):
    if x > 0:
        x -= 1
    return x

reduce_to_zero_m = np.vectorize(reduce_to_zero)


def low(l, c):
    global grid
    grid[l, c] = reduce_to_zero(grid[l, c])
    update_infos()
    fig_can.draw()


def update_infos():
    im.set_data(grid)
    info_max['text'] = 'Max: %d' % np.amax(grid)
    info_min['text'] = 'Min: %d' % np.amin(grid)
    im.set_clim(np.amin(grid), np.amax(grid))
    update_vector_field()


# window
fen = Tk()
fen.title('Hello')
# create the grid
SIZE = 40
grid = np.zeros((SIZE, SIZE))
# grid[4, 4] = 3
U, V = np.gradient(-grid)
fig = plt.figure('hehehe')
im = plt.imshow(grid, cmap='Greys', interpolation='none')
# plt.hold(True)
ve = plt.quiver(-V, U, color='red', scale=2)

# Canvas
fig_can = FigureCanvasTkAgg(fig, master=fen)
fig_can.mpl_connect('button_press_event', onclick)
fig_can.get_tk_widget().grid(row=0, column=0, rowspan=3)

# Label
info_max = Label(fen, text='Max: 0')
info_max.grid(row=0, column=1)
info_min = Label(fen, text='Min: 0 ')
info_min.grid(row=1, column=1)
# Tick
gaussian_interpolation = IntVar()
check_interpolation = Checkbutton(
    fen, text='Interpolation', variable=gaussian_interpolation,
    command=update_interpolation)
check_interpolation.grid(row=2, column=1)

# Button
reset_button = Button(fen, text='Reset', command=reset)
reset_button.grid(row=3, column=0)

test_button = Button(fen, text='Reduce', command=test)
test_button.grid(row=3, column=1)

fen.mainloop()
