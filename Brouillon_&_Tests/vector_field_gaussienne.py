import matplotlib.pyplot as plt
import numpy as np
from random import randrange
import cv2


D = np.zeros((50, 50))


# Ajout d'obstacles
for o in range(30):
    i, j = randrange(50), randrange(50)
    D[i, j] = randrange(3)

kernel = np.ones((3, 3), np.float32) / 25
B = cv2.filter2D(D, -1, kernel)

U, V = np.gradient(-D)
U1, V1 = np.gradient(-B)

# plt.hold(True)
plt.figure(1)
plt.imshow(D, interpolation='none', cmap='PuBu')
plt.quiver(V, -U, color='red')
plt.figure(2)
plt.imshow(B, interpolation='none', cmap='Greys')
plt.quiver(V1, -U1, color='red')
# plt.imshow(D, cmap='Greys')
plt.show()
