#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np


class VectorField():
    def __init__(self, multi_array=None, numpy_array=None):

        if multi_array is None and numpy_array is None:
            self.multi_array = Float64MultiArray()
            self.numpy_array = np.array()

        elif numpy_array is None and type(multi_array) is Float64MultiArray:
            self.numpy_array = self.np_from_multi_array(multi_array)

        elif multi_array is None and type(numpy_array) is np.ndarray:
            self.multi_array = self.multi_array_from_np(numpy_array)


    def get(self, i, j, k):
        return self.multi_array.data[self.offset + self.width_dimension.stride * i + self.depth_dimension.stride * j + k]

    # def __init__(self, height, width, depth):
    #     self.height = height
    #     self.width = width
    #     self.depth = depth

    #     # Heigth
    #     self.height_dimension = MultiArrayDimension()
    #     self.height_dimension.label = 'Height'
    #     self.height_dimension.size = self.height
    #     self.height_dimension.stride = self.depth * self.width * self.depth

    #     # Width
    #     self.width_dimension = MultiArrayDimension()
    #     self.width_dimension.label = 'Width'
    #     self.width_dimension.size = self.width
    #     self.width_dimension.stride = self.depth * self.width

    #     # Depth
    #     self.depth_dimension = MultiArrayDimension()
    #     self.depth_dimension.label = 'Depth'
    #     self.depth_dimension.size = self.depth
    #     self.depth_dimension.stride = self.depth

    #     # MultiArrayLayout
    #     self.layout = [
    #         self.height_dimension, self.width_dimension, self.depth_dimension]
    #     self.offset = 0

    #     # Data
    #     self.data = [(i+1, k+1) for i in range(self.height)
    #                  for k in range(self.width)]

    #     # Float64MultiArray
    #     self.array = Float64MultiArray()
    #     self.array.layout = self.layout
    #     self.array.data = self.data



if __name__ == '__main__':
    # vf = VectorField(3, 4, 2)
    # print vf.get(0, 1, 0)
    vf = VectorField()
