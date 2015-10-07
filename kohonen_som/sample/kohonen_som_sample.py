#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import numpy as np
import matplotlib.pyplot as plt

import rospy
from kohonen_som.srv import Clustering, ClusteringRequest

if __name__ == '__main__':
    N = 20
    np.random.seed(100)
    unit = np.random.rand(N, N, 3)
    plt.imshow(src_data, interpolation="none")
    # plt.savefig("source.png")
    
