ros_nmpt_saliency
=================

ros catkin package for saliency tracking using nmpt

use ros_nmpt_saliency_node

This package listens to camera/image_raw topic and analyzes the video for saliency. It gives a point output 0..1 for x, 0..1 for y to specify where in the image the saliency point lies by publishing it. 
