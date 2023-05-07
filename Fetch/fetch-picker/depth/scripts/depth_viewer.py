import numpy as np
import matplotlib.pyplot as plt

# load the depth image
# image = np.load("/home/henri/catkin_fetch_ws/src/gqcnn/data/examples/clutter/phoxi/fcgqcnn/depth_0.npy")
image = np.load("/home/henri/catkin_fetch_ws/src/fetch-picker/depth/scripts/depth_array1.npy")
# image = np.load("/home/henri/catkin_fetch_ws/src/gqcnn/data/examples/clutter/phoxi/dex-net_4.0/depth_0.npy")

image = np.squeeze(image)

# display the image using matplotlib's imshow function
plt.imshow(image, cmap='gray')

# show the plot
plt.show()