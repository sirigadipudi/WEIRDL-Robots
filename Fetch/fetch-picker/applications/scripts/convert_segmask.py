import cv2
import numpy as np

mask=cv2.imread("/home/henri/Downloads/segg_image.png", cv2.IMREAD_GRAYSCALE)
mask[mask!=255] = 0
mask = cv2.resize(mask, (300, 380), interpolation = cv2.INTER_AREA)
cv2.imshow("mask", mask)
cv2.waitKey(0)
print(np.unique(mask))
cv2.imwrite("mask.png", mask)
