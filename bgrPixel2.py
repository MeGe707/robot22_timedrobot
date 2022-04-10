
import cv2
import numpy

cymurghslogo = cv2.imread("cymurghs.jpg")

cymurghslogo[50, 30] = [0, 0, 255]

for i in range(100):
    cymurghslogo[70, i] = [0, 0, 255]


cv2.imshow("cymurghs", cymurghslogo)

cv2.waitKey(0)
cv2.destroyAllWindows()
