import cv2
import random
import numpy as np

## Readme ##

#Press any key to generate a new map
#Select obstacle_density from trackbar in output window
#Press Esc key to Quit
#Select Map Size and Scaling Factor below:
size=(16,16)
scaling_factor=32

def nothing(x):
    pass
class Viewer(object):
    cv2.namedWindow('window')

    # create trackbars for obstacle_density
    cv2.createTrackbar('ObsDen','window',20,100,nothing)

    while True:
        obstacle_density = (cv2.getTrackbarPos('ObsDen','window'))/100

        print(obstacle_density)

        img=np.ones(size,dtype=np.uint8)
        img=img*255

        x=random.sample(range(size[0]*size[1]),int(size[0]*size[1]*obstacle_density))

        for i in x:
            img[int(i/size[1]),i%size[1]]=0;

        res=np.zeros((size[0]*scaling_factor,size[1]*scaling_factor),dtype=np.uint8)

        #res = cv2.resize(img, (512, 512))

        for i in range(0,size[0]*scaling_factor,scaling_factor):
            for j in range(0,size[1]*scaling_factor,scaling_factor):
                res[i:i+scaling_factor,j:j+scaling_factor]=img[int(i/scaling_factor),int(j/scaling_factor)];

        cv2.imshow('window',res)

        k=cv2.waitKey(0) & 0xFF
    
        if k == 27:
            break
    
    cv2.destroyAllWindows()
