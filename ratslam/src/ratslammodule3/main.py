'''

RatSLAM usage example.

'''

import cv2
import numpy as np

# Importing ratslam modules 
from MAP_RatSlamModule import ratslam
from MAP_RatSlamModule import _globals as gb

if __name__ == '__main__':

    #data = r'./outputWithSkybox.avi'
    data = r'./LACMOR_small_2voltas_mod.mp4'
    video = cv2.VideoCapture(data)
    cv2.waitKey(1000)

    # RatSLAM new object 
    slam = ratslam.Ratslam()
    
    loop = 0
    _, frame = video.read()

    while True:
        
        loop += 1

        flag,frame = video.read()
        if frame is None:
            break

        #width  = video.get( cv2.cv.CV_CAP_PROP_FRAME_WIDTH  ) # Paulo
        width  = video.get( cv2.CAP_PROP_FRAME_WIDTH ) 
        height = video.get( cv2.CAP_PROP_FRAME_HEIGHT )


        # Changing values in _globals variables as example. The globals file
        # set variables from pose cells, local view cells, visual odometry...  
        gb.IMAGE_WIDTH = width
        gb.IMAGE_HEIGHT = height

        img = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
        img = np.array( img )

        if loop%7==0:
            
            # The True setting is refers to pass a GRAY image. 
            slam(img, True)

            # Getting all elements of experiences list. The poses of experience_map may change with map correction, 
            # so it's better getting all updated poses.
            poses = []
            for exp in slam.map.experiences:
                poses.append([exp.x_m, exp.y_m, exp.th_rad])
            #print poses    

            '''
            # Getting only the last element of experiences list. Not recommended because the previous poses may change
            # with map correction 
            exp = slam.map.experiences
            x, y, theta = exp[-1].x_m, exp[-1].y_m, exp[-1].th_rad 
            print x,y,theta
            '''
            

    slam.save()
