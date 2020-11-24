'''

RatSLAM usage example.

'''
from __future__ import division # Change Python 3 - Paulo    
import cv2
import numpy as np
import sys

# Importing ratslam modules 
from MAP_RatSlamModule import ratslam3 # Change Python 3 - Paulo    
from MAP_RatSlamModule import _globals3 as gb # Change Python 3 - Paulo 

import matplotlib
from mpl_toolkits.mplot3d import Axes3D
matplotlib.use('TKAgg')
from matplotlib import pyplot as plot
import time
from matplotlib import colors as mcolors

# def plotAndSaveResult(frame, slam):
def plotAndSaveResult(slam):

    # =========================================================
    # PLOT THE CURRENT RESULTS ================================
    #b, g, r   = cv2.split( frame )
    #rgb_frame = cv2.merge( [r, g, b] )

    plot.clf()

    # RAW IMAGE -------------------
    '''
    ax = plot.subplot( 2, 2, 1 )
    plot.title( 'Image' )
    plot.imshow( rgb_frame, interpolation='nearest', animated=True, label='blah' )
    ax.get_xaxis().set_ticks([])
    ax.get_yaxis().set_ticks([])
    '''
    # -----------------------------
    # RAW ODOMETRY ----------------
    '''
    plot.subplot(1, 2, 1)
    plot.title('RAW ODOMETRY')
    plot.plot(slam.odometry[0], slam.odometry[1])
    plot.plot(slam.odometry[0][-1], slam.odometry[1][-1], 'ko')
    '''

    # -----------------------------
    # POSECELL --------------
    '''
    ax = plot.subplot(2, 2, 3, projection='3d')
    #fig = plot.figure()
    #ax = fig.add_subplot(111, projection='3d')
    plot.title('Pose cell network')

    xp.append(slam.network.best_x)
    yp.append(slam.network.best_y)
    thp.append(slam.network.best_th)

    ax.scatter(xp, yp, thp, color = 'b')

    plot.scatter(slam.network.best_x, slam.network.best_y, slam.network.best_th, 'y', '.')

    ax.set_xlim(0, 80)
    ax.set_ylim(0, 80)
    ax.set_zlim(0, 36)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    '''
    # -----------------------------
    # EXPERIENCE MAP --------------

    #ax = plot.subplot(1, 2, 2)

    plot.title('experience map')
    xs = []
    ys = []
    for exp in slam.map.experiences:
        xs.append(exp.x_m)
        ys.append(exp.y_m)
    

    plot.plot(xs, ys, color='navy', ls='-')
    # plot.scatter(xs[0], ys[0], s=300, color='navy')
    # plot.scatter(xs[1:len(xs)-1], ys[1:len(ys)-1], color='navy', marker='o')


    # plot.scatter(slam.map.experiences[int(slam.map.current_exp_id)].x_m,
                # slam.map.experiences[int(slam.map.current_exp_id)].y_m, s=300, color='w', alpha=1, edgecolors='navy',linewidths=2)


    #ax.get_xaxis().set_ticks([])
    #ax.get_yaxis().set_ticks([])
    plot.xlabel("x")
    plot.ylabel("y")

    # -----------------------------
    #plot.savefig(r'../RatSLAM_CNS/image/result')

    # -----------------------------
    # saved_name_eps = saved_name + '.eps'
    plot.savefig('map_irace_test', format='eps', dpi=1000)

    # saved_name_png = saved_name + '.png'
    plot.savefig('map_irace_test')


    plot.tight_layout()

    #cv2.waitKey(25)
    #plot.pause(0.1)

def plotResult(frame, slam):

    # =========================================================
    # PLOT THE CURRENT RESULTS ================================
    b, g, r   = cv2.split( frame )
    rgb_frame = cv2.merge( [r, g, b] )

    plot.clf()

    # RAW IMAGE -------------------
    
    ax = plot.subplot( 2, 2, 1 )
    plot.title( 'Image' )
    plot.imshow( rgb_frame, interpolation='nearest', animated=True, label='blah' )
    ax.get_xaxis().set_ticks([])
    ax.get_yaxis().set_ticks([])

    # RAW ODOMETRY ----------------
    
    # plot.subplot(1, 2, 2)

    plot.subplot(2, 2, 2)
    plot.title('RAW ODOMETRY')
    plot.plot(slam.odometry[0], slam.odometry[1])
    plot.plot(slam.odometry[0][-1], slam.odometry[1][-1], 'ko')
    
    #------------------------------

    # POSECELL --------------
    
    ax = plot.subplot(2, 2, 3, projection='3d')
    #fig = plot.figure()
    #ax = fig.add_subplot(111, projection='3d')
    plot.title('Pose cell network')

    # xp.append(slam.network.best_x)
    # yp.append(slam.network.best_y)
    # thp.append(slam.network.best_th)

    # ax.scatter(xp, yp, thp, color = 'b')

    ax.scatter(slam.network.best_x, slam.network.best_y, slam.network.best_th, 'g')

    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_zlim(0, 36)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    

    # -----------------------------
    # EXPERIENCE MAP --------------

    # ax = plot.subplot(1, 2, 1)

    ax = plot.subplot(2, 2, 4)
    plot.title('experience map')
    xs = []
    ys = []
    for exp in slam.map.experiences:
        xs.append(exp.x_m)
        ys.append(exp.y_m)

    plot.plot(xs, ys, color='navy', ls='-')
    plot.scatter(xs[0], ys[0], s=300, color='navy')
    plot.scatter(xs[1:len(xs)-1], ys[1:len(ys)-1], color='navy', marker='o')


    plot.scatter(slam.map.experiences[int(slam.map.current_exp_id)].x_m,
                slam.map.experiences[int(slam.map.current_exp_id)].y_m, s=300, color='w', alpha=1, edgecolors='navy',linewidths=2)

    ax.get_xaxis().set_ticks([])
    ax.get_yaxis().set_ticks([])
    plot.xlabel("x")
    plot.ylabel("y")

    # -----------------------------

    #plot.savefig(r'../RatSLAM_CNS/image/result')

    plot.tight_layout()

    plot.pause(0.1)

# if __name__ == '__main__':
def main3(frame):

    # data = sys.argv[1] # LACMOR_small_2voltas_mod.mp4 and outputWithSkybox.avi
    # data = r'./LACMOR_small_2voltas_mod.mp4'
    # video = cv2.VideoCapture(data)
    cv2.waitKey(1000)

    # RatSLAM new object 
    slam = ratslam3.Ratslam()
    
    loop = 0
    # _, frame = video.read()

    while True:
        
        loop += 1

        # flag,frame = video.read()
        if frame is None:
            break

        #width  = video.get( cv2.cv.CV_CAP_PROP_FRAME_WIDTH  ) # Change Paulo
        # width  = video.get( cv2.CAP_PROP_FRAME_WIDTH ) 
        # height = video.get( cv2.CAP_PROP_FRAME_HEIGHT )

        width  = frame.shape[1]
        height = frame.shape[0]


        # Changing values in _globals variables as example. The globals file
        # set variables from pose cells, local view cells, visual odometry...  
        gb.IMAGE_WIDTH = width
        gb.IMAGE_HEIGHT = height

        img = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
        img = np.array( img )

        if loop%7==0:
            
            # The True setting is refers to pass a GRAY image. 
            slam(img, True)

            plotResult(frame, slam)

            # Getting all elements of experiences list. The poses of experience_map may change with map correction, 
            # so it's better getting all updated poses.
            # poses = []
            # for exp in slam.map.experiences:
            #     poses.append([exp.x_m, exp.y_m, exp.th_rad])
            #     print (poses[-1]) # Change Python 3 - Paulo    

            '''
            # Getting only the last element of experiences list. Not recommended because the previous poses may change
            # with map correction 
            exp = slam.map.experiences
            x, y, theta = exp[-1].x_m, exp[-1].y_m, exp[-1].th_rad 
            print x,y,theta
            '''

    # slam.save()
