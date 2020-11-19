
# -------------------------------------------------------------------[ header ]

import numpy as np
import local_view_match as lv
import posecellnetwork  as pc
import visual_odometry  as vo
import experience_map   as em
from _globals import *

# ------------------------------------------------------------[ RatSLAM class ]

class Ratslam(object):
    def __init__(self):
        '''
        visual-odometry: object of VisualOdometry class that controls odometry
        information
        visual-templates: object of LocalViewMatch class that controls the
        matching scenes
        network: object of PoseCellNetwork class that controls activity in the
        pose cell network and the actions in the experience map
        map: object of ExperienceMap class
        vtrans: translational velocity of the robot
        vrot: rotational velocity of the robot
        prev-vt: id of the previous visual template
        time-diff: the time difference between frames

        # not used
        frame_count: the number of frames that have been
        time_s: the time difference between frames
        '''
        self.visual_odometry  = vo.VisualOdometry()
        self.visual_templates = lv.LocalViewMatch()
        self.network          = pc.PoseCellNetwork()
        self.map              = em.ExperienceMap()

        self.vtrans      = 0
        self.vrot        = np.pi/2.0
        self.prev_vt     = 0
        self.frame_count = 0
        self.time_s      = 0
        self.time_diff   = 1

        x, y, th = self.visual_odometry.odometry
        self.odometry = [[x], [y], [th]]



    def __call__(self, img, greyscale):
        '''
        Purpose: This routine updates the current position of the experience
        map since the last experience.

        Algorithm: First, calculates the translational velocity and rotational
        velocity between the current frame (data) and the previous frame.
        Second, finds the matching template and the angle between the current
        and previous template. Third, determines an action for the experience
        map's topological graph. Finally, updates the current position of the
        experience map since the last experience, performs the action from pose
        cell network and graph relaxation.

        Inputs:
            img: current frame
            greyscale: True if the current frame is on grayscale, False if
            the current frame is not on grayscale

        Outputs: -
        '''
        # first: calculate the translational velocity and rotational velocity
        # between the current frame (data) and the previous frame
        vtrans, vrot           = self.visual_odometry( img, greyscale )



        # second: find the matching template and the angle between the current
        # and previous template
        current_vt, vt_rad     = self.visual_templates( img, greyscale )

        x, y, th = self.visual_odometry.odometry
        self.odometry[0].append(x)
        self.odometry[1].append(y)
        self.odometry[2].append(th)

        # third: determine an action for the experience map's topological graph
        action, matched_exp_id = self.network( current_vt, vt_rad, vtrans, vrot, self.time_diff )

        # finally: update the current position of the experience map since the
        # last experience, perform the action from pose cell network and graph
        # relaxation
        self.map.on_odo( vtrans, vrot, self.time_diff )

        if action == CREATE_NODE: # create new experience node
            self.map.on_create_experience( matched_exp_id, current_vt )
            self.map.on_set_experience( matched_exp_id, 0 )
        if action == CREATE_EDGE: # link together previous exp node w/ current one
            self.map.on_create_link( self.map.current_exp_id, matched_exp_id, self.network.get_relative_rad() )
            self.map.on_set_experience( matched_exp_id, self.network.get_relative_rad() )
        if action == SET_NODE: # matched exp is the current exp; no link & no new exp node
            self.map.on_set_experience( matched_exp_id, self.network.get_relative_rad() )

        self.map.iterate()

        """
        # interface
        # get where the robot is and where it can go
        Later:
        current_exp, exps_goals = self.map.get_status()
        id = raw_input('digite qual exp')
        self.map.get_distance_and_direction(int(id))
        """

        """
        Later:
        self.map.calculate_path_to_goal(self.time_s)
        self.map.get_goal_waypoint()
        """

    def save(self):
        '''
        Purpose: This routine saves all the visual template, pose cell and
        experience map information..

        Algorithm: Creates files for each object and stores all the information.

        Inputs: -

        Outputs: -
        '''
        self.network.save()
        self.visual_templates.save()
        self.map.save()

    def load(self):
        '''
        Purpose: This routine loads all the visual template, pose cell and
        experience map information

        Algorithm: Opens files and loads all the information.

        Inputs: -

        Outputs: -
        '''
        self.visual_templates.load()
        self.network.load()
        self.map.load()

    def prune(self):
        #TODO
        deleted_exps = self.map.prune(self.network.visual_templates.size)
        if deleted_exps.size > 1:
            self.network.prune(deleted_exps)
