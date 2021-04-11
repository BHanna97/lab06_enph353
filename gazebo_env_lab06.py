
import cv2
import gym
import math
import rospy
import roslaunch
import time
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from time import sleep

from gym.utils import seeding
import QLearn

class Gazebo_Lab06_Env(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        LAUNCH_FILE = '/home/fizzer/enph353_gym-gazebo/gym_gazebo/envs/ros_ws/src/enph353_lab06/launch/lab06_world.launch'
        gazebo_env.GazeboEnv.__init__(self, LAUNCH_FILE)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world',
                                              Empty)

        self.action_space = spaces.Discrete(3)  # F,L,R
        self.reward_range = (-np.inf, np.inf)
        self.episode_history = []

        self._seed()

        self.bridge = CvBridge()
        self.timeout = 0  # Used to keep track of images with no line detected
        self.illegal_actions = 0 #keep track of repeat actions
        self.lower_blue = np.array([97,  0,   0])
        self.upper_blue = np.array([150, 255, 255])
        

    def process_image(self, data):
        '''
            @brief Coverts data into a opencv image and displays it
            @param data : Image data from ROS

            @retval (state, done)
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("raw", cv_image)
        state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # state = QLearn.loadQ(self.filename)
        done = False
        #do I only want to consider the lower part?
        #need to double check if this should be min or max
        #how to test this??
        #need a better threshold
        
        gray_arr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        bottom_arr = gray_arr[-200:, :]
     
        avg = bottom_arr.mean(axis = 0).T
        
        max_index = np.argmin(avg)
        pos = float(max_index)/float(bottom_arr.shape[1])
        index = int(np.floor(pos*10))
        state[index] = 1
        image = cv2.putText(gray_arr, str(state), (10, 75), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.5, (255, 255, 255), 1, cv2.LINE_AA)
        image = cv2.putText(gray_arr, str(pos), (10, 25), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.5, (255, 255, 255), 1, cv2.LINE_AA) 
        image = cv2.putText(gray_arr, str(np.amin(avg)), (10,50), cv2.FONT_HERSHEY_SIMPLEX,  
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Array", gray_arr)
        cv2.waitKey(3)

        if np.amin(avg) >= 170:
            self.timeout += 1
        else:
            self.timeout = 0
        if self.timeout > 30:
            done = True
        if self.illegal_actions > 20:
            done = True
        
        # length = np.arange(1, bottom_arr.shape[1] + 1)
        # print(np.multiply(length, avg).shape)
        # print(length.shape)
        # print(avg.shape)
        # center_of_mass = np.divide(np.multiply(length, avg), length)
        #print(np.divide(center_of_mass, length))
        #find center of mass
        #if in between then assign either uppper or lower
        #com/width
        
            
        # if self.timeout == 30:
        #     done = True
        # elif np.all(state_arr == 0):
        #     self.timeout += 1
        # else:
        #     self.timeout = 0

        #     state = state_arr.tolist()
            
        
        # TODO: Analyze the cv_image and compute the state array and
        # episode termination condition.
        #
        # The state array is a list of 10 elements indicating where in the
        # image the line is:
        # i.e.
        #    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0] indicates line is on the left
        #    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0] indicates line is in the center
        #
        # The episode termination condition should be triggered when the line
        # is not detected for more than 30 frames. In this case set the done
        # variable to True.
        #
        # You can use the self.timeout variable to keep track of which frames
        # have no line detected.

        return state, done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.episode_history.append(action)

        vel_cmd = Twist()

        if action == 0:  # FORWARD
            vel_cmd.linear.x = 0.4
            vel_cmd.angular.z = 0.0
            self.illegal_actions = 0
        elif action == 1:  # LEFT
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.5
            self.illegal_actions += 1
        elif action == 2:  # RIGHT
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = -0.5
            self.illegal_actions += 1

        self.vel_pub.publish(vel_cmd)
        
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/pi_camera/image_raw', Image,
                                              timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, done = self.process_image(data)

        # Set the rewards for your action
        if not done:
            if action == 0:  # FORWARD
                reward = 4
            elif action == 1:  # LEFT
                reward = 2
            else:
                reward = 2  # RIGHT
        else:
            reward = -200

        return state, reward, done, {}

    def reset(self):

        print("Episode history: {}".format(self.episode_history))
        self.episode_history = []
        print("Resetting simulation...")
        # Resets the state of the environment and returns an initial
        # observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # read image data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/pi_camera/image_raw',
                                              Image, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        self.timeout = 0
        state, done = self.process_image(data)

        return state
    
    