#!/usr/bin/python3

import rospy
import tf
import copy
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from fp_step1.srv import GetNextDestination, GetNextDestinationRequest


class VFHController():

    def __init__(self):
        
        rospy.init_node('VFH_controller', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # VFH constants
        self.SECTOR_DEGREE = 5
        self.a = 1
        self.b = 0.25
        self.l = 2
        self.S_MAX = int(60 / 5)
        self.VALLEY_MIN_WIDTH = int(20 / 5)
        self.HISTOGRAM_THRESHOLD = 0.75

        # controlling constants (a P-controller works just fine)
        self.k_p = 1
        self.v = 0.3

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # goal definition
        self.goal_x = 0
        self.goal_y = 0
        self.REACH_THRESHOLD = 0.2

        # robot proccessing rate
        self.dt = 0.005
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)

        # defining the states of the robot
        self.FORWARD, self.TURN, self.FOLLOW = 0, 1, 2
        self.state = self.FORWARD

    
    def sector_to_degree(self, n):
        if not n:
            return None
        
        return (n * self.SECTOR_DEGREE) % 360

    def degree_to_sector(self, heading, n):
        rp = n - heading
        if rp < 0:
            rp += 360

        return int(rp / self.SECTOR_DEGREE)


    # Odometry Methods #####################################################

    def get_heading(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        )) 
        
        yaw = math.degrees(yaw)
        if yaw < 0:
            yaw += 360
        
        return yaw
    

    def get_position(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        return x, y
    
    def get_goal_angle(self, x, y):
        goal_angle = math.degrees(math.atan((self.goal_y - y) / (self.goal_x - x)))

        if self.goal_x < x:
            goal_angle = 180 + goal_angle
        
        if goal_angle < 0:
            goal_angle += 360

        return goal_angle


    # Histogram Methods ####################################################

    def get_polar_histogram(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan).ranges

        histogram = np.empty([int(len(laser_data) / self.SECTOR_DEGREE)])
        for i in range(len(histogram)):
            sum = 0
            for j in range(self.SECTOR_DEGREE):
                c = 0 if laser_data[i * self.SECTOR_DEGREE + j] > 5 else 1
                sum += c * max(self.a - self.b * laser_data[i * self.SECTOR_DEGREE + j], 0)
            histogram[i] = sum

        histogram = self.smoothen_histogram(histogram)
        # plt.subplot(2, 1, 1)
        # plt.bar(np.arange(len(histogram)), histogram)
        
        histogram = self.to_binary_array(histogram)
        # plt.subplot(2, 1, 2)
        # plt.bar(np.arange(len(histogram)), histogram)
        # plt.show()
        
        return histogram
    

    def smoothen_histogram(self, histogram):
        res = np.empty([len(histogram)])
        for i in range(len(histogram)):
            sum = 0
            
            for j in range(self.l + 1):
                if j == 0:
                    sum += self.l * histogram[i]
                else:
                    sum += (self.l - j + 1) * histogram[(i + j) % len(histogram)]
                    sum += (self.l - j + 1) * histogram[i - j]
            
            res[i] = sum / (2 * self.l + 1)
        
        return res


    def adjacent_zeros_count(self, binary_histogram, indx):
        zeros_count = 0

        for j in range(self.S_MAX):
            i = (indx + j) % len(binary_histogram)
            if binary_histogram[i] == 1:
                break
            zeros_count += (1 - binary_histogram[i])

        for j in range(-1, -self.S_MAX, -1):
            if binary_histogram[indx + j] == 1:
                break
            zeros_count += (1 - binary_histogram[indx + j])

        return zeros_count
    

    def to_binary_array(self, histogram):
        threshold = self.HISTOGRAM_THRESHOLD * max(histogram)
        
        binary_histogram = np.where(histogram <= threshold, 0, 1)
        
        res = copy.deepcopy(binary_histogram)
        for i in range(len(binary_histogram)):
            if binary_histogram[i] == 1 or (90 / self.SECTOR_DEGREE) < i < (270 / self.SECTOR_DEGREE):
                res[i] = 1
                continue

            if self.adjacent_zeros_count(binary_histogram, i) < self.VALLEY_MIN_WIDTH:
                res[i] = 1
                
        return res
    

    def find_direction(self, k_targ, binary_histogram):
        if not np.any(binary_histogram == 0):
            return None

        left_index = k_targ
        right_index = k_targ

        while right_index == k_targ or left_index != right_index:
            if binary_histogram[left_index] == 0:
                left_index = left_index if left_index >= 0 else left_index + len(binary_histogram)
                
                valley_size = self.adjacent_zeros_count(binary_histogram, left_index)
                rospy.loginfo(f"Valley size: {valley_size}")
                
                k_n = left_index
                k_f = (left_index - self.S_MAX) + 1 if valley_size >= self.S_MAX \
                    else (left_index - valley_size) + 1

                rospy.loginfo(f"left valley - k_n: {k_n}, k_f: {k_f}")
                rospy.loginfo(f"decision: {((k_n + k_f) / 2) % len(binary_histogram)}")

                return ((k_n + k_f) / 2) % len(binary_histogram)
            
            elif binary_histogram[right_index] == 0:
                valley_size = self.adjacent_zeros_count(binary_histogram, right_index)
                
                k_n = right_index
                k_f = right_index + self.S_MAX - 1 if valley_size >= self.S_MAX \
                    else right_index + valley_size - 1
                
                rospy.loginfo(f"right valley - k_n: {k_n}, k_f: {k_f}")
                rospy.loginfo(f"decision: {((k_n + k_f) / 2) % len(binary_histogram)}")
                
                return ((k_n + k_f) / 2) % len(binary_histogram)
            
            else:
                left_index -= 1
                right_index = (right_index + 1) % len(binary_histogram)

    
    # Navigation Methods ###################################################

    def request_next_goal(self):
        try:
            server_call = rospy.ServiceProxy('GetNextDestination', GetNextDestination)

            response = server_call(GetNextDestinationRequest())
            self.goal_x = response.next_x
            self.goal_y = response.next_y

            return response.last_goal

        except rospy.ServiceException as error:
            rospy.logerr(f"Mission service call failed: {error}")
    
    
    def dist(self, x1, y1, x2, y2):
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    

    def follow_goal(self):
        
        move_cmd = Twist()
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():

            self.cmd_vel.publish(move_cmd)

            x, y = self.get_position()
            heading = self.get_heading()

            if self.dist(x, y, self.goal_x,  self.goal_y) < self.REACH_THRESHOLD:
                if self.request_next_goal() == False:
                    rospy.loginfo("Reached one of the checkpoints!")
                else:
                    rospy.loginfo("Yay! Got out of the maze!")
                    move_cmd = Twist()
                continue


            goal_heading = self.get_goal_angle(x, y)
            goal_sector = self.degree_to_sector(heading, goal_heading)

            histogram = self.get_polar_histogram()

            if histogram[goal_sector] == 0:
                err = goal_heading - heading
                if err > 180:
                    err = - 360 + err
                err = math.radians(err)
                
                rospy.loginfo(f"heading: {heading} - goal heading: {goal_heading}")
                rospy.loginfo(f"heading right towards the goal! - err: {err}")

                P = self.k_p * err

                move_cmd.angular.z = P
            
            else:
                k_targ = self.degree_to_sector(heading, goal_heading)
                steering_sector = self.find_direction(k_targ, histogram)
                steering_degree = self.sector_to_degree(steering_sector)
            
                if not steering_degree:
                    rospy.loginfo("can't find a way!!")
                    move_cmd.angular.z = 0
                    continue

                err = steering_degree #abs(heading - steering_degree)
                if err > 180:
                    err = - 360 + err
                err = math.radians(err)
                
                rospy.loginfo(f"steering degree: {steering_degree} - err: {err}")

                P = self.k_p * err

                move_cmd.angular.z = P     

            self.r.sleep()


    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

        rospy.sleep(3)
            

if __name__ == '__main__':
    try:
        rospy.sleep(10)

        vfhc = VFHController()
        vfhc.follow_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
