#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rosyard_common.msg import Map, Cone
import matplotlib.pyplot as plt
import scipy.interpolate as si
from sklearn import svm
from rosyard_common_scripts.common_methods import float_to_rgb_str
import numpy as np


def cones_to_xy(cones):
    """ Expands the cone objects list to x, y position of cones
        Args:
            cones (list): cones objects
        Returns:
            x (list): list of x parameter
            y (list): list of y parameter
    """
    x = []
    y = []
    for cone in cones:
        x.append(cone.position.x)
        y.append(cone.position.y)
    return x, y

def bind_xy(x, y):
    """ Binds the x,y values together to represent a geomertrical point
        Args:
            x (list): list of x parameter
            y (list): list of y parameter
        Returns:
            point (list): list of geometrical (x,y) points
    """
    point = []
    for i in range(len(x)):
        point.append([x[i],y[i]])
    return point

def distance(p1, p2):
    """ calculates the distance between two geometrical points
        Args:
            p1: point containing (x,y) coordinates of p1
            p2: point containing (x,y) coordinates of p2
        Returns:
            distance between p1 & p2
    """
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def getBSpline(ls):
    """ Interpolate the polynomial geometrical points list.
        Args:
            points (list): list of geometrical points(cones) location
        Returns:
            new_points (list): Interpolated/splined geometrical points list
    """
    n=len(ls)*10
    degree=2
    ls = np.asarray(ls)
    count = ls.shape[0]

    # Prevent degree from exceeding count-1, otherwise splev will crash
    degree = np.clip(degree,1,count-1)

    temp = []
    for l in range(count-degree+1):
        temp.append(l)

    # Calculate knot vector
    kv = np.array([0]*degree + temp + [count-degree]*degree,dtype='int')

    # Calculate query range
    u = np.linspace(0,(count-degree),n)

    # Calculate result
    return np.array(si.splev(u, (kv,ls.T,degree))).T



def nearest(side2, n_side2, side1, x):
    """ Finds the nearest point in side2 from side1 point having index 'x'
        Args:
            side2 (list): list of geometrical points(cones) location
            n_side2: length of side2 list
            side1 (list): list of geometrical points(cones) location
            x: index value of point in side1
        Returns:
            mp: point containing (x,y) nearest to side1[x]
    """
    md = distance(side2[x%n_side2], side1[x])
    mp = side2[(x%n_side2)]

    for i in range(x+1, n_side2):
    	d = distance(side2[i%n_side2], side1[x])
    	if d < md:
    		md, mp = d, side2[i % n_side2]
    return mp


def expand_xy(points):
    """ Expands the x, y coordinates from the input list.
        Args:
            points (list): points containing list of x,y coordinates
        Returns:
            x (list): list of x coordinate
            y (list): list of y coordinate
    """
    x = []
    y = []
    for point in points:
    	x.append(point[0])
    	y.append(point[1])

    return x, y


def clasify_cones(X, y, TestSet):
    """ Classify the cones into two different list.
        Args:
            X (list): List of training sample to train svm model
            y (list): List of classes for training svm model
            TestSet (list): list of coordinates to classify
        Returns:
            C1 (list): classified list out of TestSet with class1
            C2 (list): classified list out of TestSet with class2
    """
    clf = svm.SVC(kernel='rbf', C=1E6, gamma = 'auto')
    poly = clf.fit(X, y)
    C1 = []
    C2 = []
    for point in TestSet:
        pred_y = clf.predict([point])
        if pred_y == [0]:
            C1.append(point)
        elif pred_y == [1]:
            C2.append(point)
    return C1, C2


#calculates centerline between two set of points
def get_centerline(side1, side2):
    """ Calculates the centerline between two lists of geometrical points(x,y).
        Args:
            side1 (list): points containing list of x,y coordinates(cones)
            side2 (list): points containing list of x,y coordinates(cones)
        Returns:
            centerline (list): list of centerline points(x,y)
    """
    n_side1, n_side2 = len(side1), len(side2)
    p1 = []
    p2 = []
    if n_side1 < n_side2:
    	p1 = side1
    	nearest_p = []
    	for j in range(n_side1):
    		nearest_p.append(nearest(side2, n_side2, side1, j))
    	p2 = nearest_p
    else:
    	p2 = side2
    	nearest_p = []
    	for j in range(n_side2):
    		nearest_p.append(nearest(side1, n_side1, side2, j))
    	p1 = nearest_p

    return [(l[0]+r[0])/2 for l, r in zip(p1, p2)], [(l[1]+r[1])/2 for l, r in zip(p1, p2)]

def remove_dup(ls):
    """ Removes redundant(duplicate) data from a list.
        Args:
            ls (list): list of data
        Returns:
            new_list (list): List without redundant(duplicate) data
    """
    new_list = []
    for l in ls:
        if l not in new_list:
            new_list.append(l)
    return new_list


def get_sorted(ls):
    """ Sort the given coordinates list according to their spatial location
        and orient them clockwise(in some cases sorting make anti-clockwise orientation).
        Args:
            ls (list): list of coordinates
        Returns:
            new_list (list): List spatially ordered
    """
    # function to find the nearest point
    def nearestS(x, ls, O):
    	min_d, min_p = 999999, None
    	for p in ls:
    		if p in O: continue
    		d = distance(p, x)
    		if d<min_d: min_d, min_p = d, p
    	return min_p

    # function to order according to the nearest point
    def order(ls):
    	ordered, p, c, n = [ls[0]], ls[0], 1, len(ls)
    	while c<n:
    		p = nearestS(p, ls, ordered)
    		if p == None: break
    		ordered.append(p); c+=1
    	return ordered

    return order(ls)

# function to orient clockwise
def orient_clockwise(ls):
    """  orient the given list clockwise(in some cases sorting make anti-clockwise orientation).
        Args:
            ls (list): list of coordinates
        Returns:
            new_list (list): List spatially rotated
    """
    #---> Note :
    if ls[1][0]<ls[0][0]:
    	oriented = ls[1:]
    	oriented.reverse()
    	oriented = [ls[0]]+oriented
    	return oriented
    return ls

def return_list(ls):
    """  Returns x, y coordinate in [x, y] format.
        Args:
            ls (list): list of only x, y coordinate
        Returns:
            ret (list): [x, y]
    """
    ret = []
    for l in ls:
        ret.append([l[0],l[1]])
    return ret

def pre_process_cones(blue_cones, yellow_cones, noisy_cones):
    """ Pre-processing of cones having mis-identified color, redundant cones, unsorted order
        Args:
            blue_cones (list): list of x, y coordinates
            yellow_cones (list): list of x, y coordinates
            noisy_cones (list): list of x, y coordinates(coes having noisy color)
        Returns:
            blue_cones (list): list of x, y coordinates(after pre-processing)
            yellow_cones (list): list of x, y coordinates(after pre-processing)
    """
    #---> Cone color classification
    X = blue_cones+yellow_cones

    test_set = blue_cones+yellow_cones+noisy_cones

    y_blue = []
    y_yellow = []
    for x in blue_cones:
        y_blue.append(0)
    for x in yellow_cones:
        y_yellow.append(1)

    y = y_blue + y_yellow

    blue_cones, yellow_cones = clasify_cones(X, y, test_set)

    #---> Remove duplicate cones from cones list
    blue_cones = remove_dup(blue_cones)
    yellow_cones = remove_dup(yellow_cones)

    #---> Sorting the cones
    blue_cones = get_sorted(blue_cones)
    yellow_cones = get_sorted(yellow_cones)
    blue_cones = orient_clockwise(blue_cones)
    yellow_cones = orient_clockwise(yellow_cones)

    return blue_cones, yellow_cones


class CenterLineEstimation:
    def __init__(self):
        self.calc_once = True
        self.initial_lap = False
        self.map = Map()
        self.map.header.frame_id = "map"

    def getMap(self):
        """
            Args:
            Returns:
                self.map: map object of this class
        """
        return self.map

    def mapCallback(self, slam_map):
        if self.calc_once:
            #print('Callback_Called')
            #self.calc_once = False
            #---> getting cones data from point cloud
            cones = point_cloud2.read_points(slam_map, skip_nans=True, field_names=("x", "y", "z", "rgb"))
            #x = 0
            faulty_cones = []
            for cone in cones:
                p = Point()
                c = Cone()
                # remark-- orange cone has green intensity of 148 so, changed it in common_methods.py
                c_color = float_to_rgb_str(cone[3])

                #print(c_color)

                p.x = cone[0]
                p.y = cone[1]
                p.z = cone[2]

                c.position = p
                #---> Initial color classification
                if(c_color == 'b'):
                    c.color = c.BLUE
                    self.map.cone_blue.append(c)
                elif(c_color == 'y'):
                    c.color = c.YELLOW
                    self.map.cone_yellow.append(c)
                elif(c_color == 'o'):
                    c.color = c.ORANGE
                    self.map.cone_orange.append(c)
                else:
                    c.color = c.NONE
                    faulty_cones.append(c)

            blue_x, blue_y = cones_to_xy(self.map.cone_blue)
            yellow_x, yellow_y = cones_to_xy(self.map.cone_yellow)
            faulty_x, faulty_y = cones_to_xy(faulty_cones)

            blue_cones = bind_xy(blue_x, blue_y)
            yellow_cones = bind_xy(yellow_x, yellow_y)
            f_cones = bind_xy(faulty_x, faulty_y)

            #---> Pre-processing cones data(cone color classification, removal of duplicate cones, spatial ordering)
            blue_cones, yellow_cones = pre_process_cones(blue_cones, yellow_cones, f_cones)

            #---> Join the ends of map(make circular) --pre-processing filters the duplicate cones
            if not self.initial_lap:
                blue_cones.append(blue_cones[0])
                yellow_cones.append(yellow_cones[0])

            #---> Calculating the splined data for blue and
            #     yellow cones to get more accurate centerline
            blue_cones = getBSpline(blue_cones)
            yellow_cones = getBSpline(yellow_cones)

            '''
            blue_x, blue_y = expand_xy(blue_cones)
            yellow_x, yellow_y = expand_xy(yellow_cones)
            plt.plot(blue_x,blue_y, c='blue')
            plt.plot(yellow_x,yellow_y, c='yellow')
            '''

            #---> calculating centerline
            midx, midy = get_centerline(blue_cones, yellow_cones)

            center_line = []
            #---> Assigning calculated centerline points to map object
            for mx, my in zip(midx, midy):
                p = Point()
                p.x = mx
                p.y = my
                center_line.append(p)
            self.map.centerline = center_line

            #---> When at the end calculation must stop
            if (midx[0] == midx[-1]) and (midy[0] == midy[-1]):
                self.calc_once = False
            #plt.plot(midx, midy, c='pink')
            #plt.show()


if __name__ == "__main__":

    # init node
    rospy.init_node('center_line_estimator', anonymous=True)

    cheat_driver_active = rospy.get_param('/node_parameters/cheating_driver/activate')

    if cheat_driver_active:
        map_topic = rospy.get_param('/nodes/gt_cone_topic_name') # get ground truth
    else:
        map_topic = rospy.get_param('/nodes/slam_map_topic_name') # get real data

    #location to publish center_line
    center_line_topic = rospy.get_param('/nodes/map_topic_name')
    updateRate = rospy.get_param('/nodes/map_rate')

    center_line_est = CenterLineEstimation()
    #Subscriber
    rospy.Subscriber(map_topic, PointCloud2, center_line_est.mapCallback)
    #Publisher
    map_publisher = rospy.Publisher(center_line_topic, Map, queue_size=10)

    rate = rospy.Rate(updateRate)

    try:
        while not rospy.is_shutdown():
            #rospy.spin()
            map = center_line_est.getMap()
            map_publisher.publish(map)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", 'Center Line Estimation')
