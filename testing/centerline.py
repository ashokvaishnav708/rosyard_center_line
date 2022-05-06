#centerline.py

import numpy as np
import random
import math
import matplotlib.pyplot as plt
from data import MapPointsXY
from sklearn import svm
import scipy.interpolate as si
import yaml
import os

BLUE_CONES = "cones_left"
YELLOW_CONES = "cones_right"
CENTER_LINE = "middle_points"

def get_bspline(ls):
    n=1000
    degree=2
    ls = np.asarray(ls)
    count = ls.shape[0]

    degree = np.clip(degree,1,count-1)

    temp = []
    for l in range(count-degree+1):
    	temp.append(l)
    kv = np.array([0]*degree + temp + [count-degree]*degree,dtype='int')
    u = np.linspace(0,(count-degree),n)

    return np.array(si.splev(u, (kv,ls.T,degree))).T


def calc_linespace(arr):
	n_array = []
	i = 0
	while i < (len(arr)-1):
		xs = np.linspace(arr[i], arr[i+1], num=4)
		for x in xs:
			n_array.append(x)
		i = i+1
	return n_array

def bind_xy(x,y):
	point = []
	for i in range(len(x)):
		point.append([x[i],y[i]])
	return point


def expand_xy(points):
	x = []
	y = []
	for point in points:
		x.append(point[0])
		y.append(point[1])

	return x, y

def remove_dup(ls):
	new_list = []
	for l in ls:
		if l not in new_list:
			new_list.append(l)
	return new_list

def clasify_cones(X, y, TestSet):
    clf = svm.SVC(kernel='rbf', C=1E6)
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

def distance(p1, p2):
	return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def nearest(side2, n_side2, side1, x):
	md = distance(side2[x%n_side2], side1[x])
	mp = side2[(x%n_side2)]

	for i in range(x+1, n_side2):
		d = distance(side2[i%n_side2], side1[x])
		if d < md:
			md, mp = d, side2[i % n_side2]
	return mp

#calculates centerline between two set of points
def get_centerline(side1, side2):
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



def get_sorted(ls):

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
def orient_clockwise(L):
	if L[1][0]<L[0][0]:
		oriented = L[1:]
		oriented.reverse()
		oriented = [L[0]]+oriented
		return oriented
	return L

def calc_centerline(t_left, t_right):

	anonymous_points = t_left+t_right

	#cone color classification start
	X = t_left+t_right

	y_l = []
	y_r = []
	for x in t_left:
		y_l.append(0)
	for x in t_right:
		y_r.append(1)

	y = y_l + y_r

	
	t_left, t_right = clasify_cones(X, y, anonymous_points)

	t_left = remove_dup(t_left)
	t_right = remove_dup(t_right)

	t_left = get_sorted(t_left)
	t_right = get_sorted(t_right)

	#t_left = orient_clockwise(t_left)
	#t_right = orient_clockwise(t_right)

	t_left = get_bspline(t_left)
	t_right = get_bspline(t_right)

	midx, midy = get_centerline(t_left, t_right)

	return midx, midy

def get_track(flag):

	path = r'C:\Users\0202a\Downloads\New folder\tracks'
	tracks = os.path.join(path)
	
	track_names = os.listdir(tracks)
	if flag is 1:
		return track_names
	else:
		track_list =  list(map(lambda t: os.path.join(path, t), track_names))  
		#print('\n', track_list[:3], 'tracks')
		
		#track = tracks[0]
		#tracks = tracks[1:]
		return track_list 

def load_track(path):
    with open(path) as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    blue_cones = data[BLUE_CONES] if BLUE_CONES in data else None
    yellow_cones = data[YELLOW_CONES] if YELLOW_CONES in data else None
    center_points = data[CENTER_LINE] if CENTER_LINE in data else None

    """ Debug Messages """
    def debug_msg(val, str):
        if val is None:
            print("Load Warning: Key '" + str + "' returns None")
        elif len(val) == 0:
            print("Load Warning: Key '" + str + "' is empty")

    debug_msg(blue_cones, BLUE_CONES)
    debug_msg(yellow_cones, YELLOW_CONES)
    debug_msg(center_points, CENTER_LINE)

    return blue_cones, yellow_cones, center_points


def return_list(ls):
	ret = []
	for l in ls:
		ret.append([l[0],l[1]])
	return ret

if __name__ == "__main__":

	#map_points = MapPointsXY()
	
	#t_left, t_right = map_points.get_data()
	track_list = get_track(0)
	track_names = get_track(1)


	for t in range(len(track_list)):
		t_left,t_right,_ = load_track(track_list[t])

		x,y = expand_xy(t_left)
		plt.plot(x,y,'.',c='blue')
		x,y = expand_xy(t_right)
		plt.plot(x,y,'.',c='yellow')
		
		midx, midy = calc_centerline(t_left,t_right)

		plt.plot(midx, midy)
		
		plt.savefig('centerline/centerline_'+track_names[t][:-5])
		#plt.show()
		print('\n centerline', track_names[t], 'saved')
		plt.clf()
		
