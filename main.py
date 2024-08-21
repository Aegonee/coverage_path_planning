#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib
from scipy.interpolate import interp1d

'''
------------------------------------------------------
******************************************************

Here goes the variables used in the code

+++ File Reading +++
input               file reading

+++ Function Parameters +++
point1              temp used in func distance(a,b)
point2              temp used in func distance(a,b)
temp_x              temp used in func distance(a,b)
temp_y              temp used in func distance(a,b)
distance_output     result returned by func distance(a,b)
point_array         2-dimensional array storing points
vertex              element(point coordinates) in point_array

+++ UAV Constants +++
L                   radius of the coverage area of THE UAV (maybe a list for different UAVs? )
min_radius          minimum turning radius of the UAV
v_straght           velocity of the UAV when covering straihgt line
v_arc               velocity of the UAV when covering arc 
time_max            maximum endurance of UAV

+++ Area Constants +++
startpoint          coordinate of the start point
alpha               repeated area ration: s_repeated/L

******************************************************
------------------------------------------------------
'''

#read UAV parameters
'''
input = open("UAV","r+")
input.readlines
'''
'''
print(type())
   
'''


#read coverage area


#function: distance between 2 points
def point_distance(point1,point2):                                        # point1/2 must be a tupple/list
    temp_x=point2[0]-point1[0]
    temp_y=point2[1]-point1[1]
    distance_output=np.sqrt(temp_x*temp_x+temp_y*temp_y)
    return distance_output

#function: distance between 2 point, but circular arc
def arc_distance(point1,point2):
    radius_temp=point_distance(point1,point2)
    return np.pi*radius_temp*0.5


#funciton: calcultation of maximum width
def max_width(point_array):                                         # point_array is a 2-dimensional array storing points
    i,j = 0,0
    corr_relation=[]
    distance_array=[]
    for i in range(len(point_array)-1):
        for j in range(i+1,len(point_array)-1):
            corr_relation.append((i,j))
            distance = point_distance(point_array[i],point_array[j])
            distance_array.append(distance)
    point_index=distance_array.index(max(distance_array))
    return (max(distance_array),corr_relation[point_index])
    
#function: spliting an area
def split(a):
    pass

#function: find the width of a rectangle
def find_width(a):
    side1 = point_distance(a[0],a[1])
    side2 = point_distance(a[1],a[2])
    wid_result = min(side1,side2)
    if side1>side2:
        wid_point1=(a[1],a[2])
        wid_point2=(a[0],a[3])
    else:
        wid_point1=(a[0],a[1])
        wid_point2=(a[3],a[2])
    return wid_result,(wid_point1,wid_point2)

#function: specialized function for spliting rectangular areas
def split_rec(a,alpha,L):
    i=0
    tar_point_1,tar_point_2=[],[]
    block=0
    wid_temp,pointtemp=find_width(a)
    ratio1=(L*(1-alpha))/wid_temp
    ratio2=(0.5*L)/wid_temp
    if wid_temp%(L*(1-alpha))!=0:
        block=wid_temp//(L*(1-alpha))+1
    else:
        block=wid_temp//(L*(1-alpha))
    wid1_x=pointtemp[0][1][0]-pointtemp[0][0][0]
    wid1_y=pointtemp[0][1][1]-pointtemp[0][0][1]
    wid2_x=pointtemp[1][1][0]-pointtemp[1][0][0]
    wid2_y=pointtemp[1][1][1]-pointtemp[1][0][1]
    curr_point_x1=pointtemp[0][0][0]
    curr_point_y1=pointtemp[0][0][1]
    curr_point_x2=pointtemp[1][0][0]
    curr_point_y2=pointtemp[1][0][1]

    while i < block:
        if i==0:
            curr_point_x1+=wid1_x*ratio2
            curr_point_y1+=wid1_y*ratio2
            curr_point_x2+=wid2_x*ratio2
            curr_point_y2+=wid2_y*ratio2
#           curr_point_x1+=wid1_x*ratio1-wid1_x*ratio2
#           curr_point_y1+=wid1_y*ratio1-wid1_y*ratio2
#           curr_point_x2+=wid2_x*ratio1-wid2_x*ratio2
#           curr_point_y2+=wid2_y*ratio1-wid2_y*ratio2
        else:
            curr_point_x1+=wid1_x*ratio1
            curr_point_y1+=wid1_y*ratio1
            curr_point_x2+=wid2_x*ratio1
            curr_point_y2+=wid2_y*ratio1
        tar_point_1.append((curr_point_x1,curr_point_y1))
        tar_point_2.append((curr_point_x2,curr_point_y2))
        i+=1
    i=0
    return tar_point_1,tar_point_2
    
#function: tentative function used to calculate the total time of coverage
'''
Potential Error:        NEED TO BE SOLVED!
1. len(route)<3
2. len(route) is even
3. len(route)=3
'''
def time_cal(route,v):                          
# route: split not index, list/tuple    v: a tuple/list of 2 elements: v[0] for v_straight, v[1] for v_arc
    time_total=0
    dist_total=0
    dist_temp=0
    dist_collect=[]
    for i in range(len(route)):
        if i == 0 or i == 1 or i==len(route)-2:
            dist_temp=point_distance(route[i],route[i+1])
            dist_collect.append(dist_temp)
            time_total+=dist_temp/v[0]
            dist_total+=dist_temp
        elif i==len(route)-1:
            dist_temp=point_distance(route[i],route[0])
            dist_collect.append(dist_temp)
            time_total+=dist_temp/v[0]
            dist_total+=dist_temp
        else:
            if (i-2)%2==0:
                dist_temp=arc_distance(route[i],route[i+1])
                dist_collect.append(dist_temp)
                time_total+=dist_temp/v[1]
                dist_total+=dist_temp
            else:
                dist_temp=point_distance(route[i],route[i+1])
                dist_collect.append(dist_temp)
                time_total+=dist_temp/v[0]
                dist_total+=dist_temp
    return dist_total,time_total


#function: find the start point
def start_point(a1,a2,point1):
    dist_temp=[]
    point_array_sorted=[point1]
    dist_temp.append(point_distance(a1[0],point1))
    dist_temp.append(point_distance(a1[len(a1)-1],point1))
    dist_temp.append(point_distance(a2[0],point1))
    dist_temp.append(point_distance(a2[len(a2)-1],point1))
    min_index=dist_temp.index(min(dist_temp))
    if min_index==0:
        i=0
        while i < len(a1):
            if i%2==0:
                point_array_sorted.append(a1[i])
                point_array_sorted.append(a2[i])
            else:
                point_array_sorted.append(a2[i])
                point_array_sorted.append(a1[i])
            i+=1
    elif min_index==1:
        i=len(a1)-1
        j=0
        while i >= 0:
            if j%2==0:
                point_array_sorted.append(a1[i])
                point_array_sorted.append(a2[i])
            else:
                point_array_sorted.append(a2[i])
                point_array_sorted.append(a1[i])
            i-=1
            j+=1
    elif min_index==2:
        i=0
        while i < len(a1):
            if i%2==0:
                point_array_sorted.append(a2[i])
                point_array_sorted.append(a1[i])
            else:
                point_array_sorted.append(a1[i])
                point_array_sorted.append(a2[i])
            i+=1
        pass
    else:
        i=len(a1)-1
        j=0
        while i >= 0:
            if j%2==0:
                point_array_sorted.append(a2[i])
                point_array_sorted.append(a1[i])
            else:
                point_array_sorted.append(a1[i])
                point_array_sorted.append(a2[i])
            i-=1
            j+=1
        pass
    i=0
    j=0
    return point_array_sorted

'''
Main

'''

rectangle=[(0,0),(100,100),(300,-100),(200,-200)]
point=(0,50)
R=20
alph=0.15
arr1,arr2=split_rec(rectangle,alph,R)
route=start_point(arr1,arr2,point)
v=(100,50)
time=time_cal(route,v)
#print(find_width(rectangle))
#print(split_rec(rectangle,alph,R))
print(arr1,arr2)
print(route)

print(time)


x=np.arange(1,11)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set(xlim=[-200, 400], ylim=[-400, 200], title='Coverage Route', ylabel='Y-Axis', xlabel='X-Axis')
x=[0,100,300,200,0]
y=[0,100,-100,-200,0]
x1=[]
y1=[]
x0=[point[0]]
y0=[point[1]]
for i in range(len(route)):
    x1.append(route[i][0])
    y1.append(route[i][1])
x1.append(route[0][0])
y1.append(route[0][1])
for i in range(len(x1)):
    if (i!=0 and i!=1 and i!=len(x1)-1 and i!=len(x1)-2):
        if i%2==0:
            #plt.plot(Arc(xy=((x[i]+x[i+1])/2,(y[i]+y[i+1]/2),)))       #3 order Bezier curve
            pass
        pass
    pass
plt.plot(x,y)
plt.plot(x1,y1,"--")
plt.plot(x0,y0,"o")
plt.plot(route[1][0],route[1][1],"o")
plt.plot(route[len(route)-1][0],route[len(route)-1][1],"o")
ax.grid(True)
plt.axis("equal")
#plt.show()

# number of turning points
# criterion: max_width
'''
work need to be done: (rectangle)

1. build a class 'area'

2. finish function 'split'

3. finish file reading/writing

4. visualization of the result
'''

