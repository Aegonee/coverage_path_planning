#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# from typing import Counter, List
from sqlite3 import Time
from tkinter import CENTER
from typing import Counter, List, Sequence
from importlib_metadata import NullFinder
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
from sklearn.model_selection import ParameterSampler
from sqlalchemy import null
import re

from sqlalchemy.sql.expression import false

# import matplotlib
# from main import max_width, time_cal

'''
------------------------------------------------------
******************************************************

Version: 0.1.0_alpha

Version: 0.1.1_alpha
Bug fix:
1. turning trajectories have now been optimized.
2. the 2 abnormal points will no longer appear.
3. 4 new methods are now available: pre_process(), trans_process(), detrans_process(), mat_multiply().

Version: 0.1.2_alpha
added features:
1. now the trajactories of turning path are arcs.
2. plot1() and plot2() have been rewritten
3. new method arc() for class zigzagarea() is now available.

Version: 0.1.3_alpha
added features:
1. now the number of turning is available as output.
2. now the sub-area boundaries of multi-UAV tasks is available.
3. now the distance covered and the time consumed by any of the multi-UAV tasks are available.

Here goes the variables used in the code

!!! All the internal template variables are encapsulated and cannot be accessed from external !!!

+++ Function Parameters +++


+++ UAV Constants +++
L                   radius of the coverage area of THE UAV (maybe a list for different UAVs? )
min_radius          minimum turning radius of the UAV
v_straght           velocity of the UAV when covering straihgt line
v_arc               velocity of the UAV when covering arc 
time_max            maximum endurance of UAV

+++ Area Constants +++
startpoint          coordinate of the start point
alpha               repeated area ration: s_repeated/L


+++ Class Method(zigzagarea) +++
find_shape()            return the total number of the specific area
dot_product()           return the dot product of two vectors, static method
cross_product()         return the cross product of two 2-DIMENSIONAL vectors, static method
isconvec()              determine whether the area is a convex polygon
point_point_dist()      return the distance between two points, static method
point_line_dist()       return the distance between a point and a line, static method
arc_dist()              return the distance of a circular arc trace, static method
min_width()             find the minimun width of a specific polygon
inv()                   find the target 2-DIMENSIONAL transformation matrix, static method
cood_trans()            exert the matrix left to the column vector, static method
init()                  assign alpha and minimum turning radius to zigzagarea class
inv_list()              re-arrange the list to reverse oder, statiic method
base_vector()           find the base_vector of the coordinate transformation
point_sort()            after assign the the start point, calculate the coverage route
split()                 according to the minimum turning radius and alpha, split the area(transformed)
trans_solve()           before split, coordinate transformation must be excuted
trans_desolve()         after split, inverse coordinate transformation must be excuted


+++ Class Method(totalarea) +++
area()                  assign the initial value of total area
area_sub()              calculate the area of all blocks and save the value to internal variables
QK_sort()               quick sort
area_split()            split the area based on UAV parameters and return the sub-area points
UAV_input()             input the UAV parameters
coverage()              assigh the task to every single UAV, return the coverage route of task



ATTENTION: USE DECIMAL TO MAKE RESULT MORE ACCURATE!!!


******************************************************
------------------------------------------------------
'''
class zigzagarea:

    #Single UAV part
    start_point=[0,0]             #coordinate of the start point
    shape=0                       #number of polygon edges
    point=[]                      #ordinal list of the coordinates of vertices
    trans_point=[]                #ordinal list of the transformed coordinates of vertices
    vector=[]                     #vectors of polygon edges
    width=0                       #minimum width of polygon
    point_line=[]                 #3 points, opposite point of the edge corresponding to the minimum width, and the edge
    b_vector=[]                   #base vector, orientation: the edge corresponding to the minimum width, and minimum width
    displace=[0,0]                #displacement during coordinate transformation
    isdisp=False                  #if displacement has taken place when coordinate transforming
    min_radius=0                  #minimum turning radius of UAV
    alpha=0                       #repetition rate of coverage area
    split_right=[]                #un-desolved points, temperate list of split() method
    split_left=[]                 #un-desolved points, temperate list of split() method
    returnl=[]                    #desolved points, temperate list of split() method
    returnr=[]                    #desolved points, temperate list of split() method
    returnr=[]                    #path points of coverage area
    velocity_straight=0           #average speed of covering straight route
    velocity_arc=0                #average speed of covering arc route
    time=0                        #total time of covering the whole area
    dist=0                        #total distance travelled
    mat_process=[]
    circle_direction=0
    points_sorted=[]
    iscalc=False

    #Area distribution part
    area_point=[]
    area_total=0
    sub_area=[]
    trans_sub=[]


    def __init__(self,point__input) -> None:                       #checked √
        zigzagarea.point=point__input
    
    def __str__(self) -> str:
        pass 
    
    def find_shape(self) -> int:                            #checked √
        self.shape=len(self.point)
        return self.shape

    @staticmethod
    def depth(vector):
        for i in vector:
            if not isinstance(i,(list,tuple)):
                return 1
            else:
                return zigzagarea.depth(i)+1

    @staticmethod
    def dot_product(vector1,vector2) -> float:              #partly checked √?
        __dot_temp=0    #IndexError
        if len(vector1)!=len(vector2) or isinstance(vector1,(tuple,list))!=True or isinstance(vector1,(tuple,list))!=True:
            raise Exception("Invalid Value!",vector1,vector2)
        else:
            for i in range(len(vector1)):
                __dot_temp+=vector1[i]*vector2[i]
        return __dot_temp

    @staticmethod                                           #partly checked √?
    def cross_product(vector1,vector2) -> float:
        __dot_temp=0
        if len(vector1)!=len(vector2) or isinstance(vector1,(tuple,list))!=True or isinstance(vector1,(tuple,list))!=True:
            raise Exception("Invalid Value!",vector1,vector2)
        else:
            __dot_temp=vector1[0]*vector2[1]-vector2[0]*vector1[1]
        return __dot_temp
    
    @staticmethod
    def approximate(float1, float2, error = 0.001):
        if abs(float1 - float2) <= abs(error * float1) and abs(float1 - float2)<= error * abs(error * float2):
            return True
        else:
            return False
    
    def isconvex(self) -> bool:
        __dot_temp=[]
        for i in range(len(self.point)):                  #error exist
            if i!=len(self.point)-1:
                self.vector.append((self.point[i+1][0]-self.point[i][0],self.point[i+1][1]-self.point[i][1]))
            else:
                self.vector.append((self.point[0][0]-self.point[i][0],self.point[0][1]-self.point[i][1]))
        for i in range(len(self.vector)):
            if i!=len(self.vector)-1:
                __dot_temp.append(self.cross_product(self.vector[i],self.vector[i+1]))
            else:
                __dot_temp.append(self.cross_product(self.vector[i],self.vector[0]))
        for i in range(len(__dot_temp)-1):
            if __dot_temp[i]*__dot_temp[i+1]<0:
                return False
        return True
    
    @staticmethod
    def point_point_dist(point1,point2) -> float:           #partly checked √?
        __temp_x=point2[0]-point1[0]
        __temp_y=point2[1]-point1[1]
        __distance_output=np.sqrt(__temp_x*__temp_x+__temp_y*__temp_y)
        return __distance_output
    
    @staticmethod
    def point_line_dist(point1,line) -> float:              #partly checked √?
        if line[1][0]!=line[0][0]:
            __k=(line[1][1]-line[0][1])/(line[1][0]-line[0][0])
            __b=(line[1][0]*line[0][1]-line[0][0]*line[1][1])/(line[1][0]-line[0][0])
            __denomi=math.sqrt(__k*__k+1)
            __dist_temp=(abs(__k*point1[0]-point1[1]+__b))/__denomi
        else:
            __dist_temp=abs(line[0][0]-point1[0])
        return __dist_temp

    @staticmethod
    def arc_dist(point1,point2) -> float:
        __temp_x=point2[0]-point1[0]
        __temp_y=point2[1]-point1[1]
        __radius_temp=np.sqrt(__temp_x*__temp_x+__temp_y*__temp_y)
        return np.pi*__radius_temp*0.5

    def base_vector(self) ->list:
        __k=0
        __b=0
        __x1,__y1=0,0
        self.b_vector=[]
        __min_wid,__vertex=self.min_width()
        self.b_vector.append((__vertex[1][1][0]-__vertex[1][0][0],__vertex[1][1][1]-__vertex[1][0][1]))
        if __vertex[1][1][0]-__vertex[1][0][0]!=0:
            __k=(__vertex[1][1][1]-__vertex[1][0][1])/(__vertex[1][1][0]-__vertex[1][0][0])
            __b=(__vertex[1][1][0]*__vertex[1][0][1]-__vertex[1][0][0]*__vertex[1][1][1])/(__vertex[1][1][0]-__vertex[1][0][0])
            __x1=-(__b*__k-__vertex[0][0]-__k*__vertex[0][1])/(1+__k*__k)
            __y1=(__b+__k*__vertex[0][0]+__k*__k*__vertex[0][1])/(1+__k*__k)
            self.b_vector.append((__vertex[0][0]-__x1,__vertex[0][1]-__y1))
        else:
            self.b_vector.append((__vertex[0][0]-__vertex[1][0][0],0))
        '''
        print("base vector:")
        print(self.b_vector)
        '''
        return self.b_vector

    @staticmethod
    def inv(mat) ->list:
        __inv_mat=[[1,0],[0,1]]
        __det=mat[0][0]*mat[1][1]-mat[0][1]*mat[1][0]
        if __det==0:
            raise Exception("Invalid Value!",mat)
        else:
            __inv_mat[0][0]=(1/__det)*mat[1][1]
            __inv_mat[1][1]=(1/__det)*mat[0][0]
            __inv_mat[0][1]=-(1/__det)*mat[0][1]
            __inv_mat[1][0]=-(1/__det)*mat[1][0]
        return __inv_mat
    
    @staticmethod
    def det(mat) -> list:
        __det=mat[0][0]*mat[1][1]-mat[0][1]*mat[1][0]
        return __det
    
    @staticmethod
    def coord_trans(matrix,point) ->list:               #PS: point is actually a MATRIX, 2xN
        __point_return=[]
        for i in point:
            __point_return.append((i[0]*matrix[0][0]+i[1]*matrix[1][0],i[0]*matrix[0][1]+i[1]*matrix[1][1]))
        return __point_return

    @staticmethod
    def inv_list(list1) ->list:
        __temp=0
        __list_return=[]
        __counter=len(list1)-1
        while __counter>=0:
            __temp=list1[__counter]
            __list_return.append(__temp)
            __counter-=1    
        return __list_return

    @staticmethod
    def mat_multiply(mat1,mat2):
        __result=[]
        __i11=mat1[0][0]*mat2[0][0]+mat1[0][1]*mat2[1][0]
        __i12=mat1[0][0]*mat2[0][1]+mat1[0][1]*mat2[1][1]
        __i21=mat1[1][0]*mat2[0][0]+mat1[1][1]*mat2[1][0]
        __i22=mat1[1][0]*mat2[0][1]+mat1[1][1]*mat2[1][1]
        __result.append((__i11,__i12))
        __result.append((__i21,__i22))
        return __result

    def min_width(self) -> tuple:                            #partly checked √?
        __shape_temp=self.find_shape()
        __dist_temp=0
        __index_temp=0
        __max_temp_j=0
        __min_temp_i=0
        __result_line=[]
        __result_point=[]
        __tar_point=[]
        __ifodd=(__shape_temp%2==1)
        for i in range(len(self.point)):
            if i+1!=len(self.point):
                __bottom1=self.point[i]
                __bottom2=self.point[i+1]
                __flag1,__flag2=i,i+1
            else:
                __bottom1=self.point[i]
                __bottom2=self.point[0]
                __flag1,__flag2=i,0
            for j in range(len(self.point)):
                if j!=__flag1 and j!=__flag2:
                    __dist_temp=self.point_line_dist(self.point[j],(__bottom1,__bottom2))
                    if __dist_temp>=__max_temp_j:
                        __max_temp_j=__dist_temp
                        __tar_point=self.point[j]
            if __min_temp_i==0:
                __min_temp_i=__max_temp_j
                __result_point=__tar_point
                __result_line=(__bottom1,__bottom2)
            else:
                if __max_temp_j<=__min_temp_i:
                    __min_temp_i=__max_temp_j
                    __result_point=__tar_point
                    __result_line=(__bottom1,__bottom2)
            __max_temp_j=0

        self.width=__min_temp_i
        self.point_line=(__result_point,__result_line)
        return (__min_temp_i,(__result_point,__result_line))      
            
    def init(self,radius,alpha_input):                      #before split, init() and min_width() must be done
        self.min_radius=radius
        self.alpha=alpha_input
    
    def trans_solve(self) -> tuple: 
        self.trans_point=[]
        __temp_index=0
        __mat_return=[]
        __temp_wid,__comp_point=self.min_width()
        __mid_mat=self.coord_trans(self.inv(self.base_vector()),self.point)
        __comp_mat=self.coord_trans(self.inv(self.base_vector()),__comp_point[1])
        if __comp_mat[0][0]!=0 or __comp_mat[0][1]!=0:
            self.isdisp=True
            self.displace[0]=__comp_mat[0][0]
            self.displace[1]=__comp_mat[0][1]
            for i in range(len(__mid_mat)):
                __temp_x=__mid_mat[i][0]-__comp_mat[0][0]
                __temp_y=__mid_mat[i][1]-__comp_mat[0][1]
                __mat_return.append((__temp_x,__temp_y))
            for i in range(len(__mat_return)):
                if __mat_return[i][0]==0 and __mat_return[i][1]==0:
                    __temp_index=i
            while len(self.trans_point)<len(__mat_return):
                self.trans_point.append(__mat_return[__temp_index])
                if __temp_index+1==len(__mat_return):
                    __temp_index=__temp_index+1-len(__mat_return)
                else:
                    __temp_index+=1
        else:
            for i in range(len(__mid_mat)):
                if __mid_mat[i][0]==0 and __mid_mat[i][1]==0:
                    __temp_index=i
            while len(self.trans_point)<len(__mid_mat):
                self.trans_point.append(__mid_mat[__temp_index])
                if __temp_index+1==len(__mid_mat):
                    __temp_index=__temp_index+1-len(__mid_mat)
                else:
                    __temp_index+=1
        return self.trans_point

    def trans_desolve(self) -> list:
        __temp_x,__temp_y=0,0
        __return_left=[]
        __return_right=[]
        if self.isdisp==True:

            for i in range(len(self.split_left)):
                __temp_x=self.split_left[i][0]+self.displace[0]
                __temp_y=self.split_left[i][1]+self.displace[1]
                __return_left.append((__temp_x,__temp_y))
            for i in range(len(self.split_right)):
                __temp_x=self.split_right[i][0]+self.displace[0]
                __temp_y=self.split_right[i][1]+self.displace[1]
                __return_right.append((__temp_x,__temp_y))
        __return_left=self.coord_trans(self.base_vector(),__return_left)
        __return_right=self.coord_trans(self.base_vector(),__return_right)
        self.returnl=__return_left
        self.returnr=__return_right
        return __return_left,__return_right

    def trans_process(self) -> None:
        __mat_mid=[]
        __norm1=self.point_point_dist(self.point_line[1][0],self.point_line[1][1])
        __norm2=self.point_line_dist(self.point_line[0],self.point_line[1])
        __mat_mid.append((self.b_vector[0][0]/__norm1,self.b_vector[0][1]/__norm1))
        __mat_mid.append((self.b_vector[1][0]/__norm2,self.b_vector[1][1]/__norm2))
        self.returnl=self.coord_trans(self.inv(__mat_mid),self.returnl)
        self.returnr=self.coord_trans(self.inv(__mat_mid),self.returnr)
        self.mat_process=__mat_mid
        return

    def detrans_process(self) -> None:
        self.returnl=self.coord_trans(self.mat_process,self.returnl)
        self.returnr=self.coord_trans(self.mat_process,self.returnr)
        return

    def split(self) -> list:
        __xl_temp,__xr_temp=0,0
        __dist_trav=0
        __block=0
        __itr_temp=1
        __index_temp=0
        __point_left=[]
        __point_right=[]
        __max_dist=self.point_line_dist(self.trans_point[2],(self.trans_point[0],self.trans_point[1]))
        __max_point=[]
        __counter=0
        __left_counter,__right_counter=0,0
        __return_left=[]
        __return_right=[]
        __break_point=0

        for i in range(len(self.trans_point)):
            if i!=0 and i!=1:
                if self.point_line_dist(self.trans_point[i],(self.trans_point[0],self.trans_point[1]))>=__max_dist:
                    __max_dist=self.point_line_dist(self.trans_point[i],(self.trans_point[0],self.trans_point[1]))
                    __index_temp=i
                    __max_point=self.trans_point[i]
        while __itr_temp<__index_temp:
                __point_right.append(self.trans_point[__itr_temp])
                __itr_temp+=1
        __itr_temp=__index_temp+1
        while __itr_temp<=len(self.trans_point):
            if __itr_temp==len(self.trans_point):
                __point_left.append(self.trans_point[0])
                break
            else:
                    __point_left.append(self.trans_point[__itr_temp])
            __itr_temp+=1

        __point_left=self.inv_list(__point_left)
        __trans_radius=self.min_radius/self.width
        if (__max_dist-__trans_radius)%(__trans_radius*(1-self.alpha))!=0:
            __block=(__max_dist-__trans_radius)//(__trans_radius*(1-self.alpha))+2
        else:
            __block=(__max_dist-__trans_radius)//(__trans_radius*(1-self.alpha))+1
        __ratio1=__trans_radius*(1-self.alpha)
        __ratio2=0.5*__trans_radius

        while __counter<=__block-1:
            #first point
            if __counter==0:
                __dist_trav+=__ratio2
                if __left_counter+1==len(__point_left):
                    if __counter!=__block-1:
                        __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                            (__max_point[0]-__point_left[__left_counter][0])/\
                            (__max_point[1]-__point_left[__left_counter][1])
                    __return_left.append((__xl_temp,__dist_trav)) 
                elif __point_left[__left_counter+1][1]>=__dist_trav:
                    __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                        (__point_left[__left_counter+1][0]-__point_left[__left_counter][0])/\
                        (__point_left[__left_counter+1][1]-__point_left[__left_counter][1])
                    __return_left.append((__xl_temp,__dist_trav))
                else:
                    __left_counter+=1
                    if __left_counter+1==len(__point_left):
                        if __counter!=__block-1:
                            __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                                (__max_point[0]-__point_left[__left_counter][0])/\
                                (__max_point[1]-__point_left[__left_counter][1])
                        __return_left.append((__xl_temp,__dist_trav))                        
                    else:
                        __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                            (__point_left[__left_counter+1][0]-__point_left[__left_counter][0])/\
                            (__point_left[__left_counter+1][1]-__point_left[__left_counter][1])
                        __return_left.append((__xl_temp,__dist_trav))
                if __right_counter+1==len(__point_right):
                    if __counter!=__block-1:
                        __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                            (__max_point[0]-__point_right[__right_counter][0])/\
                            (__max_point[1]-__point_right[__right_counter][1])
                    __return_right.append((__xr_temp,__dist_trav))
                elif __point_right[__right_counter+1][1]>=__dist_trav:
                    __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                        (__point_right[__right_counter+1][0]-__point_right[__right_counter][0])/\
                        (__point_right[__right_counter+1][1]-__point_right[__right_counter][1])
                    __return_right.append((__xr_temp,__dist_trav))
                else:
                    __right_counter+=1
                    if __right_counter+1==len(__point_right):
                        if __counter!=__block-1:
                            __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                                (__max_point[0]-__point_right[__right_counter][0])/\
                                (__max_point[1]-__point_right[__right_counter][1])
                        __return_right.append((__xr_temp,__dist_trav))
                    else: 
                        __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                            (__point_right[__right_counter+1][0]-__point_right[__right_counter][0])/\
                            (__point_right[__right_counter+1][1]-__point_right[__right_counter][1])
                        __return_right.append((__xr_temp,__dist_trav))
            #second and more points
            else:
                __dist_trav+=__ratio1
                if __left_counter+1==len(__point_left):
                    if __counter==__block-1 and __max_point[1]<__dist_trav:
                        pass
                    else:
                        __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                            (__max_point[0]-__point_left[__left_counter][0])/\
                            (__max_point[1]-__point_left[__left_counter][1]) 
                    __return_left.append((__xl_temp,__dist_trav))
                elif __point_left[__left_counter+1][1]>=__dist_trav:
                    __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                        (__point_left[__left_counter+1][0]-__point_left[__left_counter][0])/\
                        (__point_left[__left_counter+1][1]-__point_left[__left_counter][1])
                    __return_left.append((__xl_temp,__dist_trav))
                else:
                    __left_counter+=1
                    if __left_counter+1==len(__point_left):
                        if abs(__max_point[1]-__point_left[__left_counter][1])>0.001:
                            if __counter!=__block-1:
                                __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                                    (__max_point[0]-__point_left[__left_counter][0])/\
                                    (__max_point[1]-__point_left[__left_counter][1])
                            __return_left.append((__xl_temp,__dist_trav))
                        else:
                            if __counter!=__block-1:
                                __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                                    (__point_left[__left_counter][0]-__point_left[__left_counter-1][0])/\
                                    (__point_left[__left_counter][1]-__point_left[__left_counter-1][1])
                            __return_left.append((__xl_temp,__dist_trav))
                        #__xl_temp=__point_left[__left_counter][]                     
                    else:
                        __xl_temp=__point_left[__left_counter][0]+(__dist_trav-__point_left[__left_counter][1])*\
                            (__point_left[__left_counter+1][0]-__point_left[__left_counter][0])/\
                            (__point_left[__left_counter+1][1]-__point_left[__left_counter][1])
                        __return_left.append((__xl_temp,__dist_trav))

                if __right_counter+1==len(__point_right):
                    if __counter==__block-1 and __max_point[1]<__dist_trav:
                        pass
                    else:
                        __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                            (__max_point[0]-__point_right[__right_counter][0])/\
                            (__max_point[1]-__point_right[__right_counter][1])
                    __return_right.append((__xr_temp,__dist_trav))
                elif __point_right[__right_counter+1][1]>=__dist_trav:
                    __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                        (__point_right[__right_counter+1][0]-__point_right[__right_counter][0])/\
                        (__point_right[__right_counter+1][1]-__point_right[__right_counter][1])
                    __return_right.append((__xr_temp,__dist_trav))
                else:
                    __right_counter+=1
                    if __right_counter+1==len(__point_right):
                        if abs(__max_point[1]-__point_right[__right_counter][1])>0.001:
                            if __counter!=__block-1:
                                __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                                    (__max_point[0]-__point_right[__right_counter][0])/\
                                    (__max_point[1]-__point_right[__right_counter][1])
                            __return_right.append((__xr_temp,__dist_trav))
                        else:
                            if __counter!=__block-1:
                                __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                                    (__point_right[__right_counter][0]-__point_right[__right_counter-1][0])/\
                                    (__point_right[__right_counter][1]-__point_right[__right_counter-1][1])
                            __return_right.append((__xr_temp,__dist_trav))
                    else: 
                        __xr_temp=__point_right[__right_counter][0]+(__dist_trav-__point_right[__right_counter][1])*\
                            (__point_right[__right_counter+1][0]-__point_right[__right_counter][0])/\
                            (__point_right[__right_counter+1][1]-__point_right[__right_counter][1])
                        __return_right.append((__xr_temp,__dist_trav))
            __counter+=1

        self.split_right=__return_right
        self.split_left=__return_left

        return self.split_left,self.split_right

    def pre_process(self,index):
        self.trans_process()
        if index==0:
            for i in range(len(self.returnl)):
                if i%2==1:
                    if i+1<len(self.returnl):
                        if self.returnl[i][0]<=self.returnl[i+1][0]:
                            self.returnl[i+1]=(self.returnl[i][0],self.returnl[i+1][1])
                        else:
                            self.returnl[i]=(self.returnl[i+1][0],self.returnl[i][1])
                    else:
                        continue
                else:
                    if i+1<len(self.returnr):
                        if self.returnr[i][0]>=self.returnr[i+1][0]:
                            self.returnr[i+1]=(self.returnr[i][0],self.returnr[i+1][1])
                        else:
                            self.returnr[i]=(self.returnr[i+1][0],self.returnr[i][1])
                    else:
                        continue
        elif index==1:
            for i in range(len(self.returnl)-1,-1,-1):
                if (len(self.returnl)-i)%2==0:
                    if i-1>=0:
                        if self.returnl[i][0]<=self.returnl[i-1][0]:
                            self.returnl[i-1]=(self.returnl[i][0],self.returnl[i-1][1])
                        else:
                            self.returnl[i]=(self.returnl[i-1][0],self.returnl[i][1])
                    else:
                        continue
                else:
                    if i-1>=0:
                        if self.returnr[i][0]>=self.returnr[i-1][0]:
                            self.returnr[i-1]=(self.returnr[i][0],self.returnr[i-1][1])
                        else:
                            self.returnr[i]=(self.returnr[i-1][0],self.returnr[i][1])
        elif index==2:
            for i in range(len(self.returnl)):
                if i%2!=1:
                    if i+1<len(self.returnl):
                        if self.returnl[i][0]<=self.returnl[i+1][0]:
                            self.returnl[i+1]=(self.returnl[i][0],self.returnl[i+1][1])
                        else:
                            self.returnl[i]=(self.returnl[i+1][0],self.returnl[i][1])
                    else:
                        continue
                else:
                    if i+1<len(self.returnr):
                        if self.returnr[i][0]>=self.returnr[i+1][0]:
                            self.returnr[i+1]=(self.returnr[i][0],self.returnr[i+1][1])
                        else:
                            self.returnr[i]=(self.returnr[i+1][0],self.returnr[i][1])
                    else:
                        continue
        elif index==3:
            for i in range(len(self.returnl)-1,-1,-1):
                if (len(self.returnl)-i)%2!=0:
                    if i-1>=0:
                        if self.returnl[i][0]<=self.returnl[i-1][0]:
                            self.returnl[i-1]=(self.returnl[i][0],self.returnl[i-1][1])
                        else:
                            self.returnl[i]=(self.returnl[i-1][0],self.returnl[i][1])
                    else:
                        continue
                else:
                    if i-1>=0:
                        if self.returnr[i][0]>=self.returnr[i-1][0]:
                            self.returnr[i-1]=(self.returnr[i][0],self.returnr[i-1][1])
                        else:
                            self.returnr[i]=(self.returnr[i-1][0],self.returnr[i][1])
        
        self.detrans_process()
        return

    def arc(self,point1,point2):
        __center=((point1[0]+point2[0])*0.5,(point1[1]+point2[1])*0.5)
        __vec=((point1[0]-point2[0])*0.5,(point1[1]-point2[1])*0.5)
        __vec1=[__vec]
        __result=[]
        i=0
        __result.append(point1)
        while i<181:
            __angle = 1/180*math.pi
            __mat=((math.cos(__angle),math.sin(__angle)),(-math.sin(__angle),math.cos(__angle)))
            __vec_temp=self.coord_trans(__mat,__vec1)
            __point=(__vec_temp[0][0]+__center[0],__vec_temp[0][1]+__center[1])
            __result.append(__point)
            __vec1=__vec_temp
            i+=1
        return __result
    
    def point_sort(self,start_point=[0,0]):
        self.start_point[0]=start_point[0]
        self.start_point[1]=start_point[1]
        __dist_temp=0
        __dist_return=self.point_point_dist(start_point,self.returnl[0])
        __mark,__mark_temp=0,0
        __i=1
        __points_sorted=[]
        while __i<4:
            if __i==1:
                __dist_temp=self.point_point_dist(start_point,self.returnl[len(self.returnl)-1])
                __mark_temp=1
            elif __i==2:
                __dist_temp=self.point_point_dist(start_point,self.returnr[0])
                __mark_temp=2
            elif __i==3:
                __dist_temp=self.point_point_dist(start_point,self.returnr[len(self.returnr)-1])
                __mark_temp=3
            if __dist_temp<=__dist_return:
                __dist_return=__dist_temp
                __mark=__mark_temp
            __i+=1

        self.pre_process(__mark)

        __points_sorted.append(start_point)
        if __mark==0:
            for i in range(len(self.returnl)):
                if i%2==0:
                    __points_sorted.append(self.returnl[i])
                    __points_sorted.append(self.returnr[i])
                elif i%2==1:
                    __points_sorted.append(self.returnr[i])
                    __points_sorted.append(self.returnl[i])
        elif __mark==1:
            for i in range(len(self.returnl)):
                if i%2==0:
                    __points_sorted.append(self.returnl[len(self.returnl)-1-i])
                    __points_sorted.append(self.returnr[len(self.returnr)-1-i])
                elif i%2==1:
                    __points_sorted.append(self.returnr[len(self.returnr)-1-i])
                    __points_sorted.append(self.returnl[len(self.returnl)-1-i])                    
        elif __mark==2:
            for i in range(len(self.returnr)):
                if i%2==0:
                    __points_sorted.append(self.returnr[i])
                    __points_sorted.append(self.returnl[i])
                elif i%2==1:
                    __points_sorted.append(self.returnl[i])
                    __points_sorted.append(self.returnr[i])
        else:
            for i in range(len(self.returnl)):
                if i%2==0:
                    __points_sorted.append(self.returnr[len(self.returnr)-1-i])
                    __points_sorted.append(self.returnl[len(self.returnl)-1-i])
                elif i%2==1:
                    __points_sorted.append(self.returnl[len(self.returnl)-1-i])
                    __points_sorted.append(self.returnr[len(self.returnr)-1-i])
        __points_sorted.append(start_point)
        self.returnr=__points_sorted

        __center=((__points_sorted[2][0]+__points_sorted[3][0])*0.5,(__points_sorted[2][1]+__points_sorted[3][1])*0.5)
        __vec_x=__points_sorted[2][0]-__points_sorted[1][0]
        __vec_y=__points_sorted[2][1]-__points_sorted[1][1]
        __vec1=[((__points_sorted[2][0]-__points_sorted[3][0])*0.5,(__points_sorted[2][1]-__points_sorted[3][1])*0.5)]
        __angle = 0.5*math.pi
        __mat=((math.cos(__angle),math.sin(__angle)),(-math.sin(__angle),math.cos(__angle)))
        __vec1=self.coord_trans(__mat,__vec1)
        if self.dot_product(__vec1[0],(__vec_x,__vec_y))<0:
            self.circle_direction=1
        else:
            self.circle_direction=0
        self.points_sorted=__points_sorted

        return __points_sorted

    def turning_cal(self):
        return int((len(self.points_sorted)+2)/2)-3
 
    def time_cal(self,v_straght,v_arc):
        self.velocity_straight=v_straght
        self.velocity_arc=v_arc
        __total_dist=0
        __total_time=0
        ##print('lengthxxx',len(self.points_sorted))
        for i in range(len(self.points_sorted)-1):

            if i == 0 or i == 1 or i==len(self.points_sorted)-1 or i==len(self.points_sorted)-2:
                __total_dist+=self.point_point_dist(self.points_sorted[i],self.points_sorted[i+1])
                __total_time+=self.point_point_dist(self.points_sorted[i],self.points_sorted[i+1])/v_straght
            else:
                if i%2==0 and i!=0:
                    __total_dist+=self.arc_dist(self.points_sorted[i],self.points_sorted[i+1])
                    __total_time+=self.arc_dist(self.points_sorted[i],self.points_sorted[i+1])/v_arc
                elif i%2==1:
                    __total_dist+=self.point_point_dist(self.points_sorted[i],self.points_sorted[i+1])
                    __total_time+=self.point_point_dist(self.points_sorted[i],self.points_sorted[i+1])/v_straght
            '''
            if i < len(self.points_sorted)-2:
                if i % 2 == 0 and i !=0:
                    __total_dist += self.arc_dist(self.points_sorted[i], self.points_sorted[i + 1])
                    __total_time += self.arc_dist(self.points_sorted[i], self.points_sorted[i + 1]) / v_arc
                elif i % 2 == 1 or i ==0:
                    __total_dist += self.point_point_dist(self.points_sorted[i], self.points_sorted[i + 1])
                    __total_time += self.point_point_dist(self.points_sorted[i], self.points_sorted[i + 1]) / v_straght
            if i == len(self.points_sorted)-2:
                __total_dist+= 0
                __total_time+= 0
            '''
            #print('total_time',i,self.points_sorted[i],__total_time)

        return __total_dist,__total_time

class totalarea(zigzagarea):

    sub_point=[]
    sub_alpha=[]
    sub_radius=[]
    sub_v=[]
    sub_area=[]
    trans_sub=[]
    mission_point=[]
    circle_direction_total=[]
    dist_total=[]
    time_total=[]
    turning_total=[]
    boundary=[]
    

    def __init__(self,point_input) -> None:                       #checked √
        totalarea.point=point_input

    @staticmethod
    def area(areapoint) -> float:
        __S=0
        for i in range(len(areapoint)):
            if i !=len(areapoint)-1:
                __S+=areapoint[i][0]*areapoint[i+1][1]-areapoint[i+1][0]*areapoint[i][1]
            else:
                __S+=areapoint[i][0]*areapoint[0][1]-areapoint[0][0]*areapoint[i][1]
        __S=abs(__S)*0.5
        return __S
    
    def area_sub(self,split_area,num,proportion) -> list:
        self.area_point=split_area
        __sub_temp=[]
        __sub_Ttemp=[]
        if num!=len(proportion):
            raise Exception("Invalid Value!",num,proportion)
        __total_area=self.area(split_area)
        for i in range(num):
            __sub_temp.append(__total_area*proportion[i])               #Existing errors!
            __sub_Ttemp.append(__total_area*proportion[i]*abs(self.det(self.inv(self.base_vector()))))
        self.area_total=self.area(self.area_point)
        self.sub_area=__sub_temp
        self.trans_sub=__sub_Ttemp
        return __sub_temp

    def QK_sort(self,input_L,begin,end) -> list:
        if begin>end:
            return
        __temp1=input_L[begin][0][1]
        __temp=input_L[begin]
        __i=begin
        __j=end
        while __i!=__j:
            while input_L[__j][0][1]>=__temp1 and __j>__i:
                __j-=1
            while input_L[__i][0][1]<=__temp1 and __j>__i:
                __i+=1
            if __j>__i:
                __i_temp=input_L[__i]
                input_L[__i]=input_L[__j]
                input_L[__j]=__i_temp
        input_L[begin]=input_L[__i]
        input_L[__i]=__temp
        self.QK_sort(input_L,begin,__i-1)
        self.QK_sort(input_L,__i+1,end)
    
    def area_split(self) -> list:
        __max_width=self.trans_point[2][1]
        __flag=0
        __ispara=False
        __left_vertex=[]
        __right_vertex=[]
        __counter=0
        __point_pair=[]
        __x0=0
        __y0=0
        __current_pos=[]
        __mark=0
        __area_temp=0
        __iter_area=0
        __upper=0
        __bottom=0
        __result=[]
        self.sub_point=[]


        #Must run self.area_sub() and self.trans_solve() before
        #Calulate total area and subdomain area, then distrubute the area
        for i in range(len(self.trans_point)):
            if i!=0 and i!=1:
                if __max_width<=self.trans_point[i][1]:
                    __max_width=self.trans_point[i][1]
                    __flag=i


        #Distinguish exceptions
        if abs(self.trans_point[__flag-1][1]-self.trans_point[__flag][1])<0.001:
            __ispara=True
        

        #Fill the vertex arrays(L&R)
        __left_vertex.append(self.trans_point[0])
        for i in range(len(self.trans_point)-1,__flag-1,-1):
            __left_vertex.append(self.trans_point[i])
        
        for i in range(1,__flag+1):
            if __ispara==True:
                if i!=__flag:
                    __right_vertex.append(self.trans_point[i])
            else:
                __right_vertex.append(self.trans_point[i])
    
        #Find point pairs
        #!!!When len(left_convex)==2 or len(left_convex)==2 exceptions would appear!!!
        __counter=0
        __mark=1
        __point_pair.append((__left_vertex[0],__right_vertex[0]))
        if len(__left_vertex)>2:
            while __mark<len(__left_vertex)-1:
                if __right_vertex[__counter+1][1]<__left_vertex[__mark][1]:         
                    __counter+=1
                else:
                    __x0=((__right_vertex[__counter+1][0]-__right_vertex[__counter][0])*__left_vertex[__mark][1]+\
                        __right_vertex[__counter][0]*__right_vertex[__counter+1][1]-\
                        __right_vertex[__counter+1][0]*__right_vertex[__counter][1])/\
                        (__right_vertex[__counter+1][1]-__right_vertex[__counter][1])
                    __y0=__left_vertex[__mark][1]
                    __point_pair.append((__left_vertex[__mark],(__x0,__y0)))
                    __mark+=1
        __point_pair.append((__left_vertex[len(__left_vertex)-1],__right_vertex[len(__right_vertex)-1]))
        __counter=0
        __mark=1
        __x0=0
        __y0=0
        if len(__right_vertex)>2:
            while __mark<len(__right_vertex)-1:
                if __left_vertex[__counter+1][1]<__right_vertex[__mark][1]:
                    __counter+=1
                else:
                    __x0=((__left_vertex[__counter+1][0]-__left_vertex[__counter][0])*__right_vertex[__mark][1]+\
                        __left_vertex[__counter][0]*__left_vertex[__counter+1][1]-\
                        __left_vertex[__counter+1][0]*__left_vertex[__counter][1])/\
                        (__left_vertex[__counter+1][1]-__left_vertex[__counter][1])
                    __y0=__right_vertex[__mark][1]
                    __point_pair.append(((__x0,__y0),__right_vertex[__mark]))
                    __mark+=1
        __point_pair.append((__left_vertex[len(__left_vertex)-1],__right_vertex[len(__right_vertex)-1]))
        __counter=0
        __mark=0
        __x0=0
        __y0=0


        #QKsort
        __point_pair=list(set(__point_pair))
        if len(__point_pair)!=0:
            self.QK_sort(__point_pair,0,len(__point_pair)-1)

        #split area
        #   !!!current position need to be adjusted!!!
        #   !!!overflowing indices need to be rewrited!!!
        #   !!!
        __mark=0
        __flag=0
        __counter=1
        __current_pos=[__point_pair[0][0],__point_pair[0][1]]
        __result.append(__point_pair[0])
        

        while __mark<=len(self.trans_sub)-2:   

            if __flag==0:
                if __point_pair[__counter][0][0]==__point_pair[__counter][1][0] and\
                        __point_pair[__counter][0][1]==__point_pair[__counter][1][1]:
                    __iter_area=self.area((__current_pos[0],__current_pos[1],__point_pair[__counter][1]))
                else:    
                    __iter_area=self.area((__current_pos[0],__current_pos[1],\
                    __point_pair[__counter][1],__point_pair[__counter][0]))
            else:
                if __point_pair[__counter][0][0]==__point_pair[__counter][1][0] and\
                        __point_pair[__counter][0][1]==__point_pair[__counter][1][1]:        
                    __iter_area=self.area((__point_pair[__counter-1][0],__point_pair[__counter-1][1],\
                    __point_pair[__counter][1]))
                else:
                    __iter_area=self.area((__point_pair[__counter-1][0],__point_pair[__counter-1][1],\
                    __point_pair[__counter][1],__point_pair[__counter][0]))
            __area_temp+=__iter_area
            __flag+=1
            if __area_temp<self.trans_sub[__mark]:
                __counter+=1
                continue
            else:

                if __flag==1:
                    __point_temp=__current_pos
                else:
                    __point_temp=__point_pair[__counter-1]

                __area_temp-=__iter_area
                __s=self.trans_sub[__mark]-__area_temp
                if __point_pair[__counter][0][0]!=__point_temp[0][0] or __point_pair[__counter][1][0]!=__point_temp[1][0]:
                    __a=__point_temp[0][0]-__point_temp[1][0]-\
                            __point_pair[__counter][0][0]+__point_pair[__counter][1][0]
                    __b=-__point_temp[0][0]*__point_pair[__counter][0][1]+\
                            __point_pair[__counter][0][0]*__point_temp[0][1]+\
                            __point_temp[1][0]*__point_pair[__counter][1][1]-\
                            __point_pair[__counter][1][0]*__point_temp[1][1]
                    __bottom=__point_temp[1][0]-__point_temp[0][0]
                    __delta=__point_pair[__counter][0][1]-__point_temp[0][1]
                    __square=__b*__b+2*__b*__bottom*__delta+math.pow(__bottom*__delta,2)+8*__a*__s*__delta+\
                            2*__a*__b*__point_temp[0][1]+2*__a*__bottom*__delta*__point_temp[0][1]+math.pow(__a*__point_temp[0][1],2)
                    __sol1=(-__bottom*__delta-__b+__a*__point_temp[0][1]+math.sqrt(__square))/(2*__a)
                    __sol2=(-__bottom*__delta-__b+__a*__point_temp[0][1]-math.sqrt(__square))/(2*__a)
                    if __sol1<__point_pair[__counter][0][1] and __sol1>=__point_temp[0][1]:
                        __current_sol=__sol1
                    elif __sol2<__point_pair[__counter][0][1] and __sol2>=__point_temp[0][1]:
                        __current_sol=__sol2
                    else:
                        raise Exception("math error!",__sol1,__sol2,__result,self.trans_sub)
                    __x1=(-__point_temp[0][0]*__current_sol+__point_pair[__counter][0][0]*__current_sol-\
                            __point_pair[__counter][0][0]*__point_temp[0][1]+\
                            __point_temp[0][0]*__point_pair[__counter][0][1])/__delta
                    __x2=(-__point_temp[1][0]*__current_sol+__point_pair[__counter][1][0]*__current_sol-\
                            __point_pair[__counter][1][0]*__point_temp[1][1]+\
                            __point_temp[1][0]*__point_pair[__counter][1][1])/__delta
                else:
                    __current_sol=__s/(__point_temp[1][0]-__point_temp[0][0])+__point_temp[0][1]
                    __x1=__point_temp[0][0]
                    __x2=__point_temp[1][0]
                __current_pos=((__x1,__current_sol),(__x2,__current_sol))
                __result.append(__current_pos)
                __mark+=1
                __area_temp=0
                __flag=0
        __result.append(__point_pair[len(__point_pair)-1])
        
        #print(__result)

        #sub-area points sort
        __sub_temp=[]
        for i in range(len(__result)-1):
            for j in range(len(__left_vertex)):
                if __result[i][0][1]>=__left_vertex[j][1]:
                    continue
                else:
                    __bottom_index_l=j-1
                    break
            for j in range(len(__right_vertex)):
                if __result[i][1][1]>=__right_vertex[j][1]:
                    continue
                else:
                    __bottom_index_r=j-1
                    break
            for j in range(len(__left_vertex)):
                if __result[i+1][0][1]>__left_vertex[j][1]:
                    continue
                else:
                    __top_index_l=j
                    break
            for j in range(len(__right_vertex)):
                if __result[i+1][1][1]>__right_vertex[j][1]:
                    continue
                else:
                    __top_index_r=j
                    break
            __sub_temp.append(__result[i][0])
            __sub_temp.append(__result[i][1])
            for j in range(__bottom_index_r+1,__top_index_r,1):
                __sub_temp.append(__right_vertex[j])
            if i==len(__result)-2:
                if __ispara==True:
                    __sub_temp.append(__result[i+1][1])
                    __sub_temp.append(__result[i+1][0])
                else:
                    __sub_temp.append(__result[i+1][1])
            else:
                __sub_temp.append(__result[i+1][1])
                __sub_temp.append(__result[i+1][0])
            for j in range(__top_index_l-1,__bottom_index_l,-1):
                __sub_temp.append(__left_vertex[j])
            if self.isdisp==True:
                __neo_sub=[]
                for k in range(len(__sub_temp)):
                    __temp_x=__sub_temp[k][0]+self.displace[0]
                    __temp_y=__sub_temp[k][1]+self.displace[1]
                    __neo_sub.append((__temp_x,__temp_y))
                __sub_temp=__neo_sub
            __sub_temp=self.coord_trans(self.base_vector(),__sub_temp)
            self.sub_point.append(__sub_temp)
            __sub_temp=[]
        if self.isdisp==True:
            __temp2=[]
            for i in range(len(__result)):
                __sub_result=[]
                for j in range(len(__result[i])):
                    __temp_x=__result[i][j][0]+self.displace[0]
                    __temp_y=__result[i][j][1]+self.displace[1]
                    __sub_result.append((__temp_x,__temp_y))
                __sub_result=self.coord_trans(self.base_vector(),__sub_result)
                __temp2.append(__sub_result)
            self.boundary=__temp2
        '''
        print("")
        print(self.sub_point)
        print("")
        print("sub boundary")
        print(self.boundary)    
        print("")
        '''
        return __result

    def UAV_input(self,s_radius,s_alpha,s_v,m_point):
        self.sub_radius=s_radius
        self.sub_alpha=s_alpha
        self.sub_v=s_v
        self.mission_point=m_point
        return

    def coverage(self) -> list:
        __result=[]
        for i in range(len(self.sub_point)):
            print(self.sub_point)
            print(self.sub_alpha)
            print(self.sub_radius)
            __point_temp=zigzagarea(self.sub_point[i])
            __point_temp.init(self.sub_radius[i],self.sub_alpha[i])
            __point_temp.min_width()
            __point_temp.base_vector()
            __point_temp.trans_solve()
            __point_temp.split()
            __point_temp.trans_desolve()
            __sub_result=__point_temp.point_sort(self.mission_point)
            __result.append(__sub_result)
            self.circle_direction_total.append(__point_temp.circle_direction)
            __sub_dist,__sub_time=__point_temp.time_cal(self.sub_v[i][0],self.sub_v[i][1])            #v_straight and v_arc
            self.dist_total.append(__sub_dist)
            self.time_total.append(__sub_time)
            self.turning_total.append(__point_temp.turning_cal())
            del __point_temp        
        return __result


class concavearea(zigzagarea):

    concave_index = []
    concave_point = []
    edge = []

    def __init__(self, point__input) -> None:

        super().__init__(point__input)
        self.check_concave()
        self.isconvex()
        self.line_segment_gene()
        print("self.point is {0}".format(self.point))
        print("line segment is {0}".format(self.edge))
        print("concave_point is {0}".format(self.concave_point))
        print("concave_index is {0}".format(self.concave_index))

    def line_segment_gene(self):
        self.edge = []
        _next_i = 0
        for i in range(len(self.point)):
            _next_i = i + 1
            if _next_i == len(self.point):
                _next_i = 0
            self.edge.append((self.point[i], self.point[_next_i]))

    # sort the point list by x coordinate
    def x_sort(self, arr):

        if len(arr) <= 1:
            return arr
        _pivot = arr[len(arr)//2]
        _left = []
        _middle = []
        _right = []
        for i in range(len(arr)):
            if arr[i][0] < _pivot[0]:
                _left.append(arr[i])
            elif arr[i][0] == _pivot[0]:
                _middle.append(arr[i])
            else:
                _right.append(arr[i])
        return self.x_sort(_left) + _middle + self.x_sort(_right)

    # sort the point list by y coordinate
    def y_sort(self, arr):

        if len(arr) <= 1:
            return arr
        _pivot = arr[len(arr)//2]
        _left = []
        _middle = []
        _right = []
        for i in range(len(arr)):
            if arr[i][1] < _pivot[1]:
                _left.append(arr[i])
            elif arr[i][1] == _pivot[1]:
                _middle.append(arr[i])
            else:
                _right.append(arr[i])
        return self.x_sort(_left) + _middle + self.x_sort(_right)        

    def GrahamHull(self, multi_connect = []):

        self.GrahamTemp = self.x_sort(self.point)        
        _hull = []
        is_concave = False
        # _next_i = 0

        # print("sorted by x, the point list is {0}".format(self.GrahamTemp))

        # generate lower part of hull
        for i in range(len(self.GrahamTemp)):

            if len(_hull) < 2:
                _hull.append(self.GrahamTemp[i])
            else:
                _vector1 = _hull[len(_hull)-1][0]-_hull[len(_hull)-2][0], _hull[len(_hull)-1][1]-_hull[len(_hull)-2][1]
                _vector2 = self.GrahamTemp[i][0]-_hull[len(_hull)-2][0], self.GrahamTemp[i][1]-_hull[len(_hull)-2][0]
                _cross = self.cross_product(_vector1, _vector2)

                '''
                print("vector 1 = {0}, vector2 = {1}".format(_vector1,_vector2))
                print("cross product is {0}".format(_cross))
                '''

                while _cross < 0 and len(_hull) >= 2:
                    _hull.pop()
                    if len(_hull) < 2:
                        continue
                    else:
                        _vector1 = _hull[len(_hull)-1][0]-_hull[len(_hull)-2][0], _hull[len(_hull)-1][1]-_hull[len(_hull)-2][1]
                        _vector2 = self.GrahamTemp[i][0]-_hull[len(_hull)-2][0], self.GrahamTemp[i][1]-_hull[len(_hull)-2][0]
                        _cross = self.cross_product(_vector1, _vector2)

                _hull.append(self.GrahamTemp[i])

                '''
                if _cross >= 0:
                    _hull.append(self.GrahamTemp[i])
                else:
                    _hull.pop()
                    if len(_hull) < 2:
                        _hull.append(self.GrahamTemp[i])
                '''
            '''        
            print("in {0}th execution, _hull = {1}, the next point is {2}".format(i, _hull, self.GrahamTemp[i]))
            print("")
            '''

        _lower_part = _hull
        _hull = []            
        self.GrahamTemp.reverse()

        # generate upper part of hull
        for i in range(len(self.GrahamTemp)):

            if len(_hull) < 2:
                _hull.append(self.GrahamTemp[i])
            else:
                _vector1 = _hull[len(_hull)-1][0]-_hull[len(_hull)-2][0], _hull[len(_hull)-1][1]-_hull[len(_hull)-2][1]
                _vector2 = self.GrahamTemp[i][0]-_hull[len(_hull)-2][0], self.GrahamTemp[i][1]-_hull[len(_hull)-2][0]
                _cross = self.cross_product(_vector1, _vector2)

                while _cross < 0 and len(_hull) >= 2:
                    _hull.pop()
                    if len(_hull) < 2:
                        continue
                    else:
                        _vector1 = _hull[len(_hull)-1][0]-_hull[len(_hull)-2][0], _hull[len(_hull)-1][1]-_hull[len(_hull)-2][1]
                        _vector2 = self.GrahamTemp[i][0]-_hull[len(_hull)-2][0], self.GrahamTemp[i][1]-_hull[len(_hull)-2][0]
                        _cross = self.cross_product(_vector1, _vector2)

                _hull.append(self.GrahamTemp[i])

        _upper_part = _hull

        print("upper part is {0}".format(_upper_part))
        print("lower part is {0}".format(_lower_part))

        _upper_part.pop()
        _upper_part.pop(0)

        _hull = _lower_part + _upper_part
        self.hull = _hull
        return _hull

    # Graham method
    '''
    def check_concave(self):
        self.concave_point = []
        self.concave_index = [0] * len(self.point)
        _hull = self.GrahamHull()
        if len(self.point) == len(_hull):
            self.is_concave = False
            return False
        elif len(self.point) < len(_hull):
            raise ValueError("Unknow error: Please check your input.")
        else:
            self.concave_point = set(self.point) - set(_hull)
            self.concave_point = list(self.concave_point)
            for i in range(len(self.concave_point)):
                _index = self.point.index(self.concave_point[i])
                self.concave_index[_index] = 1
            print("initiated successfully.")
            self.is_concave = True
            return True
    '''

    def check_concave(self, is_clockwise = 0):

        # default: not clockwise

        self.concave_point = []
        self.concave_index = [0] * len(self.point)
        self.is_concave = False

        for i in range(len(self.point)):
            if i == 0:
                _pre_index = len(self.point) - 1
                _next_index = i + 1
            elif i == len(self.point)-1:
                _pre_index = i - 1
                _next_index = 0
            else:
                _pre_index = i - 1
                _next_index = i + 1
            _vector1 = self.point[i][0] - self.point[_pre_index][0], self.point[i][1] - self.point[_pre_index][1]
            _vector2 = self.point[_next_index][0] - self.point[i][0], self.point[_next_index][1] - self.point[i][1]
            # _vector2 = self.point[_next_index][0] - self.point[_pre_index][0], self.point[_next_index][1] - self.point[_pre_index][1]
            # print()
            # print("vector1 = {0} vector2 = {1}".format(_vector1,_vector2))
            # print("cross_product is {0}".format(zigzagarea.cross_product(_vector1, _vector2)))
            # print()
            _x_product = zigzagarea.cross_product(_vector1, _vector2)
            if is_clockwise == 0:
                if _x_product < 0:
                    self.is_concave = True
                    self.concave_index[i] = 1
                    self.concave_point.append(self.point[i])
            else:
                if _x_product > 0:
                    self.is_concave = True
                    self.concave_index[i] = 1
                    self.concave_point.append(self.point[i])
        return self.is_concave

    def cut(self):  

        pass


    @staticmethod
    def inter(line1,line2):                         #intersection
        epsilon = 0.001
        # if (line2[0] == line2[1] or line1[0] == line1[1]):
        #     return (None, None), 2
        if line1[0][0]==line1[1][0] and line2[0][0]==line2[1][0]:
            x0=None
            y0=None
            if line1[0][0]!=line2[0][0]:
                isonline=2
            else:
                ratio1=(line2[0][1]-line1[0][1])/(line1[1][1]-line1[0][1])
                ratio2=(line2[1][1]-line1[0][1])/(line1[1][1]-line1[0][1])
                ratio3=(line1[0][1]-line2[0][1])/(line2[1][1]-line2[0][1])
                ratio4=(line1[1][1]-line2[0][1])/(line2[1][1]-line2[0][1])
                ratio=(ratio1,ratio2,ratio3,ratio4)
                point_temp=(line2[0],line2[1],line1[0],line1[1])
                counter=0
                flag=0
                for i in range(len(ratio)):
                    if ratio[i] == 0 or ratio[i] == 1:
                        temp_x=point_temp[i][0]
                        temp_y=point_temp[i][1]
                        counter+=1
                    if ratio[i]<1 + epsilon and ratio[i]>0 - epsilon:

                        flag=1
                if counter==2 and flag==0:
                    x0=temp_x
                    y0=temp_y
                    #print(ratio)
                    isonline=1
                elif counter==2 and flag==1:
                    isonline=4
                elif counter==0 and flag==1:
                    isonline=4
                else:
                    isonline=3
        elif line1[0][0]==line1[1][0]:
            k2=(line2[1][1]-line2[0][1])/(line2[1][0]-line2[0][0])
            b2=(line2[1][0]*line2[0][1]-line2[0][0]*line2[1][1])/(line2[1][0]-line2[0][0])
            x0=line1[0][0]
            y0=k2*x0+b2
            ratio1=(y0-line1[0][1])/(line1[1][1]-line1[0][1])
            ratio2=(x0-line2[0][0])/(line2[1][0]-line2[0][0])
            #ratio2=(y0-line2[0][1])/(line2[1][1]-line2[0][1])
            if ratio1<=1 + epsilon and ratio1>=0 - epsilon and ratio2<=1 + epsilon and ratio2>=0 - epsilon:
                isonline=1
            else:
                isonline=0
        elif line2[0][0]==line2[1][0]:
            k2=(line1[1][1]-line1[0][1])/(line1[1][0]-line1[0][0])
            b2=(line1[1][0]*line1[0][1]-line1[0][0]*line1[1][1])/(line1[1][0]-line1[0][0])
            x0=line2[0][0]
            y0=k2*x0+b2
            ratio1=(x0-line1[0][0])/(line1[1][0]-line1[0][0])
            # print line2
            print("-----------------line2: ({0},{1}), ({2},{3})", line2[0][0],line2[0][1],line2[1][0],line2[1][1])

            #ratio1=(y0-line1[0][1])/(line1[1][1]-line1[0][1])
            ratio2=(y0-line2[0][1])/(line2[1][1]-line2[0][1])
            if ratio1<=1 + epsilon and ratio1>=0 - epsilon and ratio2<=1 + epsilon and ratio2>=0 - epsilon:
                isonline=1
            else:
                isonline=0
        else:
            k1=(line1[1][1]-line1[0][1])/(line1[1][0]-line1[0][0])
            k2=(line2[1][1]-line2[0][1])/(line2[1][0]-line2[0][0])
            b1=(line1[1][0]*line1[0][1]-line1[0][0]*line1[1][1])/(line1[1][0]-line1[0][0])
            b2=(line2[1][0]*line2[0][1]-line2[0][0]*line2[1][1])/(line2[1][0]-line2[0][0])
            if abs(k1-k2)>0.001:
                x0=(b1-b2)/(k2-k1)
                y0=(b1*k2-b2*k1)/(k2-k1)
                ratio1=(x0-line1[0][0])/(line1[1][0]-line1[0][0])
                ratio2=(x0-line2[0][0])/(line2[1][0]-line2[0][0])
                print("ratio1: {0}, ratio2: {1}".format(ratio1,ratio2))
                if ratio1<=1 + epsilon and ratio1>=0 - epsilon and ratio2<=1 + epsilon and ratio2>=0 - epsilon:
                    isonline=1
                else:
                    isonline=0
            else:
                x0=None
                y0=None
                if abs(b2-b1)>0.001:
                    isonline=2
                else:
                    ratio1=(line2[0][0]-line1[0][0])/(line1[1][0]-line1[0][0])
                    ratio2=(line2[1][0]-line1[0][0])/(line1[1][0]-line1[0][0])
                    ratio3=(line1[0][0]-line2[0][0])/(line2[1][0]-line2[0][0])
                    ratio4=(line1[1][0]-line2[0][0])/(line2[1][0]-line2[0][0])
                    ratio=(ratio1,ratio2,ratio3,ratio4)
                    point_temp=(line2[0],line2[1],line1[0],line1[1])
                    counter=0
                    flag=0
                    for i in range(len(ratio)):
                        if ratio[i] == 0 or ratio[i] == 1:
                            temp_x=point_temp[int(i)][0]
                            temp_y=point_temp[i][1]
                            counter+=1
                        if ratio[i]<1 and ratio[i]>0:
                            flag=1
                    if counter==2 and flag==0:
                        x0=temp_x
                        y0=temp_y
                        isonline=1
                    elif counter==2 and flag==1:
                        isonline=4
                    elif counter==0 and flag==1:
                        isonline=4
                    else:
                        '''
                        print(counter)
                        print(flag)
                        print(ratio)
                        '''
                        isonline=3
        return (x0,y0),isonline

    '''
    isonline:
    0   intersection exists, but not on both line segments
    1   intersection of line segments
    2   parallel but not overlapping
    3   on the same line but not continuous
    4   overlap
    '''

    def isinner(self,line,shape):
        for i in range(len(shape)+1):
            if i!=len(shape):
                counter=i%len(shape)
                for j in range(len(shape)):
                    pass

        pass

    def cone_generation(self):
        pass

    # @staticmethod
    # def extension(area):
    #     _area = concavearea(area)
    #     if not _area.is_concave:
    #         raise ValueError("Invalid Input: the area is convex, not concave.")

    #     # 初始化子区域列表
    #     sub_area_left = []
    #     sub_area_right = []

    #     # 寻找凹点和计算切线交点
    #     for i in range(len(_area.point)):
    #         if _area.concave_index[i] == 1:
    #             # 获取凹点的前一个点和后一个点
    #             prev_point = _area.point[i - 1] if i > 0 else _area.point[-1]
    #             next_point = _area.point[(i + 1) % len(_area.point)]

    #             # 计算凹点的切线
    #             ray = (prev_point, _area.point[i])

    #             # 寻找凹点切线与多边形边的交点
    #             for j in range(len(_area.edge)):
    #                 edge = _area.edge[j]
    #                 intersection, isonline = _area.inter(ray, edge)

    #                 # 检查交点是否在边上并且不是无效点
    #                 if (
    #                     isonline == 1
    #                     and intersection is not None
    #                     and not (
    #                         math.isnan(intersection[0]) or math.isnan(intersection[1])
    #                     )
    #                 ):
    #                     # 将交点添加到子区域列表
    #                     sub_area_left.append(intersection)
    #                     sub_area_right.append(intersection)

    #                     # 添加子区域特有的点
    #                     sub_area_left.extend(
    #                         [_area.point[(i + 1) % len(_area.point)], next_point]
    #                     )
    #                     sub_area_right.extend(
    #                         [_area.point[i - 1 if i > 0 else -1], prev_point]
    #                     )

    #                     # 特殊情况处理：如果凹点两侧的边是平行的，选择更远的点作为子区域的一部分
    #                     if j == (i - 1) % len(_area.edge) and (
    #                         not _area.approximate(edge[0][0], edge[1][0], 0.001)
    #                     ):
    #                         sub_area_left.append(edge[1])
    #                         sub_area_right.append(edge[0])

    #                     break

    #     # 检查子区域列表是否有效
    #     if (
    #         not sub_area_left
    #         or not sub_area_right
    #         or any(
    #             math.isnan(point[0]) or math.isnan(point[1])
    #             for point in sub_area_left + sub_area_right
    #         )
    #     ):
    #         return None, None

    #     return sub_area_left, sub_area_right

    @staticmethod
    def extension(area):
        _area = concavearea(area)

        if not _area.is_concave:

            raise ValueError("Invalid Input: convex area.")

        else:

            if _area.is_concave == 1:

                for i in range(len(_area.point)):
                    _pre_index = i-1
                    if _pre_index == -1:
                        _pre_index = len(_area.point) - 1

                    #when ith point is a concave point
                    if _area.concave_index[i] == 1:

                        min_dist = 99999
                        pt_res = ()
                        marked_index = 0

                        _ray = (_area.point[_pre_index][0], _area.point[_pre_index][1]), (_area.point[i][0], _area.point[i][1])
                        _ray_length = math.sqrt((_ray[0][0]-_ray[1][0])*(_ray[0][0]-_ray[1][0]) + (_ray[0][1]-_ray[1][1])*(_ray[0][1]-_ray[1][1]))
                        #_ray = _area.point[i][0] - _area.point[_pre_index][0], _area.point[i][1] - _area.point[_pre_index][1]
                        _sub_area_left = []
                        _sub_area_right = []
 
                        #cross point search
                        # print("\nedge length is {0} \n".format(len(_area.edge)))
                        for j in range(len(_area.edge)):

                            print("ray is {0}, vector is {1}".format(_ray, _area.edge[j]))
                            _cross_point, _isonline_ray = _area.inter(_ray, _area.edge[j])
                            print("isonline_ray is {0}".format((_cross_point, _isonline_ray)))

                            if _isonline_ray == 0:

                                #_new_line = _cross_point[0] - _area.point[_pre_index][0], _cross_point[1] - _area.point[_pre_index][1]
                                _new_line = (_area.point[_pre_index][0], _area.point[_pre_index][1]), (_cross_point[0], _cross_point[1])
                                print("_new_line, _area.edge[j] is {0}".format((_new_line, _area.edge[j])))
                                if (abs(_new_line[0][0]-_new_line[1][0]) < 0.001 * _ray_length) and (abs(_new_line[0][1]-_new_line[1][1]) < 0.001 * _ray_length):
                                    print("**marked**") 
                                    continue
                                _cross_point_new, _isonline_new = _area.inter(_new_line, _area.edge[j])
                                print("_isonline_new is {0}".format(_isonline_new))

                                if _isonline_new == 1:
                                    
                                    cur_dist = math.sqrt((_cross_point_new[0]-_area.point[i][0])*(_cross_point_new[0]-_area.point[i][0]) + (_cross_point_new[1]-_area.point[i][1])*(_cross_point_new[1]-_area.point[i][1]))
                                    print("\n cur_dist is {0}\n", cur_dist)
                                    if cur_dist < min_dist:

                                        min_dist = cur_dist
                                        pt_res = _cross_point_new
                                        marked_index = j
                        

                        _sub_area_left.append(_area.point[_pre_index])
                        _sub_area_left.append(pt_res)
                        _next_index = marked_index + 1
                        if _next_index == len(_area.edge):
                            _next_index = 0
                        while _next_index != _pre_index:
                            _sub_area_left.append(_area.point[_next_index])
                            _next_index += 1
                            if _next_index == len(_area.edge):
                                _next_index = 0

                        _sub_area_right.append(_area.point[i])
                        _next_index = i + 1
                        if _next_index == len(_area.edge):
                            _next_index = 0
                        _flag = marked_index + 1 if marked_index + 1 != len(_area.edge) else 0
                        while _next_index != _flag:
                            _sub_area_right.append(_area.point[_next_index])
                            _next_index += 1
                            if _next_index == len(_area.edge):
                                _next_index = 0
                        _sub_area_right.append(pt_res)

                                    # _sub_area_left.append(_area.point[_pre_index])
                                    # _sub_area_left.append(_cross_point_new)
                                    # _next_index = j + 1
                                    # if _next_index == len(_area.edge):
                                    #     _next_index = 0
                                    # while _next_index != _pre_index:
                                    #     _sub_area_left.append(_area.point[_next_index])
                                    #     _next_index += 1
                                    #     if _next_index == len(_area.edge):
                                    #         _next_index = 0

                                    # _sub_area_right.append(_area.point[i])
                                    # _next_index = i + 1
                                    # if _next_index == len(_area.edge):
                                    #     _next_index = 0
                                    # _flag = j + 1 if j + 1 != len(_area.edge) else 0
                                    # while _next_index != _flag:
                                    #     _sub_area_right.append(_area.point[_next_index])
                                    #     _next_index += 1
                                    #     if _next_index == len(_area.edge):
                                    #         _next_index = 0
                                    # _sub_area_right.append(_cross_point_new)

                                    # '''
                                    # for k in range(len(_sub_area_right)):
                                    #     _next_index = k + 1 if k + 1 != len(_sub_area_right) else 0
                                    #     if zigzagarea.approximate(_sub_area_right[k][0], _sub_area_right[_next_index][0])\
                                    #           and zigzagarea.approximate(_sub_area_right[k][1], _sub_area_right[_next_index][1]):
                                    #         _flag = k

                                    # _sub_area_right.pop(k)
                                    # '''

                                    # return _sub_area_left, _sub_area_right
                        return _sub_area_left, _sub_area_right


    def voronoi(self):
        pass

    def sweep(self):
        pass


# MC = Multi-Connected
class MCarea(concavearea):

    outline = []
    point = []
    MC_point = []
    MC_sum = 0

    def __init__(self, point_input, MC_sum = 0) -> None:
        self.point = point_input
        self.outline = point_input
        self.MC_sum = MC_sum
    
    def reshape(self, MC_sum):
        self.MC_sum = MC_sum
    
    def add_area(self, area):
        self.MC_point.append(area)

    def update(self, MC_input):
        self.MC_point = []
        if len(MC_input) != self.MC_sum:
            raise ValueError("Incorrect number of multi-connected areas!")
        for i in range(len(MC_input)):
            self.MC_point.append(MC_input[i])
        return self.MC_point

    def deMC(self):
        for i in range(len(self.MC_point)):
            pass
        pass


class AreaNode:
    def __init__(self, area=None, left=None, right=None, height=0, label=None) -> None:
        self.area=area
        self.left=left
        self.right=right
        self.height=height
        self.leble=label

class AreaTree:

    root = None

    def __init__(self, root):
        self.root=root
        print("root is {0}".format(self.root))

    def grow(self, node, left = None, right = None):
        node.left = left
        node.right = right

    def find_leaves(self, root):
        leaves = []

        def traverse(node):
            if not node:
                return 
            if not node.left and not node.right:
                leaves.append(node)
            traverse(node.left)
            traverse(node.right)

        traverse(root)

        return leaves

    # def leaves_concave(self):
    #     # 先找到所有的叶子节点
    #     _leaves = self.find_leaves(self.root)
    #     # 检查叶子节点是否为 None 或者 空列表
    #     if not _leaves:
    #         return False  # 如果没有叶子节点，返回 False 表示没有叶子是凹的

    #     for leaf in _leaves:
    #         # 检查叶子的区域是否为 None
    #         if leaf.area is None:
    #             continue  # 如果叶子的区域是 None，跳过当前叶子继续检查

    #         c = concavearea(leaf.area)  # 创建 concavearea 实例
    #         if c.is_concave == 1:  # 如果叶子是凹的
    #             return True  # 返回 True 表示至少有一个叶子是凹的

    #     return False  # 如果所有叶子都是凸的，返回 False

    def leaves_concave(self):
        _leaves = self.find_leaves(self.root)
        print("leaves are ... {0}".format(_leaves))
        for i in _leaves:
            c = concavearea(i.area)
            if c.is_concave == 1:
                return True                     #leaves are concave
        return False                            #leaves are convex

    def convex_tree_generation(self, issimp = 1):

        c = []
        _leaves = []
        _flag = []
        _area_left = []
        _area_right = []
        if issimp:
            _leaves = self.find_leaves(self.root)
            _flag = self.leaves_concave()
            print("_flag is {0}".format(_flag))
            while _flag:
                for i in range(len(_leaves)):
                    c = concavearea(_leaves[i].area)
                    print("c.point is {0}".format(c.point))
                    if c.is_concave == True:
                        _area_left, _area_right = concavearea.extension(c.point)
                        print()
                        print("Area_left is {0}".format(_area_left))
                        print("Area_right is {0}".format(_area_right))
                        print()
                        _node_left = AreaNode(_area_left)
                        _node_right = AreaNode(_area_right)
                        _leaves[i].left = _node_left
                        _leaves[i].right = _node_right
                _flag = self.leaves_concave()
                _leaves = self.find_leaves(self.root)

    def traversal(self, root):
        if root:
            print(root.area)
            self.traversal(root.left)
            self.traversal(root.right)

        '''
        _stack = []
        _node = self.root
        while _stack or _node:
            while _node:
                _stack.append(_node)
                _node = _node.left
            _node = _stack.pop() 
        '''
        pass

    def search(self):
        pass

    def sort(self):
        pass

    def pre_order(self,root):
        if root:
            pass
        self.pre_oder(root.left)
        self.pre_oder(root.right)

    def insert(self, val, label):

        '''
        def _insert(root, val, label):
            if root is None:
                return Node(val=val, label=label)
            
            #label=1: concave
            #label=2: convex
            if root.label==1:
        '''

    pass

##Please input:

def plot_single_cpp(coords,radius,alpha,v_straight,v_arc,start_point,DataPath='Output.txt',FigurePath='Coverage Route.png',para='11111'):
    a = coords
    fig = plt.figure()
    ax = fig.add_subplot(111)
    b = zigzagarea(a)
    b.init(radius, alpha)
    x0 = []
    y0 = []
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    x3 = []
    y3 = []
    x4 = []
    y4 = []
    #print(b.min_width())
    #print(b.isconvex())
    c = b.base_vector()
    d = b.trans_solve()
    e = b.split()
    f = b.trans_desolve()
    g = b.point_sort(start_point)

    Dis_total_S = b.time_cal(v_straight, v_arc)[0]
    Time_total_T = b.time_cal(v_straight, v_arc)[1]
    Turning_total_M = b.turning_cal()
    Route_coords = g

    DataSave(Dis_total_S,Turning_total_M,Time_total_T,Route_coords,DataPath,para)

    '''
    h2 = open("single_agent_evaluation.txt", 'w+')
    h2.writelines(str(1) + '--' + '总路程:' + str(Dis_total_S)+ '\n')
    h2.writelines(str(2) + '--' + '总耗时:' + str(Time_total_T ) + '\n')
    h2.writelines(str(3) + '--' + '总转弯数:' + str(Turning_total_M) + '\n')
    h2.close()

    q2 = open("signle_agent_cpp_routes.txt", 'w+')
    q2.writelines('UAV_' + '路径点'+ '\n')
    index_j=0
    while index_j < len(Route_coords):
            q2.writelines(str(Route_coords[index_j])+ '\n')
            index_j = index_j+1
    q2.close()

    
    print('Dis_total_S:', b.time_cal(v_straight, v_arc)[0])
    print('Time_total_T:', b.time_cal(v_straight, v_arc)[1])
    print('Turn_total_M:',b.turning_cal())
    #print(f)
    print('Route_coords:',g)
    ##print('lengthxxd',len(g))
    '''

    for i in d:
        x1.append(i[0])
        y1.append(i[1])
    x1.append(d[0][0])
    y1.append(d[0][1])
    for i in a:
        x0.append(i[0])
        y0.append(i[1])
    x0.append(a[0][0])
    y0.append(a[0][1])
    for i in range(2):
        for j in range(len(e[i])):
            x2.append(e[i][j][0])
            y2.append(e[i][j][1])
    for i in range(2):
        for j in range(len(f[i])):
            x3.append(f[i][j][0])
            y3.append(f[i][j][1])
    for i in range(len(g)):

        if i == 0 or i == 1 or i == len(g) - 1 or i == len(g) - 2:
            x4.append(g[i][0])
            y4.append(g[i][1])
        elif i % 2 == 0:
            if (i / 2) % 2 == 1:
                if b.circle_direction == 0:
                    temp_point = b.arc(g[i], g[i + 1])
                else:
                    temp_point = b.arc(g[i + 1], g[i])
                    temp_point = b.inv_list(temp_point)
            else:
                if b.circle_direction == 0:
                    temp_point = b.arc(g[i + 1], g[i])
                    temp_point = b.inv_list(temp_point)
                else:
                    temp_point = b.arc(g[i], g[i + 1])
            for j in range(len(temp_point) - 1):
                x4.append(temp_point[j][0])
                y4.append(temp_point[j][1])
        else:
            x4.append(g[i][0])
            y4.append(g[i][1])

    temp = len(x4)
    ax.set(title='Coverage Route', ylabel='Y-Axis', xlabel='X-Axis')

    plt.plot(x4[0], y4[0], "o", label='Zero point')
    plt.plot(x4[1], y4[1], "o", label='Coverage start point')
    plt.plot(x4[len(x4) - 2], y4[len(y4) - 2], "o", label='Coverage end point')
    '''
    ax.arrow(x4[0],y4[0],x4[1]-x4[0],y4[1]-y4[0],width=0.15,head_width=1.0,head_length=1.4,length_includes_head=True,fc='b',ec='b')
    ax.arrow(x4[temp-2],y4[temp-2],x4[temp-1]-x4[temp-2],y4[temp-1]-y4[temp-2],width=0.15,head_width=1.0,\
        head_length=1.4,length_includes_head=True,fc='b',ec='b')
    '''
    ax.arrow(x4[0], y4[0], x4[1] - x4[0], y4[1] - y4[0], width=0.04, head_width=0.25, head_length=0.35,
             length_includes_head=True, fc='b', ec='b')
    ax.arrow(x4[temp - 2], y4[temp - 2], x4[temp - 1] - x4[temp - 2], y4[temp - 1] - y4[temp - 2], width=0.04,
             head_width=0.25, \
             head_length=0.35, length_includes_head=True, fc='b', ec='b')

    # when too slim try doubling the width ↑

    x4.pop(0)
    x4.pop(len(x4) - 1)

    y4.pop(0)
    y4.pop(len(y4) - 1)
    plt.plot(x4, y4, "--")

    # Show local coordinate system: width=0.01,head_width=0.2,head_length=0.3,length_includes_head=True,fc='b',ec='b'
    '''
    print(b.point_line)
    k=(b.point_line[1][1][0]-b.point_line[1][0][0])/(b.point_line[1][1][1]-b.point_line[1][0][1])
    start_x=(b.point_line[1][0][0]+k*k*b.point_line[0][0]+k*(b.point_line[0][1]-b.point_line[1][0][1]))/(1+k*k)
    start_y=-(k*(b.point_line[1][0][0]-b.point_line[0][0])-b.point_line[0][1]-k*k*b.point_line[1][0][1])/(1+k*k)

    ax.arrow(b.point_line[1][0][0],b.point_line[1][0][1],b.point_line[1][1][0]-b.point_line[1][0][0],b.point_line[1][1][1]-b.point_line[1][0][1]\
        ,width=0.15,head_width=1.0,head_length=1.4,length_includes_head=True,fc='r',ec='r')
    ax.arrow(start_x,start_y,b.point_line[0][0]-start_x,b.point_line[0][1]-start_y,width=0.15,head_width=1.0,\
        head_length=1.4,length_includes_head=True,fc='r',ec='r')
    plt.plot(start_x,start_y,"o",label='Zero Point of Local Sytem')
    '''

    plt.plot(x0, y0, "orange")

    ax.grid(True)
    plt.axis("equal")
    plt.legend(loc='upper left', frameon=True)
    
    if para[1] == '1': 
        plt.savefig(FigurePath,dpi=300)
    
    plt.show()

#############

def plot_multi_cpp(coords,UAV_num,sep_area,radius,alpha,v,start_point,DataPath='output.txt',FigurePath='Coverage Route.png',para='11111'):
    a = coords
    fig = plt.figure()
    ax = fig.add_subplot(111)
    b = totalarea(a)
    b.min_width()
    b.base_vector()
    ab = b.trans_solve()
    b.area_sub(a, UAV_num,sep_area)
    c = b.area_split()
    b.UAV_input(radius, alpha, v, start_point)
    d = b.coverage()
    Route_coords = d
    Dis_total_S = b.dist_total
    Time_total_T =  b.time_total
    Turning_total_M = b.turning_total
    '''
    print('Route_coords:',d)
    print('Distance_total_S:',b.dist_total)
    print('Time_total_T:', b.time_total)
    print('Turning_total_M:',b.turning_total)
    '''
    print('MultiUAV'+para+'\n')
    DataSave(Dis_total_S, Turning_total_M, Time_total_T, Route_coords, DataPath, para)
    '''
    f2 = open("multi_agent_evaluation.txt", 'w+')
    f2.writelines(str(1) + '--' + '总路程:' + str(Dis_total_S)+ '\n')
    f2.writelines(str(2) + '--' + '总耗时:' + str(Time_total_T ) + '\n')
    f2.writelines(str(3) + '--' + '总转弯数:' + str(Turning_total_M) + '\n')
    f2.close()

    index_i = 0
    print('ss', len(Route_coords[0]))
    g2 = open("multi_agent_cpp_routes.txt", 'w+')
    while index_i < UAV_num:
        g2.writelines('UAV_' + str(index_i + 1)+'路径点'+ '\n')
        index_j=0
        while index_j < len(Route_coords[index_i]):
            g2.writelines(str(Route_coords[index_i][index_j])+ '\n')
            index_j = index_j+1
        index_i = index_i+1
    g2.close()
    '''

    x0 = []
    y0 = []
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    x3 = []
    y3 = []
    x4 = []
    y4 = []
    x5 = []
    y5 = []

    for i in range(len(c)):
        if i != 0 and i != len(c) - 1:
            x0.append(c[i][0][0])
            x0.append(c[i][1][0])
            y0.append(c[i][0][1])
            y0.append(c[i][1][1])
            x2.append((c[i][0][0], c[i][1][0]))
            y2.append((c[i][0][1], c[i][1][1]))
            x5.append((b.boundary[i][0][0], b.boundary[i][1][0]))
            y5.append((b.boundary[i][0][1], b.boundary[i][1][1]))

    for i in a:
        x4.append(i[0])
        y4.append(i[1])
    x4.append(a[0][0])
    y4.append(a[0][1])

    for i in ab:
        x1.append(i[0])
        y1.append(i[1])
    x1.append(ab[0][0])
    y1.append(ab[0][1])
    ax.set(title='Coverage Route', ylabel='Y-Axis', xlabel='X-Axis')
    
    # ax.set(title='Coverage Route', ylabel='Y-Axis', xlabel='X-Axis')
    # plt.plot(x1,y1)

    __counter = 0
    for i in d:
        x3_temp = []
        y3_temp = []
        for j in range(len(i)):
            if j == 0 or j == 1 or j == len(i) - 1 or j == len(i) - 2:
                x3_temp.append(i[j][0])
                y3_temp.append(i[j][1])
            elif j % 2 == 0:
                if (j / 2) % 2 == 1:
                    if b.circle_direction_total[__counter] == 0:
                        temp_point = b.arc(i[j], i[j + 1])
                    else:
                        temp_point = b.arc(i[j + 1], i[j])
                        temp_point = b.inv_list(temp_point)
                else:
                    if b.circle_direction_total[__counter] == 0:
                        temp_point = b.arc(i[j + 1], i[j])
                        temp_point = b.inv_list(temp_point)
                    else:
                        temp_point = b.arc(i[j], i[j + 1])
                for k in range(len(temp_point) - 1):
                    x3_temp.append(temp_point[k][0])
                    y3_temp.append(temp_point[k][1])
            else:
                x3_temp.append(i[j][0])
                y3_temp.append(i[j][1])
        __counter += 1
        x3.append(x3_temp)
        y3.append(y3_temp)

    plt.plot(x4, y4)
    # plt.plot(x3[0][0],y3[0][0],"o",label='zero point')

    for i in range(len(d)):
        temp = len(x3[i])

        ax.arrow(x3[i][0], y3[i][0], x3[i][1] - x3[i][0], y3[i][1] - y3[i][0], width=0.01, head_width=0.2,
                 head_length=0.3, \
                 length_includes_head=True, fc='b', ec='b')
        ax.arrow(x3[i][temp - 2], y3[i][temp - 2], x3[i][temp - 1] - x3[i][temp - 2], y3[i][temp - 1] - y3[i][temp - 2],
                 width=0.01, head_width=0.2, \
                 head_length=0.3, length_includes_head=True, fc='b', ec='b')

        plt.plot(x3[i][1], y3[i][1], "o")  # , label='coverage start point')
        plt.plot(x3[i][len(x3[i]) - 2], y3[i][len(y3[i]) - 2], "o")  # ,label='coverage end point')
        x3[i].pop()
        y3[i].pop()
        x3[i].pop(0)
        y3[i].pop(0)
        plt.plot(x3[i], y3[i], '--')

    # for i in range(len(x0)):
    #    plt.plot(x0[i],y0[i],"o")

    for i in range(len(x2)):
        plt.plot(x5[i], y5[i], "r")

    ax.grid(True)
    plt.axis("equal")
    plt.legend(loc='upper left', frameon=True)
    #
    if para[1] == '1':
        plt.savefig(FigurePath,dpi=300)
    #
    plt.show()


def DataSave(dist_net, turning, time, route, path='output.txt', para='00000'):
    
    if para[0] == '0':
        return 0
    elif re.match('[01][01][01][01][01]', para) == None:
        raise Exception('Wrong Parameter!')
    else:
        print('excuted\n')
        f = open(path,'w',encoding='UTF-8')
        if para[2] == '1':
            f.write('\nTotal Distance (Task Only):\n' + str(dist_net) + '\n')
        if para[3] == '1':
            f.write('\nTotal Number of Turning Maneuvers:\n' + str(turning) + '\n')
        if para[4] == '1':
            f.write('\nTotal Time:\n' + str(time) + '\n')
        f.write('\nRoute Points:\n')
        if zigzagarea.depth(route) == 2:
            for i in range(len(route)):
                f.write(str(route[i])+'\n')
        elif zigzagarea.depth(route) == 3:
            for i in range(len(route)):
                f.write('UAV{}'.format(i+1)+'\n')
                for j in range(len(route[i])):
                    f.write(str(route[i][j])+'\n')
                f.write('\n')
        f.close()

def plot_concave_cpp(area, radius, alpha, v_straight, v_arc, start_point, datapath = './', figurepath = './', para = '11111'):
    node = AreaNode(area)
    tree = AreaTree(node)
    tree.convex_tree_generation()
    sub_areas = tree.find_leaves(tree.root)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set(title='Coverage Route', ylabel='Y-Axis', xlabel='X-Axis')
    for i in sub_areas:
        mem = i.area
        x0, y0, x1, y1, x2, y2, x3, y3, x4, y4 = [], [], [], [], [], [], [], [], [], []
        mem_sgl = zigzagarea(mem)
        mem_sgl.init(radius, alpha)
        c = mem_sgl.base_vector()
        d = mem_sgl.trans_solve()
        e = mem_sgl.split()
        f = mem_sgl.trans_desolve()
        total_dist = mem_sgl.point_sort(start_point)

        Dis_total_S, Time_total_T = mem_sgl.time_cal(v_straight, v_arc)
        Turning_total_M = mem_sgl.turning_cal()
        Route_coords = total_dist

        for i in d:
            x1.append(i[0])
            y1.append(i[1])
        x1.append(d[0][0])
        y1.append(d[0][1])
        for i in mem:
            x0.append(i[0])
            y0.append(i[1])
        x0.append(mem[0][0])
        y0.append(mem[0][1])
        for i in range(2):
            for j in range(len(e[i])):
                x2.append(e[i][j][0])
                y2.append(e[i][j][1])
        for i in range(2):
            for j in range(len(f[i])):
                x3.append(f[i][j][0])
                y3.append(f[i][j][1])
        for i in range(len(total_dist)):

            if i == 0 or i == 1 or i == len(total_dist) - 1 or i == len(total_dist) - 2:
                x4.append(total_dist[i][0])
                y4.append(total_dist[i][1])
            elif i % 2 == 0:
                if (i / 2) % 2 == 1:
                    if mem_sgl.circle_direction == 0:
                        temp_point = mem_sgl.arc(total_dist[i], total_dist[i + 1])
                    else:
                        temp_point = mem_sgl.arc(total_dist[i + 1], total_dist[i])
                        temp_point = mem_sgl.inv_list(temp_point)
                else:
                    if mem_sgl.circle_direction == 0:
                        temp_point = mem_sgl.arc(total_dist[i + 1], total_dist[i])
                        temp_point = mem_sgl.inv_list(temp_point)
                    else:
                        temp_point = mem_sgl.arc(total_dist[i], total_dist[i + 1])
                for j in range(len(temp_point) - 1):
                    x4.append(temp_point[j][0])
                    y4.append(temp_point[j][1])
            else:
                x4.append(total_dist[i][0])
                y4.append(total_dist[i][1])
        temp = len(x4)
        plt.plot(x4[0], y4[0], "o")
        plt.plot(x4[1], y4[1], "o")
        plt.plot(x4[len(x4) - 2], y4[len(y4) - 2], "o")
        # ax.arrow(x4[0], y4[0], x4[1] - x4[0], y4[1] - y4[0], width=0.04, head_width=0.25, head_length=0.35,
        #      length_includes_head=True, fc='b', ec='b')
        # ax.arrow(x4[temp - 2], y4[temp - 2], x4[temp - 1] - x4[temp - 2], y4[temp - 1] - y4[temp - 2], width=0.04,
        #      head_width=0.25, \
        #      head_length=0.35, length_includes_head=True, fc='b', ec='b')

        x4.pop(0)
        x4.pop(len(x4) - 1)

        y4.pop(0)
        y4.pop(len(y4) - 1)
        plt.plot(x4, y4, "--")
        plt.plot(x0, y0, "orange")
    ax.grid(True)
    plt.axis("equal")
    plt.legend(loc='upper left', frameon=True)
    if para[1] == '1':
        plt.savefig(figurepath,dpi=300)
    plt.show()









'''
##Please input:

UAV_num = 1

if UAV_num == 1:
    ###single UAV CPP INPUT AND SOULTION
    coords = [(2, 2), (5, 2), (6, -1), (1, -3), (0, -1)]
    radius = 0.6
    alpha = 0.08
    v_straight = 0.9
    v_arc = 0.75
    start_point = (4, -3)
    mat = [[-3, -4], [3.2, -2.4]]
    plot_single_cpp(coords, radius, alpha, v_straight, v_arc, start_point, mat)
else:
    ###Multi UAV CPP INPUT AND SOULTION
    coords = [(1, 1), (5, 2), (7, 2), (9, -2)]
    radius = (0.5, 0.35, 0.4)
    alpha = (0.1, 0.05, 0.15)
    v = ((2, 0.5), (2.5, 0.5), (3.0, 1.5))
    start_point = 
    sep_area = (0.25, 0.25, 0.5)
    plot_multi_cpp(coords, UAV_num, sep_area, radius, alpha, v, start_point)

'''

# if __name__ == '__main__':
#     coords = [(113.18, 34.33), (113.10, 34.31), (113.18, 34.38), (113.22, 34.32), (113.34, 34.37), (113.24, 34.31), (113.23, 34.24), (113.19, 34.31), (113.16, 34.23), (113.08, 34.26), (113.18, 34.33)]
#     a, b = concavearea.extension(coords)
#     print(a)
#     print(b)
#     plot_single_cpp(a, 0.02, 0.2, 0.05, 0.025, a[0])
