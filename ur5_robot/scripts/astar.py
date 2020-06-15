#!/usr/bin/env python
import rospy as rp
import numpy as np
import heapq as pq
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from copy import copy
import pickle
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from ur5_robot.srv import TfPair
import math


mapa = []
mapa2=[]
start1=(0,0,0)
class ASTAR():
    def __init__(self):
        self.map = None
        self.start = (0.2, -0.1, 0.1)
        self.start2 = (0.2, -0.1, 0.1)
        self.end = (0.4, -0.6, 0.4)
        #self.end2 = [(0.278, -0.538, 0.350 + 0.1), (0.305, -0.934, 0.359 + 0.1), (0.072, -0.712, 0.097 + 0.1),
       #              (-0.091, -0.911, 0.222 + 0.1), (-0.225, -0.528, 0.256 + 0.1)]
        self.end2 = [(0.2, -0.5, 0.3 + 0.1), (0.3, -0.9, 0.3 + 0.1), (0.1, -0.7, 0.1 + 0.1),
                                   (-0.1, -0.9, 0.3 + 0.1), (-0.2, -0.5, 0.3 + 0.1)]
        self.i = 0
        rp.init_node('graph_search')
        self.pub = rp.Publisher('bfs', OccupancyGrid, queue_size=10)
        self.path_pub = rp.Publisher('path', Path, queue_size=10)
        print("Object initialized!")

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for p in path:
            rp.logerr(p)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header.frame_id = 'map'
            pose.header.stamp = rp.Time.now()
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def heuristics(self, pos):
        distance = abs(self.end[0] - pos[0]) + abs(self.end[1] - pos[1])+ abs(self.end[2] - pos[2])
        return distance

    def distancexy(self, pos):
        if(abs(pos[0])>0 and abs(pos[1])>0 and abs(pos[2]) == 0) or (abs(pos[0])==0 and abs(pos[1])>0 and abs(pos[2]) > 0) or (abs(pos[0])>0 and abs(pos[1])==0 and abs(pos[2]) > 0):
            distance=1.414
        elif (abs(pos[0]) > 0 and abs(pos[1]) > 0 and abs(pos[2]) > 0):
            distance = 1.2
        else:
            distance=1.0

        return distance

    def make_path(self, came_from, current):
        a = round(current[0], 1)
        b = round(current[1], 1)
        c = round(current[2], 1)
        current=(a,b, c)
        path=[current]
        while current!=start1:
            current=came_from[current]
            path.append(current)
        print(path)
        path.reverse()

        return path

    def check_if_valid(self, a):
        global mapa, mapa2
        in_free_space=True
        for o in range(0,len(mapa2)):
            if round(a[0],1) == round(mapa2[o][2],1) and round(a[1],1) == round(mapa2[o][1],1):
                index=o
                if round(a[2],1) <=  round(mapa2[index][2],1):
                    in_free_space = False
                    break
                else:
                    in_free_space = True
                    break
        return in_free_space


    def search(self):

        global mapa
        global mapa2, start1
        mapa = []
        mapa2 = []
        tBool = False

        if self.i > len(self.end2) - 1:
            self.i = 0
        self.end = self.end2[self.i]
        if self.i > 0:
            self.start = self.end2[self.i - 1]
        else:
            self.start = self.end2[len(self.end2) - 1]

        rp.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback)
        rp.sleep(0.2)
        mapa2 = list(mapa)

        try:
            with open("data.pkl", "rb") as a_file:
                pathtemp = pickle.load(a_file)
        except:
            pathtemp = []
        if pathtemp==[]:
            tBool=True

        closedset =[]
        neighbours = [(0, 0.1, 0), (0.1, 0, 0), (0, -0.1, 0), (-0.1, 0, 0), (0, 0, 0.1), (0, 0,-0.1),
                     (0.1, 0, 0.1), (0, -0.1, 0.1), (0, 0.1, 0.1), (-0.1, 0, 0.1),
                     (0.1, 0, -0.1), (0, -0.1, -0.1), (0, 0.1, -0.1), (-0.1, 0, -0.1),
                      (0.1, 0.1, 0), (0.1, -0.1, 0), (-0.1, 0.1, 0), (-0.1, -0.1, 0),
                      (0.1, 0.1, 0.1), (0.1, -0.1, 0.1), (-0.1, 0.1, 0.1), (-0.1, -0.1, 0.1),
                      (0.1, 0.1, -0.1), (0.1, -0.1, -0.1), (-0.1, 0.1, -0.1), (-0.1, -0.1, -0.1),]
        a = round(self.start[0], 1)
        b = round(self.start[1], 1)
        c = round(self.start[2], 1)
        start1=(a,b, c)
        came_from={start1:start1}
        #came_from={}
        openset=[start1]

        g={}
        f = {}
        f_open = {}
        h={}
        g[start1]=0
        h[start1]=self.heuristics(start1)
        f[start1]=g.get(start1)+h.get(start1)
        f_open[start1]=f.get(start1)
        if tBool:
            while not len(openset)<0:
                x=min(f_open, key=f_open.get)
                a = round(x[0], 1)
                b = round(x[1], 1)
                c = round(x[2], 1)
                x=(a, b, c)
                del f_open[x]
                #self.map.data[col + row * self.map.info.width] = 50
                if x==self.end:
                    break

                openset.remove(x)
                closedset.append(x)
                for neigh in neighbours:
                    temp = (round(x[0] + neigh[0],1), round(x[1] + neigh[1],1), round(x[2] + neigh[2],1))
                    if self.check_if_valid(temp) and temp not in closedset:
                        #self.map.data[col + neigh[0] + (row + neigh[1]) * self.map.info.width]=10
                        if temp in closedset:
                            pass
                        g_temp=g.get(x)+self.distancexy(neigh)
                        temp_is_better=False
                        if not temp in closedset:
                            openset.append(temp)
                            h[temp]=self.heuristics(temp)
                            temp_is_better=True
                        elif g_temp<g[temp]:
                            temp_is_better=True
                        if temp_is_better:
                            came_from[temp]=x
                            g[temp]=g_temp
                            f[temp]=g_temp+h.get(temp)
                            f_open[temp]=g_temp+h.get(temp)

            print("done")
            path=self.make_path(came_from, self.end)
            self.publish_path(path)
            a_file = open("data.pkl", "wb")
            pickle.dump(path, a_file)
            a_file.close()
            self.i += 1

def show_distance_client( x, y):
    rp.wait_for_service('distance_between_tfs')
    try:
        euklid = rp.ServiceProxy('distance_between_tfs', TfPair)
        resp1 = euklid(x, y)
        return resp1.data0, resp1.data1, resp1.data2
    except rp.ServiceException:
        print("Service call faileed:")

def callback(cloud):
    assert isinstance(cloud, PointCloud2)
    global mapa
    gen = point_cloud2.read_points(cloud, skip_nans=True)
    xkw, ykw, zkw = show_distance_client('world', 'kinect_link')
    for p in gen:
        mapa.append(p)

if __name__ == '__main__':
    dfs = ASTAR()
    while True:
        dfs.search()
