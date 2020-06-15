#!/usr/bin/env python
import rospy as rp
#from grid_map import GridMap
import numpy as np
import random
import math
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from ur5_robot.srv import TfPair
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Point32
import pickle
#from random import random


np.random.seed(444)
mapa = []
mapa2=[]
class RRT():
    def __init__(self):
        #super(RRT, self).__init__()
        rp.init_node('graph_search', log_level=rp.DEBUG)
        self.start=(0.2, -0.1, 0.1)
        self.start2 = (0.2, -0.1, 0.1)
        self.end=(0.4, -0.6, 0.4)
        self.end2=[(0.278, -0.538, 0.350+0.1), (0.305, -0.934, 0.359+0.1),(0.072, -0.712, 0.097+0.1), (-0.091, -0.911, 0.222+0.1), (-0.225, -0.528, 0.256+0.1)]
        self.parent={}
        self.resolution=100
        self.i=0
        self.path_pub = rp.Publisher('path', Path, queue_size=10)
        self.search_pub = rp.Publisher('search', Marker, queue_size=10)
        #self.pcl_pub =rp.Publisher('pcl_check', PointCloud, queue_size=10)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for p in path:
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
    def publish_search(self):
        marker = Marker()

        def add_point(p):
            pt = Point()
            pt.x = p[0]
            pt.y = p[1]
            pt.z = p[2]
            marker.points.append(pt)

        marker.header.frame_id = "map"
        marker.header.stamp = rp.Time.now()
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.color.r = 0.
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 0.5
        marker.scale.x = 0.01 * self.resolution
        for k, v in self.parent.items():
            if v is None: continue
            add_point(k)
            add_point(v)
        self.search_pub.publish(marker)
    def check_if_valid(self, a, b):
        global mapa, mapa2
        step=30
        in_free_space=True
        x=np.linspace(a[0], b[0], num=step, endpoint=True)
        y =np.linspace(a[1], b[1], num=step, endpoint=True)
        z = np.linspace(a[2], b[2], num=step, endpoint=True)
        print("checkifvalis")
        print(len(mapa2))
        for i in range (0, step):
            for o in range(0,len(mapa2)):
                if round(x[i],1) == round(mapa2[o][2],1) and round(y[i],1) == round(mapa2[o][1],1):
                    index=o
                    if round(z[i],1) <=  round(mapa2[index][2],1):
                        in_free_space = False
                        break
                    else:
                        in_free_space = True
                        break
            if not in_free_space:
                break
        return in_free_space


    def random_point(self):

        if self.end[0]>self.start[0]:
            x = random.uniform(self.start[0]-0.1, self.end[0]+0.1)
        else:
            x = random.uniform(self.end[0] - 0.1, self.start[0] + 0.1)
        if self.end[1]>self.start[1]:
            y = random.uniform(self.start[1]-0.1, self.end[1]+0.1)
        else:
            y = random.uniform(self.end[1] - 0.1, self.start[1] + 0.1)
        if self.end[2]>self.start[2]:
            z = random.uniform(self.start[2]-0.1, self.end[2]+0.1)
        else:
            z = random.uniform(self.end[2] - 0.1, self.start[2] + 0.1)
        return (x, y, z)

    def find_closest(self, pos, tablica):
        distance=9999
        for point in  tablica:
            temp=math.sqrt((pos[0]-point[0])*(pos[0]-point[0])+(pos[1]-point[1])*(pos[1]-point[1])+(pos[2]-point[2])*(pos[2]-point[2]))
            if temp<distance:
                distance=temp
                closest=point
        return closest

    def new_pt(self, pt, closest):

        alfa=math.atan2(pt[1]-closest[1],pt[0]-closest[0])
        beta = math.atan2(pt[2] - closest[2], math.sqrt((pt[0] - closest[0])*(pt[0] - closest[0])+(pt[1] - closest[1])*(pt[1] - closest[1])))
        pt=(round(closest[0]+0.15*math.cos(alfa),3), round(closest[1]+0.15*math.sin(alfa),3), round(closest[2]+0.15*math.sin(beta),3))
        return pt


    def search(self):

        global mapa
        global mapa2
        mapa=[]
        mapa2=[]
        tBool=False
        if self.i>len(self.end2)-1:
            self.i=0
        self.end=self.end2[self.i]
        if self.i>0:
            self.start=self.end2[self.i-1]
        else:
            self.start=self.end2[len(self.end2)-1]
        self.parent[self.start] = None
        k=0
        qrandprvs=[]
        qrandprvs.append(self.start)
        rp.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback)
        rp.sleep(0.2)
        mapa2=list(mapa)
        try:
            with open("data.pkl", "rb") as a_file:
                pathtemp = pickle.load(a_file)
        except:
            pathtemp = []
        if pathtemp==[]:
            tBool=True
        if tBool:
            while True:
                #rp.Subscriber("/octomap_point_cloud_centers", PointCloud2, callback)
                if len(mapa2)>0:
                    print("haloooo")
                    qrand=self.random_point()
                    np.random.seed(444)
                    temp=self.find_closest(qrand,qrandprvs)
                    qnear=temp
                    qnew=self.new_pt(qrand, qnear)
                    if self.check_if_valid(qnear, qnew):
                        self.parent[qnew]=qnear
                        qrandprvs.append(qnew)
                        self.publish_search()
                        if self.check_if_valid(qnew, self.end):
                            break
                    #print(self.parent)
                    k=k+1
                    #print(mapa)

            #rp.sleep(20.108)
            print("Done")

            self.parent[self.end] = qnew
            path = self.make_path(self.parent, self.end)
            self.publish_path(path)


            a_file = open("data.pkl", "wb")
            pickle.dump(path, a_file)
            a_file.close()
            self.i+=1

    def make_path(self, came_from, current):
        path = [current]
        print(came_from)
        print(current)
        while current != self.start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

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
    rrt = RRT()
    while True:
        rrt.search()

