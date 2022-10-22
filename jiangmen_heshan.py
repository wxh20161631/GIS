# -*- coding: utf-8 -*-
"""
Created on Wed Aug 31 17:30:54 2022

@author: 10212
"""

from time import time
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
#import shapefile
from geopandas import read_file,GeoSeries
import warnings  # 导入警告库
warnings.filterwarnings('ignore')  # 不显示警告
import pandas as pd  # 读取pandas
#import geopandas as gpd  # 读取geopandas
from math import radians, cos, sin, asin, sqrt
import numpy as np
import geopandas as gpd

from time import time

from rtree.index import Index
from pyproj import Geod

from matplotlib_scalebar.scalebar import ScaleBar
from matplotlib.patches import Patch
from matplotlib.lines import Line2D



import geopandas


from mpl_toolkits.basemap import Basemap


import fiona
from pykrige import OrdinaryKriging
from shapely.geometry import Polygon, Point



'''
               1.     open file and input data
'''
start_time = time()	# NO CODE ABOVE HER

greenspace = read_file('./jiangmen/heshan_greenspace.shp')

resident= read_file(r'C:\Users\10212\Desktop\final_data\jm\heshan_pop_final3.shp')

resident= resident.to_crs(epsg=4326)  

greenspace = greenspace.to_crs(epsg=4326)





gs1=greenspace.copy() #copy the greenspace table

gs1['geometry'][0]

gs1['geometry'].plot()

resident.plot(markersize=0.1)# plot the residential area




d0= 5760



jiangmen = ox.graph_from_place('广东省鹤山市', network_type='bike')  # 第一步，获取道路数据
ox.plot_graph(jiangmen)


def get_route(lon1, lat1, lon2, lat2, G):
    origin_point = (lat1, lon1)  # 
    destination_point = (lat2, lon2)  # 
    origin_node = ox.nearest_nodes(G, origin_point[1], origin_point[0])  # get cloest node to origin
    destination_node = ox.nearest_nodes(G, destination_point[1], destination_point[0])  # get cloest node to destination
    route = nx.shortest_path(G, origin_node, destination_node, weight='length')  # get shorest route
    distance = nx.shortest_path_length(G, origin_node, destination_node, weight='length')  # get 
    #fig, ax = ox.plot_graph_route(G, route)  # visualize the result
    print(distance)  
    return distance






#get the Gassian function
def get_G(x,y):
    if x <= y:  
        G = (np.exp(-0.5 * (x/y)**2) - np.exp(-0.5)) / (1 - np.exp(-0.5))
        return G
    else:
        return 0


gs2=gs1

zhongzhuanjuli=[]
for i in range(len(gs1)):
    fenmu_i=0
   
    
    for j in range(len(resident)):

        try:
            #print(gs1['lng'][i],gs1['lat'][i],resident['x'][j],resident['y'][j])
            dis1= int(get_route(gs1['lng'][i],gs1['lat'][i],resident['经度'][j],resident['纬度'][j],jiangmen))
        except:
            dis1 =float('inf')
            #print('error')
        else:
            dis1= int(get_route(gs1['lng'][i],gs1['lat'][i],resident['经度'][j],resident['纬度'][j],jiangmen))
            # print('right')
            
        if dis1<= d0:
            fenmu_i += float(get_G(dis1,d0)*resident['pop3'][j])
            # print('G:',get_G(dis1,d0))
        else:
            fenmu_i+=0
        

    try:

        Rj= float(gs1['area'][i]/fenmu_i)

        gs2['Rj'][i]= Rj
    except Exception as Rj:
        print('unknown error %s' % Rj)








#calculate the accessibility--Ai
res2=resident
# m=0  # fix bug
A_list=[]
for n in range(len(resident)):
    Ai=0


    
    for m in range(len(gs2)):

        try:
            dis2 = int(get_route(resident['经度'][n],resident['纬度'][n],gs2['lng'][m],gs2['lat'][m],jiangmen))
        except:
            dis2 = float('inf')
            print('error')
        else:
            dis2 = int(get_route(resident['经度'][n], resident['纬度'][n], gs2['lng'][m], gs2['lat'][m], jiangmen))


        if dis2<= d0:
            Ai +=float( get_G(dis2,d0)*gs2['Rj'][m])
        else:
            Ai+=0
        
    
    res2['Ai'][n]= Ai




res2.to_file("./output/greenspace_bike_5760.shp",encoding='utf-8')  # output data to local




# report runtime
print(f"completed in: {time() - start_time} seconds")	# NO CODE BELOW HERE
