#!/usr/bin/python
# -*- coding:utf8 -*-

from ITS import py_Node
from ITS import py_Road
from ITS import py_ITS
import numpy

# read data file
def read_city_data(filename):
    f = open(filename)
    line = f.readline()
    text = line.strip()
    text = text.split(' ')
    name = text[4]
    
    line = f.readline()
    line = f.readline()
    
    line = f.readline()
    text = line.strip()
    text = text.split(' ')
    directed = True
    if text[0] == 'directed':
        if text[1] == '1':
            directed = False
    
    line = f.readline()
    text = line.strip()
    text = text.split(' ')
    num_nodes = 0
    if text[0] == 'num_nodes':
        num_nodes = int(text[1])
        city = [None] * num_nodes

    line = f.readline()
    text = line.strip()
    text = text.split(' ')
    num_roads = 0
    if text[0] == 'num_roads':
        num_roads = int(text[1])
        if directed == True:
            road = [None] * num_roads * 2
        else:
            road = [None] * num_roads
            
    line = f.readline()
    text = line.strip()
    text = text.split(' ')
    light_time = 0
    if text[0] == 'all_light_time':
        light_time = int(text[1])

    line = f.readline()
    text = line.strip()
    text = text.split(' ')
    road_lanes = 0
    if text[0] == 'all_road_lanes':
        road_lanes = int(text[1])
    
    for i in range(num_nodes):
        line = f.readline()
        
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        m_id = 0
        if text[0] == 'id':
            m_id = int(text[1])
            
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        m_x = 0
        if text[0] == 'x':
            m_x = int(text[1])
            
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        m_y = 0
        if text[0] == 'y':
            m_y = int(text[1])
        
        city[i] = py_Node(m_id, m_x, m_y)
        line = f.readline()
    
    for i in range(num_roads):
        line = f.readline()
        
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        if text[0] == 'source':
            m_src = int(text[1])
        
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        if text[0] == 'target':
            m_tgt = int(text[1])
 
        if light_time == 0:
            line = f.readline()
            text = line.strip()
            text = text.split(' ')
            m_lig = int(text[1])
        else:
            m_lig = light_time
 
        if road_lanes == 0: 
            line = f.readline()
            text = line.strip()
            text = text.split(' ')
            m_l1 = int(text[1])
            m_l2 = int(text[2])
        else:
            if directed == True:
                m_l1 = m_l2 = road_lanes
            else:
                m_l1 = road_lanes
                m_l2 = 0
            
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        if text[0] == 'ratio':
            m_gt = m_lig * float(text[1])
        
        if directed == False:
            road[i] = py_Road(m_src, m_tgt, m_l1, m_l2, m_lig, m_gt)
            road[i].__cal_length_slope__(city[m_src].axis_real_x, city[m_tgt].axis_real_x, city[m_src].axis_real_y, city[m_tgt].axis_real_y)
            city[m_src].__add_node__(m_tgt, city[m_tgt].axis_real_x, city[m_tgt].axis_real_y)
        elif directed == True:
            road[2*i] = py_Road(m_src, m_tgt, m_l1, m_l2, m_lig, m_gt)
            road[2*i].__cal_length_slope__(city[m_src].axis_real_x, city[m_tgt].axis_real_x, city[m_src].axis_real_y, city[m_tgt].axis_real_y)
            city[m_src].__add_node__(m_tgt, city[m_tgt].axis_real_x, city[m_tgt].axis_real_y)            
            road[2*i+1] = py_Road(m_tgt, m_src, m_l2, m_l1, m_lig, m_gt)
            road[2*i+1].__cal_length_slope__(city[m_tgt].axis_real_x, city[m_src].axis_real_x, city[m_tgt].axis_real_y, city[m_src].axis_real_y)
            city[m_tgt].__add_node__(m_src, city[m_src].axis_real_x, city[m_src].axis_real_y)

        line = f.readline()

    f.close()
    return [name, city, road, directed]

def read_vehicle_data(filename):
    number_od = 0
    f = open(filename)
    line = f.readline()
    size_type = 0
    
    while (line):
        line = f.readline()
        text = line.strip()
        text = text.split(' ')
        
        if text[0] == 'total':
            number_od = int(text[1])
            origin = [None] * number_od
            destin = [None] * number_od
            size   = [None] * number_od
            i = -1
        elif text[0] == 'size':
            if text[1] == 'random':
                size_type = 0
            else:
                size_type = 1
        elif text[0] == 'origin':
            i = i+1
            origin[i] = int(text[1])
        elif text[0] == 'destination':
            del text[0]
            for m in range(len(text)):
                text[m] = int(text[m])
            destin[i] = text
        elif text[0] == 'vehicle_size':
            del text[0]
            for m in range(len(text)):
                text[m] = int(text[m])
            size[i] = text
    
    if size_type == 0:
        size = None
    
    f.close()
    return [origin, destin, number_od, size_type, size]

# calculate the parameters of the nodes on the canvas
def calc_UI_params(city):
    num_of_city = len(city)

    x_upp = x_low = city[0].axis_real_x
    y_upp = y_low = city[0].axis_real_y
    
    for n in range(num_of_city):
        if x_upp < city[n].axis_real_x:
            x_upp = city[n].axis_real_x
        if x_low > city[n].axis_real_x:
            x_low = city[n].axis_real_x
        if y_upp < city[n].axis_real_y:
            y_upp = city[n].axis_real_y
        if y_low > city[n].axis_real_y:
            y_low = city[n].axis_real_y
    
    x_stride = x_upp - x_low
    y_stride = y_upp - y_low
    stride = max(x_stride, y_stride)
    x_ratio = x_stride / stride
    y_ratio = y_stride / stride
    
    for n in range(num_of_city):
        city[n].axis_canvas_x = (city[n].axis_real_x - x_low) / stride
        city[n].axis_canvas_y = (city[n].axis_real_y - y_low) / stride
    
    return [x_low, y_low, x_ratio, y_ratio, stride]
            
if __name__ == '__main__':
    [name, city, road, directed] = read_city_data("data.igml")
    [origin_demand, destin_demand, number_demand, size_type, vehicle_size] = read_vehicle_data("data.ivml")
    scale = calc_UI_params(city)