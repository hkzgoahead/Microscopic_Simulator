#!/usr/bin/python
# -*- coding:utf8 -*-

from test_read import read_city_data
from test_read import read_vehicle_data
from test_read import calc_UI_params
from test_init_vehicle import initial_vehicle
from ITS import py_Node
from ITS import py_Road
from ITS import py_Vehicle
from ITS import py_OD
from ITS import py_ITS
from Tkinter import *
from tkFont import Font
from time import sleep
from pymouse import PyMouse
from time import sleep
from matplotlib import pyplot
from matplotlib import pylab
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import animation
import sys
import numpy
import tkFileDialog
import threading
import tkMessageBox
import math


## global params
top = None
MY_BG_COLOR = '#87CEFF'
MY_CK_COLOR = '#912CEE'
MY_COLOR = ['#CD6889', '#912CEE', '#FF2593']
MY_CANVAS_SIZE = [600, 600]
VEHICLE_FILE_PATH = None
NETWORK_FILE_PATH = None
SUC = 0
name = None
city = None
road = None
road_queue = None
directed = 0
origin_demand = None
destin_demand = None
number_demand = None
size_type = None
vehicle_size = None
vehicle_lane = 6
vehicle = None
c1 = None
c = None
vehicle = None
num_of_city = 0
num_of_road = 0
simulator_speed = 1
object_size = None
s1 = None
scale = None
cmd_begin = 0
cmd_pause = 0
pause_button = None
en_1 = None
en_2 = None
en_3 = None
enID = None
enX  = None
enY  = None
buAxis = None
loc = None
loc_line = None
new_canvas = None
new_numOfNodes = 0
new_ODmax = 0
new_ODmin = 0
OD_generator = None
tag = -1
tag_idx = [None] * 2
event_show_add_line = 1
control_system_act = 0
analyse_system_act = 0
analyse_system_end = 0
flow = None
predict = None
ax = None
fig = None
line = None
draw_time = 1
length_matrix = None
#sim_controller = None
thread = None
thread_aly = None

## main window
def main_window():
    global MY_BG_COLOR
    top = Tk()
    top.geometry('1300x700')
    top.attributes("-alpha", 0.8)
    top['bg'] = MY_BG_COLOR

    return top

## main canvas : show the traffic network(700*1050[600*600])
def prepare_canvas(top, city, road, directed):
    global MY_CANVAS_SIZE
    global object_size
    
    h = MY_CANVAS_SIZE[0]
    w = MY_CANVAS_SIZE[1]
    c = Canvas(top, bg="white", height=h+100, width=w+100)
    road_color = ['#B0B0B0', '#87CEFA']
    
# draw
    num_of_city = len(city)
    num_of_road = len(road)
    object_size = numpy.zeros(5)
    object_size[0] = 10 * int(10/num_of_city)    # object_size[0]: node_size(half)
    object_size[1] = 1.2 * object_size[0]        # object_size[1]: road_size
    object_size[2] = int(object_size[0]/6/2)     # object_size[2]: vehicle_size
    object_size[3] = int(object_size[0]/4/2) 
    object_size[4] = int(object_size[0]/2/2)
    for n in range(num_of_city):
        for i in range(4):
            m_link = city[n].link[i]
            if m_link != -1 and m_link > n:
                param = [city[n].axis_canvas_x*w          + 50, 
                         (1-city[n].axis_canvas_y)*h      + 50,
                         city[m_link].axis_canvas_x*w     + 50, 
                         (1-city[m_link].axis_canvas_y)*h + 50]
                c.create_line(param[0], param[1], param[2], param[3], width=object_size[1], fill=road_color[0], activefill=road_color[1])
                if directed == True:
                    c.create_line(param[0], param[1], param[2], param[3], dash=(4,4))
    
    for n in range(num_of_city):
        x1 = city[n].axis_canvas_x      *w + (50-object_size[0])
        y1 = (1-city[n].axis_canvas_y)  *h + (50-object_size[0])
        x2 = city[n].axis_canvas_x      *w + (50+object_size[0])
        y2 = (1-city[n].axis_canvas_y)  *h + (50+object_size[0])
        
        c.create_oval(x1, y1, x2, y2, fill='green', outline='green', activefill='yellow')

    return [c, object_size]

## the following three function draws the CLOCK and SCALE
# draw time clock
def prepare_button(top):
    global MY_CK_COLOR
    c1 = Canvas(top, bd=0, bg=MY_CK_COLOR, height=60, width=150)
    c1.create_rectangle(0, 0, 150, 60, fill=MY_CK_COLOR, outline=MY_CK_COLOR)  
    return c1

# draw simulator speed scale
def prepare_scale(top, sp):
    simulator_speed_scale = Scale(top, from_=1, to=20, orient=HORIZONTAL, label='simulator speed', variable=sp, length=150, bd=0, bg=MY_CK_COLOR, troughcolor=MY_CK_COLOR)
    return simulator_speed_scale

# time
def prepare_time_text(milise, second, minute):
    text = str('')
    if minute < 10:
        text = text + str(0) + str(minute) + ':'
    else:
        text = text + str(minute) + ':'
    if second < 10:
        text = text + str(0) + str(second) + '.'
    else:
        text = text + str(second) + '.'
    if milise < 10:
        text = text + str(0) + str(milise)
    else:
        text = text + str(milise)

    return text

# check if all vehicles end their travel
def is_all_arrived(vehicle):
    all_arrived = 1
    
    for i in range(len(vehicle)):
        v = vehicle[i]
        if v == None:
            all_arrived = 0
        elif v.arrived == 0:
            all_arrived = 0
        
        if all_arrived == 0:
            break

    return all_arrived

## the followint three function is about DRAWING VEHICLES
# draw vehicles
def prepare_vehicle_block(c, scale, pos, size, ob_size, slope, tag):
    global MY_COLOR
    global MY_CANVAS_SIZE
    global directed
    global vehicle_lane
    
    h = MY_CANVAS_SIZE[0]
    w = MY_CANVAS_SIZE[1]

    if directed == False:
        tmpx = tmpy = 0
    else:
        if slope != 0 and slope != 90 and slope != 270 and slope != 360:
            slope = float(slope)/360.0 * numpy.pi
            tmpx = vehicle_lane * numpy.sin(slope)
            tmpy = -vehicle_lane * numpy.cos(slope)
        elif slope == 0:
            tmpx = 0
            tmpy = vehicle_lane
        elif slope == 90:
            tmpx = vehicle_lane
            tmpy = 0
        elif slope == 180:
            tmpx = 0
            tmpy = -vehicle_lane
        elif slope == 270:
            tmpx = -vehicle_lane
            tmpy = 0
    
    pos[0] = (pos[0]-scale[0])/scale[4] * w  + 50 + tmpx
    pos[1] = (1-(pos[1]-scale[1])/scale[4]) * h + 50 - tmpy

    rt = c.create_oval(pos[0]-ob_size, pos[1]-ob_size, pos[0]+ob_size, pos[1]+ob_size, fill=MY_COLOR[size], outline=MY_COLOR[size], tags=tag)
    
    return rt

# move vehicles
def vehicle_move(c, rt, scale, pos, size, ob_size, slope):
    global MY_CANVAS_SIZE
    global directed
    global vehicle_lane
    
    h = MY_CANVAS_SIZE[0]
    w = MY_CANVAS_SIZE[1]
    
    if directed == False:
        tmpx = tmpy = 0
    else:
        if slope != 0 and slope != 90 and slope != 180 and slope != 270:
            slope = float(slope)/180.0 * numpy.pi
            tmpx = vehicle_lane * numpy.sin(slope)
            tmpy = -vehicle_lane * numpy.cos(slope)
        elif slope == 0:
            tmpx = 0
            tmpy = -vehicle_lane
        elif slope == 90:
            tmpx = vehicle_lane
            tmpy = 0
        elif slope == 180:
            tmpx = 0
            tmpy = vehicle_lane
        elif slope == 270:
            tmpx = -vehicle_lane
            tmpy = 0
            
    pos[0] = (pos[0]-scale[0])/scale[4] * w  + 50 + tmpx
    pos[1] = (1-(pos[1]-scale[1])/scale[4]) * h + 50 - tmpy
    
    if ~math.isnan(numpy.round(pos[0])) and ~math.isnan(numpy.round(pos[1])) and ~math.isnan(ob_size):
        c.coords(rt, pos[0]-ob_size, pos[1]-ob_size, pos[0]+ob_size, pos[1]+ob_size)
    else:
        print pos
        
def delete_arrived_vehicle(c, tagstr):
    c.delete(tagstr)

## the following two 'main_' functions are the kernel functions of the code    
def main_prepare():
    global name
    global city
    global road
    global directed
    global origin_demand
    global destin_demand
    global number_demand
    global size_type
    global vehicle_size
    global c1
    global c
    global vehicle
    global num_of_city
    global num_of_road
    global length_matrix
    
    global simulator_speed
    global s1
    global scale
    global object_size
    global MY_CK_COLOR
    global MY_BG_COLOR
    global pause_button

    [name, city, road, directed] = read_city_data(NETWORK_FILE_PATH)
    [origin_demand, destin_demand, number_demand, size_type, vehicle_size] = read_vehicle_data(VEHICLE_FILE_PATH)
    print origin_demand
    print destin_demand
    scale = calc_UI_params(city)
    top.title(name)
    vehicle = initial_vehicle(origin_demand, destin_demand, number_demand, size_type, vehicle_size, city, road, directed)
    num_of_city = len(city)
    num_of_road = len(road)

    length_matrix = 10000.0*numpy.ones(num_of_city*num_of_city)
    for i in range(num_of_city):
        for j in range(num_of_city):
            if j == i:
                length_matrix[num_of_city*i+j] = 0
            else:
                for l in range(4):
                    if city[i].link[l] != -1 and j == city[i].link[l]:
                        length_matrix[num_of_city*i+j] = city[i].dist[l]

    pause_button = Button(top, text="pause", bg=MY_BG_COLOR, fg=MY_CK_COLOR, command=cmd_simulate_pause)
    pause_button.grid(row=0, column=0)

    c1 = prepare_button(top)
    c1.grid(row=1, column=0)

    simulator_speed = StringVar()
    s1 = prepare_scale(top, simulator_speed)
    s1.grid(row=2, column=0)
    
    # canvas
    [c, object_size] = prepare_canvas(top, city, road, directed)
    c.grid(row=0, column=1, rowspan=30, columnspan=30) 
    
def main_simulate():
    main_begin = 1
    global name
    global city
    global road
    global road_queue
    global directed
    global origin_demand
    global destin_demand
    global number_demand
    global size_type
    global vehicle_size
    global c1
    global c
    global vehicle
    global num_of_city
    global num_of_road    
    global simulator_speed
    global s1
    global scale
    global object_size
    global cmd_begin
    global cmd_pause
    global pause_button
    global ITS_center
    global control_system_act
    global analyse_system_act
    global analyse_system_end 
    global flow
    global predict
    global draw_time
    global length_matrix
    global thread
    global thread_aly

    # parameters for ITS center
    flow = [None] * num_of_road
    predict = [None] * num_of_road
    ITS_center = [None] * num_of_road
    road_queue = [None] * num_of_road

    # random possion stream
    vehicle_rectangle = None
    vehicle_onRoad    = 0
    vehicle_arrived   = 0            
    # time interval
    come_label        = [None] * number_demand
    poisson_lamda     = numpy.zeros(number_demand)
    time_interval     = [None] * number_demand
    time_accumulation = [None] * number_demand
    for i in range(number_demand):
        come_label[i]        = numpy.zeros(len(destin_demand[i]))
        poisson_lamda[i]     = 2.5
        time_interval[i]     = numpy.random.exponential(poisson_lamda[i], len(destin_demand[i]))
        time_accumulation[i] = numpy.zeros(len(destin_demand[i]))
        for j in range(len(destin_demand[i])):
            if j == 0:
                time_accumulation[i][j] = time_interval[i][j]
            else:
                time_accumulation[i][j] = time_accumulation[i][j-1] + time_interval[i][j]
            
    # simulator time unit
    simulator_time   = 0
    simulator_minute = 0
    simulator_second = 0
    simulator_milise = 0
    simulator_unit   = 0.01
    basic_unit       = 0.01
    m_font = Font(family='Century Schoolbook L',size=24, weight='bold')
    m_text = prepare_time_text(simulator_milise, simulator_second, simulator_minute)
    time_text = c1.create_text(75, 30, text=m_text, font=m_font, activefill='yellow', fill='white')
    
    index = None
  
    while ((is_all_arrived(vehicle) != 1 or main_begin == 1)):
        while cmd_pause == 1:
            sleep(1)
            
        main_begin = 0
        sleep(basic_unit)
        simulator_unit = basic_unit * int(simulator_speed.get())
        simulator_time   = simulator_time   + simulator_unit
        simulator_milise = simulator_milise + int(simulator_speed.get())
        if simulator_milise >= 100:
            simulator_milise = 0
            simulator_second = simulator_second + 1
            if simulator_second == 60:
                simulator_second = 0
                simulator_minute = simulator_minute + 1
                control_system_act = 1
        
        # check for new-coming vehicles
        for i in range(number_demand):
            for j in range(len(destin_demand[i])):
                #　新上路的车：时间上此次应该上路，且还没标记上路
                # New vehicles that should be on road but not symbolized yet
                if time_accumulation[i][j] <= simulator_time and come_label[i][j] == 0:
                    temp_id = 0
                    for t_i in range(i):
                        temp_id += int(len(destin_demand[t_i]))
                    temp_id += int(j)
                    # temp_id 是按照需求列表排列下来的一个标记
                    # temp_id is the subscript for the demand list
                    if index == None:
                        index = [temp_id]
                    else:
                        index.append(temp_id)
                    
                    # 设置ID
                    # Set ID
                    vehicle[temp_id].ID = temp_id
                    print temp_id
                    # 设置上路时间
                    # Set the departure time for the vehicle
                    vehicle[temp_id].departure_time = simulator_time
                    
                    #print length_matrix
                    #　出发前最短路算法
                    # Generate a shortest path for the vehicle before departure
                    vehicle[temp_id].__shortest_path__(length_matrix, num_of_city)
                    #　如果旅行路段为１，则直接找到新上路的车的道路
                    #　否则找到该车的第一个路段，设置其临时终点
                    # If the route length is 1,
                    # Otherwise, find the fitst link of its route
                    if vehicle[temp_id].plan_route_length == 1:
                        for m in range(num_of_road):
                            if (road[m].id1 == vehicle[temp_id].ori_id
                                and road[m].id2 == vehicle[temp_id].des_id):
                                m_road  = m
                                break
                    else:
                        vehicle[temp_id].temp_destin = [city[vehicle[temp_id].temp_destin_id].axis_real_x,
                                                        city[vehicle[temp_id].temp_destin_id].axis_real_y];
                        for m in range(num_of_road):
                            if (road[m].id1 == vehicle[temp_id].ori_id 
                                and road[m].id2 == vehicle[temp_id].temp_destin_id):
                                m_road = m
                                break
                    # 设置该车的道路和道路的斜率
                    # set the road(link) and slope of road for the vehicle
                    vehicle[temp_id].road  = m_road
                    vehicle[temp_id].slope = road[m_road].slope

                    # 更新相关的道路的参数：流量＋１；车辆加入队列中并获取前车位置
                    # the flow of the road(link) is added by 1
                    road[m_road].flow = road[m_road].flow + 1
                    
                    if road_queue[m_road] == None or len(road_queue[m_road]) == 0:
                        road_queue[m_road] = [temp_id]
                    else:
                        vehicle[temp_id].header_pos = vehicle[road_queue[m_road][len(road_queue[m_road])-1]].position
                        road_queue[m_road].append(temp_id)                    
                    
                    # 为图形打标记
                    # symbolize
                    M = vehicle[temp_id].ori_id
                    N = vehicle[temp_id].des_id
                    vr = prepare_vehicle_block(c, scale, 
                                               vehicle[temp_id].position, 
                                               vehicle[temp_id].size, 
                                               object_size[vehicle[temp_id].size+2], 
                                               road[m_road].slope, 
                                               "v"+str(M)+str(N)+str(temp_id))
                         
                    if vehicle_rectangle == None:
                        vehicle_rectangle = [vr]
                    else:
                        vehicle_rectangle.append(vr)
                        
                    come_label[i][j] = 1
                    vehicle_onRoad = vehicle_onRoad+1
                        
                elif time_accumulation[i][j] > simulator_time:
                    break
                
        # update all the vehicles' position
        if vehicle_rectangle != None:
            for i in range(vehicle_onRoad):
                m_ha = 0
                #　对已经出发但是没有到达的车辆
                # for vehicle that has already departed but has not arrived yet
                if vehicle[index[i]].departure_time > 0 and vehicle[index[i]].arrived == 0:
                    hp = [-1, -1]
                    hv = -1
                    loc = -1
                    #　寻找前车的速度和位置
                    # find the leading vehicle's speed and position
                    for j in range(len(road_queue[vehicle[index[i]].road])):
                        if road_queue[vehicle[index[i]].road][j] == index[i]:
                            loc = j-1
                            break;
                        
                    if (loc > 1 
                        and vehicle[road_queue[vehicle[index[i]].road][loc]].arrived == 0 
                        and vehicle[road_queue[vehicle[index[i]].road][loc]].temp_origin == vehicle[index[i]].temp_origin 
                        and vehicle[road_queue[vehicle[index[i]].road][loc]].temp_destin == vehicle[index[i]].temp_destin):
                        hp = vehicle[road_queue[vehicle[index[i]].road][loc]].position
                        hv = vehicle[road_queue[vehicle[index[i]].road][loc]].velocity

                    #　更新加速度、速度和位置
                    # update acceleration, speed and position
                    # hp, hv, m_ha is for the leading vehicle
                    m_ha = vehicle[index[i]].__generate_acc_velocity__(0,hp,hv,m_ha)
                    vehicle[index[i]].__calc_vel__(simulator_unit)
                    vehicle[index[i]].__calc_pos__(simulator_unit)
                    
                    #　ＵＩ上移动车辆
                    # Moving vehicle on the User Interface
                    if ~math.isnan(vehicle[index[i]].position[0]) and ~math.isnan(vehicle[index[i]].position[1]):
                        vehicle_move(c, vehicle_rectangle[i], scale, 
                                     vehicle[index[i]].position, vehicle[index[i]].size, 
                                     object_size[vehicle[index[i]].size+2], vehicle[index[i]].slope)
                    else:
                        vehicle[index[i]].arrived = 1
                        vehicle[index[i]].acc_velocity = 0.0
                        vehicle[index[i]].speed = 0.0
                        vehicle[index[i]].position = vehicle[index[i]].destination
                        
                        delete_arrived_vehicle(c, vehicle_rectangle[waste_list[i]])
                        road[vehicle[index[i]].road].flow = road[vehicle[index[i]].road].flow - 1
                        del vehicle_rectangle[index[i]]
                        del road_queue[vehicle[index[i]]][0]

                        if len(road_queue[vehicle[index[i]].road]) > 0:
                            vehicle[road_queue[vehicle[index[i]].road][0]].header_pos  = [0.0, 0.0]
                            vehicle[road_queue[vehicle[index[i]].road][0]].header_velo = 0.0
                    
                    #　如果该车的旅途经过多个路段
                    if vehicle[index[i]].plan_route_length > 1:
                        #　判断该车是否到达某个路口节点
                        vehicle[index[i]].__is_temp_arrived__() # 此函数中应该实现计划路段数目自减，更新车辆的位置、临时出发点和临时终点
                        #　如果是的话涉及如下操作：从旧的队列中删除该车，添加到新的队列中；相应的，道路流量－１、＋１；
                        #　　　　　　　　　　　　　获取新的道路的编号和斜率，设置到车辆的数据域中
                        if vehicle[index[i]].temp_arrived == 1:
                            if len(road_queue[vehicle[index[i]].road]) > loc+2:
                                vehicle[road_queue[vehicle[index[i]].road][loc+2]].header_pos  = [-1, -1]
                                vehicle[road_queue[vehicle[index[i]].road][loc+2]].header_velo = -1
                                vehicle[road_queue[vehicle[index[i]].road][loc+2]].__generate_acc_velocity__(0,[-1,-1],-1,0)
                                vehicle[road_queue[vehicle[index[i]].road][loc+2]].__calc_vel__(simulator_unit)
                                vehicle[road_queue[vehicle[index[i]].road][loc+2]].__calc_pos__(simulator_unit)
                                
                            del road_queue[vehicle[index[i]].road][loc+1]
                            road[vehicle[index[i]].road].flow = road[vehicle[index[i]].road].flow-1
                            vehicle[index[i]].temp_destin = [city[vehicle[index[i]].temp_destin_id].axis_real_x,
                                                             city[vehicle[index[i]].temp_destin_id].axis_real_y]
                            for m in range(num_of_road):
                                if road[m].id1 == vehicle[index[i]].temp_origin_id and road[m].id2 == vehicle[index[i]].temp_destin_id:
                                    road[m].flow = road[m].flow+1
                                    vehicle[index[i]].slope = road[m].slope
                                    vehicle[index[i]].road = m
                                    if road_queue[m] == None:
                                        road_queue[m] = [vehicle[index[i]].ID]
                                    else:
                                        road_queue[m].append(vehicle[index[i]].ID)
                        
        # update the arrived vehicles
        waste_list = None
        for i in range(vehicle_onRoad):
            if vehicle[index[i]] != None:
                if vehicle[index[i]].departure_time > 0 and vehicle[index[i]].arrived == 0:
                    vehicle[index[i]].__is_arrived__()
                    if vehicle[index[i]].arrived == 1:
                        if waste_list == None:
                            waste_list = [i]
                        else:
                            waste_list.append(i)

        if waste_list != None:
            vehicle_arrived = vehicle_arrived + len(waste_list)
            vehicle_onRoad = vehicle_onRoad - len(waste_list)              
            for i in range(len(waste_list)):
                M = vehicle[index[waste_list[i]]].ori_id
                N = vehicle[index[waste_list[i]]].des_id
                delete_arrived_vehicle(c, "v"+str(M)+str(N)+str(index[waste_list[i]]))
#                delete_arrived_vehicle(c, vehicle_rectangle[waste_list[i]])
                road[vehicle[index[waste_list[i]]].road].flow = road[vehicle[index[waste_list[i]]].road].flow - 1
                vehicle[index[waste_list[i]]].velocity = 0.0
                vehicle[index[waste_list[i]]].acc_velocity = 0.0
                
                del vehicle_rectangle[waste_list[i]]
                
                del road_queue[vehicle[index[waste_list[i]]].road][0]
                if len(road_queue[vehicle[index[waste_list[i]]].road]) > 0:
                    vehicle[road_queue[vehicle[index[waste_list[i]]].road][0]].header_pos  = [-1, -1]
                    vehicle[road_queue[vehicle[index[waste_list[i]]].road][0]].header_velo = -1     
                    vehicle[index[i]].__generate_acc_velocity__(0,[-1,-1],-1,0)
                    vehicle[index[i]].__calc_vel__(simulator_unit)
                    vehicle[index[i]].__calc_pos__(simulator_unit)
                    
                del index[waste_list[i]]
                
        # control system acts
        if control_system_act == 1:
            control_system_act = 0
            analyse_system_act = 1
            draw_time = simulator_minute
            
            for i in range(num_of_road):
                if flow[i] == None:
                    flow[i] = [road[i].flow]
                else:
                    flow[i].append(road[i].flow)
                
                if simulator_minute >= 6:
                    # 1 : past data(5)
                    # 2 : past deviation(3)
                    # 3 : link(4)
                    # 4 : past day(5)
                    # 5 : past error(3)
                    if predict[i] == None:
                        for j in range(simulator_minute-1):
                            if predict[i] == None:
                                predict[i] = [flow[i][j]]
                            else:
                                predict[i].append(flow[i][j])

                    if ITS_center[i] == None:
                        ITS_center[i] = py_ITS(5,3,4,5,3,flow[i][0],flow[i][1],flow[i][2],flow[i][3],flow[i][4],0,0,0,0,0,0,0,0,0,0.01)
                    else:
                        ITS_center[i].__predict_flow_parameter_tuning__(flow[i][simulator_minute-1])
                        ITS_center[i].__predict_flow_data_updating__(flow[i][simulator_minute-1],0,0,0,0,0,0,0,0,0)
                    
                    ITS_center[i].__predict_flow__()
                    predict[i].append(int(ITS_center[i].preFlow))                    
                        
        # update time
        m_text = prepare_time_text(simulator_milise, simulator_second, simulator_minute)
        c1.itemconfig(time_text, text=m_text, font=m_font, activefill='yellow', fill='white')
        
        c.update_idletasks()
            
    # end while
    print 'simulation ends'
    #main_begin = 1
    analyse_system_end = 1

## the following functions are the CMD which attach to the menu bar
def file_read():
    global NETWORK_FILE_PATH
    global VEHICLE_FILE_PATH
    global SUC
    
    filename = tkFileDialog.askopenfilename()
    print filename
    
    if filename.find('.igml') > 0:
        NETWORK_FILE_PATH = filename
        print 'network loaded'
        # if vehicle loaded, suc=3, else suc=1 for network loaded
        if SUC == 2:
            SUC = 3
        else:
            SUC = 1
    elif filename.find('.ivml') > 0:
        VEHICLE_FILE_PATH = filename
        print 'vehicle loaded'
        # if network loaded, suc = 3, else suc=2 for vehicle loaded
        if SUC == 1:
            SUC = 3
        else:
            SUC = 2
    # Activate main when suc=3
    if SUC == 3:
        main_prepare()

def event_control_axis(event):
    global enX
    global enY
    global buAxis
    global loc
    global tag
    
    if event.num == 1:
        print event.num
        tag = int(new_canvas.gettags(CURRENT)[0])
        item = new_canvas.find_withtag(tag+1)
        e = StringVar()
        enID['textvariable'] = e
        e.set(str(tag))
        
        e1 = StringVar()
        enX['textvariable'] = e1
        e1.set(str(loc[tag][0]))
        e2 = StringVar()
        enY['textvariable'] = e2
        e2.set(str(loc[tag][1]))

def event_control_line(event):
    global loc
    global loc_line
    global tag_idx
    global event_show_add_line
    
    temp = int(new_canvas.gettags(CURRENT)[0])
    add_line = None
    
    # event.num = 3
    if event.num == 1:
        print event.x
        print event.y
        if tag_idx[0] == None:
            tag_idx[0] = temp
        else:
            if event_show_add_line == 1:
                add = tkMessageBox.askquestion(title="confirm", message="single click with two nodes will create a path.")
                show = tkMessageBox.askquestion(title="show_tips", message="Do not show this tip any more.")
                if show == 'yes':
                    event_show_add_line = 0
            else:
                add = 'yes'
            
            if add == 'yes':
                if tag_idx[1] == None and tag_idx[0] != temp:
                    tag_idx[1] = temp
                    add_line = new_canvas.create_line(50+loc[tag_idx[0]][0], 50+loc[tag_idx[0]][1], 50+loc[tag_idx[1]][0], 50+loc[tag_idx[1]][1], dash=(4,4), fill='green')
                    if loc_line == None:
                        loc_line = [tag_idx]
                    else:
                        loc_line.append(tag_idx)
                    
            tag_idx = [None] * 2

def event_change_axis():
    global loc
    global tag
    global enID
    global enX
    global enY    
    global MY_CANVAS_SIZE
    h = MY_CANVAS_SIZE[0]
    w = MY_CANVAS_SIZE[1]
    r = int(h/len(loc)/12)
    
    if tag != -1:
        loc[tag][0] = int(enX.get())
        loc[tag][1] = int(enY.get())

        item = new_canvas.find_withtag(tag+1)
        new_canvas.itemconfig(item[0], fill='grey')
        new_canvas.coords(item[0], (loc[tag][0]-r+50, loc[tag][1]-r+50, loc[tag][0]+r+50, loc[tag][1]+r+50))
    else:
        tag = int(enID.get())
        if tag >= new_numOfNodes:
            tag = -1
            tkMessageBox.showerror(title="Error(cross the bounds!)", message="Please input an ID in (0~NumOfNode-1)")
            e = StringVar()
            enID['textvariable'] = e
            e.set('point ID')
        else:
            loc[tag][0] = int(enX.get())
            loc[tag][1] = int(enY.get())
            for i in range(tag):
                if loc[i][0] == loc[tag][0] and loc[i][1] == loc[tag][1]:
                    tkMessageBox.showwarning(title="Warning(overlap)", message="There exists more than one node on the same axis.")
            item = new_canvas.find_withtag(tag+1)
            new_canvas.itemconfig(item[0], fill='grey')
            new_canvas.coords(item[0], (loc[tag][0]-r+50, loc[tag][1]-r+50, loc[tag][0]+r+50, loc[tag][1]+r+50))            
        
    tag = -1
        
def event_end_edit():
    global loc
    global enID
    global enX
    global enY
    
    new_canvas.unbind("<Double-1>")
    new_canvas.unbind("<Button-1>")
    enID['state'] = 'readonly'
    enID['bg']    = 'grey'
    enX['state']  = 'readonly'
    enX['bg']     = 'grey'
    enY['state']  = 'readonly'
    enY['bg']     = 'grey'    

def upload_data():
    global new_canvas
    global new_numOfNodes
    global new_ODmax
    global new_ODmin
    global OD_generator
    global en_1
    global en_2
    global en_3
    global enID
    global enX
    global enY
    global buAxis
    global loc
    global MY_CANVAS_SIZE
    h = MY_CANVAS_SIZE[0]
    w = MY_CANVAS_SIZE[1]
    
    if en_1 == None or en_2 == None or en_3 == None:
        tkMessageBox.showerror(title="Error!", message="Nothing can be saved.")
    elif en_1.get() == '' or en_2.get() == '' or en_3.get() == '': 
        tkMessageBox.showerror(title="Error!", message="Nothing can be saved.")
    else:
        new_canvas.delete(ALL)
        new_numOfNodes = int(en_1.get())
        new_ODmax      = int(en_2.get())+1
        new_ODmin      = int(en_3.get())
    
        OD_generator = py_OD(new_numOfNodes, new_ODmax, new_ODmin)    
        OD_generator.__generateODseries__()
        
        tkMessageBox.showinfo(title="Success", message="OD Matrix saved!")
        tkMessageBox.showinfo(title="tips", message="double LEFT click on the node to change the axis(use integers like 100).")
        tkMessageBox.showinfo(title="tips", message="single RIGHT click on two nodes to set the path(lane, traffic light, etc).")
        
        loc = numpy.zeros((new_numOfNodes,2),dtype=int)
        loc_rnd = numpy.random.rand(new_numOfNodes, 2)
        loc_rnd[:,0] = loc_rnd[:,0] * h
        loc_rnd[:,1] = loc_rnd[:,1] * w
        r = int(h/new_numOfNodes/12)
        oval = [None] * new_numOfNodes
        for i in range(new_numOfNodes):
            loc[i][0] = int(loc_rnd[i][0])
            loc[i][1] = int(loc_rnd[i][1])
            oval[i] = new_canvas.create_oval(50+loc[i][0]-r, 50+loc[i][1]-r, 50+loc[i][0]+r, 50+loc[i][1]+r, fill='blue', outline='blue', activefill='yellow', tags=(str(i)))
                
        new_canvas.bind("<Double-1>", event_control_axis)
        new_canvas.bind("<Button-1>", event_control_line)

        e1   = StringVar()
        laID = Label(top, text='ID').grid(row=0, column=3)
        enID = Entry(top, textvariable=e1)
        enID.grid(row=0, column=4)
        enID['state'] = 'readonly'
        e1.set('point ID')
        laX  = Label(top, text='axis_x').grid(row=1, column=3)
        e2   = StringVar()
        enX  = Entry(top, textvariable=e2)
        enX.grid(row=1, column=4)
        e2.set('input the value here.')
        laY  = Label(top, text='axis_y').grid(row=2, column=3)
        e3   = StringVar()
        enY  = Entry(top, textvariable=e3)
        enY.grid(row=2, column=4)
        e3.set('input the value here.')
        
        buAxis = Button(top, text='Yes', command=event_change_axis).grid(row=3, column=4)
        convinceButton = Button(top, text='Finish!', command=event_end_edit).grid(row=4, column=4)
        
def canvas_new():
    global en_1
    global en_2
    global en_3
    global MY_CANVAS_SIZE
    global new_canvas
    
    h = MY_CANVAS_SIZE[0]
    w = MY_CANVAS_SIZE[1]
    
    v1 = StringVar()
    # No. of nodes
    la_1 = Label(top, text='Number of Nodes').grid(row=0, column=0)
    en_1 = Entry(top)
    en_1.grid(row=0, column=1)
    # 需求水平最大值 Max demand
    la_2 = Label(top, text='Max demand').grid(row=1, column=0)
    en_2 = Entry(top)
    en_2.grid(row=1, column=1)
    # 需求水平最小值 Min demand
    la_3 = Label(top, text='Min demand').grid(row=2, column=0)
    en_3 = Entry(top)
    en_3.grid(row=2, column=1)
    
    bu   = Button(top, text='Yes', command=upload_data).grid(row=3, column=1)

    new_canvas = Canvas(top, bd=0, bg='white', height=h+100, width=w+100)
    new_canvas.grid(row=0, rowspan=20, column=2)

'''
class simControl(object):
    def __init__(self):
        self.thread0 = None
        self.stop_threads = threading.Event()
        
    def mainSim(self):
        self.stop_threads.clear()
        self.thread0 = threading.Thread(target = main_simulate)
        self.thread0.start()

    def stop(self):
        self.stop_threads.set()
        self.thread0.join()
        self.thread0 = None
'''

def cmd_simulate_start():
    global cmd_begin
    global thread
    #global sim_controller
    if cmd_begin == 0:
        cmd_begin = 1
        thread = threading.Thread(target=main_simulate)
        thread.daemon = True
        thread.start()
        #sim_controller.mainSim()
        
def cmd_simulate_pause():
    global cmd_begin
    global cmd_pause
    global pause_button
    if pause_button != None and cmd_begin == 1:
        if cmd_pause == 0:
            cmd_pause = 1
            pause_button['text'] = "begin";
        elif cmd_pause == 1:
            cmd_pause = 0
            pause_button['text'] = "pause";

def save_network():
    global loc
    global loc_line
    global top
    global NETWORK_FILE_PATH
    NETWORK_FILE_PATH ="./data.igml"

    for i in range(len(loc_line)):
        if loc_line[i][0] > loc_line[i][1]:
            temp = loc_line[i][0]
            loc_line[i][0] = loc_line[i][1]
            loc_line[i][1] = temp
    
    loc_line.sort()
    
    f = open(NETWORK_FILE_PATH, "w")
    f.writelines('Database for city : CIVE-615-Demo\nCreator [ Kz ON April 1, 2019 ]\ngraph [\n  directed 0\n')
    f.write('  num_nodes '+str(len(loc))+'\n')
    f.write('  num_roads '+str(len(loc_line))+'\n')
    f.write('  all_light_time '+str(100)+'\n')
    f.write('  all_road_lanes '+str(1)+'\n')
    
    for i in range(len(loc)):
        f.writelines('  node [\n    id '+str(i)+'\n'+'    x '+str(int(loc[i][0]))+'\n    y '+str(int(loc[i][1]))+'\n  ]\n')

    for i in range(len(loc_line)):
        f.writelines('  edge [\n    source '+str(loc_line[i][0])+'\n    target '+str(loc_line[i][1])+'\n    ratio 0.6\n  ]\n')
    
    f.write(']\n')
    f.close()

    top.destroy()
    top = main_window()
    menubar = prepare_menu(top)
    top.config(menu=menubar)
    top.mainloop()  

def main_analyse():
    global predict
    global flow
    global draw_time
    global analyse_system_act
    global analyse_system_end 
    
    while analyse_system_end == 0:
        while analyse_system_act == 0:
            sleep(1)
    
        x = numpy.zeros(draw_time)
        for i in range(draw_time):
            x[i] = i

        main_analyse.f.clf()
        main_analyse.a=main_analyse.f.add_subplot(411)
        main_analyse.a.set_title('Flow[0]')
    
        #if flow[0] != None and predict[0] != None:
        # only plot flot, not predicted flow
        if flow[0] != None:
            print 'flow0 exists'
            main_analyse.a.plot(x,flow[0][0:draw_time],color='g')
            #main_analyse.a.plot(x,predict[0][0:draw_time],color='b')
            
            #main_analyse.f.clf()
        main_analyse.a=main_analyse.f.add_subplot(412)
        main_analyse.a.set_title('Flow[1]')
        if flow[1] != None:
            print 'flow1 exists'
            main_analyse.a.plot(x,flow[1][0:draw_time],color='g')

        main_analyse.a=main_analyse.f.add_subplot(413)
        main_analyse.a.set_title('Flow[2]')
        if flow[2] != None:
            print 'flow2 exists'
        main_analyse.a.plot(x,flow[2][0:draw_time],color='g')
        analyse_system_act = 0

        main_analyse.a=main_analyse.f.add_subplot(414)
        main_analyse.a.set_title('Flow[3]')
        if flow[3] != None:
            print 'flow3 exists'
        main_analyse.a.plot(x,flow[3][0:draw_time],color='g')

        # show canvas
        analyse_system_act = 0
        main_analyse.canvas.show()

def cmd_analyse():
    global top
    global thread_aly
    
    cmd_begin = 1

    #在Tk的GUI上放置一个画布，并用.grid()来调整布局
    #e1   = StringVar()
    #laRoad = Label(top, text='road name: ').grid(row=0, column=32)
    #enRoad = Entry(top, textvariable=e1)
    #enRoad.grid(row=0, column=33)
    #enRoad['state'] = 'readonly'
    #enRoad['bg']    = 'grey'
    
    main_analyse.f = Figure(figsize=(5,16), dpi=50)
    main_analyse.canvas = FigureCanvasTkAgg(main_analyse.f, master=top)
    main_analyse.canvas.show()
    main_analyse.canvas.get_tk_widget().grid(row=1, column=32, rowspan=15, columnspan=15)
    
    thread_aly = threading.Thread(target=main_analyse)
    thread_aly.daemon = True
    thread_aly.start()    
    #cmd_simulate_start()

# menu bar
def prepare_menu(top):
    main_menu = Menu(top)
# bar1 : file
    filemenu = Menu(main_menu, tearoff=0)
    filemenu.add_command(label='New', command=canvas_new)
    filemenu.add_command(label='Load network', command=file_read)
    filemenu.add_command(label='Load vehicle', command=file_read)
    filemenu.add_command(label='Save', command=save_network)
    filemenu.add_separator()
    filemenu.add_command(label='Exit', command=top.quit)
    #filemenu.add_command(label='Exit', command=sysquit)
    
    main_menu.add_cascade(label='File',menu=filemenu)
# bar2 : simulation menu
    simmenu = Menu(main_menu, tearoff=0)
    simmenu.add_command(label='begin', command=cmd_simulate_start)
    simmenu.add_command(label='pause', command=cmd_simulate_pause)
    
    main_menu.add_cascade(label='simulate',menu=simmenu)

# bar3 : analyse
    anamenu = Menu(main_menu, tearoff=0)
    anamenu.add_command(label='settings')
    anamenu.add_command(label='vehicle')
    anamenu.add_command(label='road', command=cmd_analyse)
    anamenu.add_separator()
    anamenu.add_command(label='User guide')

    main_menu.add_cascade(label='analyse',menu=anamenu)

    return main_menu
'''
def sysquit():
    global top
    top.quit
'''
# main
if __name__ == '__main__':
# main window
    top = main_window()
#    global top
#sim_controller = simControl()
# menu bar
    menubar = prepare_menu(top)
    top.config(menu=menubar)
# tkinter's mainloop
    top.mainloop()
