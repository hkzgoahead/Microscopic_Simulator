from test_read import read_city_data
from test_read import read_vehicle_data
from test_read import calc_UI_params
from ITS import py_Node
from ITS import py_Road
from ITS import py_Vehicle
from ITS import py_ITS
import numpy

# initial vehicle objects
def initial_vehicle(origin_demand, destin_demand, number_demand, size_type, vehicle_type, city, road, directed):
    total_demand = 0
    num_of_city = len(city)
    if directed == True:
        num_of_road = len(road) / 2
    else:
        num_of_road = len(road)
        
    for i in range(number_demand):
        total_demand = total_demand + len(destin_demand[i])
        
    vehicle = [None] * total_demand
    num = 0
    
    if size_type == 0:
        vehicle_type = numpy.floor(numpy.random.uniform(1,3,total_demand))
    
    secure_dist   = numpy.random.randint(3,8,total_demand)
    change_list   = numpy.random.randint(0,3,total_demand)
    change_thlist = numpy.random.randint(10,30,total_demand) * 10
     
    for i in range(number_demand):
        for j in range(len(destin_demand[i])):
            m_slope = -1
            m_road = -1            
            for m in range(num_of_road):
                if road[m].id1 == origin_demand[i] and road[m].id2 == destin_demand[i][j]:
                    m_slope = road[m].slope
                    m_road  = m
                    break

            header_pos = [-1, -1]
            header_sz  = -1
            header_vel = -1

            vehicle[num] = py_Vehicle(0, int(vehicle_type[num]),
                                      origin_demand[i], destin_demand[i][j],
                                      [city[destin_demand[i][j]].axis_real_x, city[destin_demand[i][j]].axis_real_y], 
                                      [city[origin_demand[i]].axis_real_x, city[origin_demand[i]].axis_real_y],
                                      header_pos, header_sz, header_vel, secure_dist[num],
                                      change_list[num], change_thlist[num])

            vehicle[num].road  = m_road
            vehicle[num].slope = m_slope

            num = num+1
        
    return vehicle

if __name__ == '__main__':
    [name, city, road, directed] = read_city_data("data.igml")
    [origin_demand, destin_demand, number_demand, size_type, vehicle_size] = read_vehicle_data("data.ivml")
    scale = calc_UI_params(city)
    vehicle = initial_vehicle(origin_demand, destin_demand, number_demand, size_type, vehicle_size, city, road, directed)
    