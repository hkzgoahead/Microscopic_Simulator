# distutils: language = c++
from cpython cimport array

cdef extern from "Node.h":
    cdef cppclass Node:
        Node(int, float, float) except +
        void free_node()
        void add_node(int, float, float)
        
        int id
#       char** name
        float axis_real_x
        float axis_real_y
        float axis_canvas_x
        float axis_canvas_y
        int* link
        float* dist

cdef class py_Node:
    cdef Node* thisptr
    def __cinit__(self, int m_id, float m_x, float m_y):
        self.thisptr = new Node(m_id, m_x, m_y)
    def __dealloc__(self):
        if self.thisptr:
            del self.thisptr
    
    def __add_node__(self, int m_next_id, float x, float y):
        self.thisptr.add_node(m_next_id, x, y)
        
    property id:
        def __get__(self): return self.thisptr.id
        def __set__(self, m_id): self.thisptr.id = m_id
    property axis_real_x:
        def __get__(self): return self.thisptr.axis_real_x
        def __set__(self, m_a_x): self.thisptr.axis_real_x = m_a_x
    property axis_real_y:
        def __get__(self): return self.thisptr.axis_real_y
        def __set__(self, m_a_y): self.thisptr.axis_real_y = m_a_y
    property axis_canvas_x:
        def __get__(self): return self.thisptr.axis_canvas_x
        def __set__(self, m_c_x): self.thisptr.axis_canvas_x = m_c_x
    property axis_canvas_y:
        def __get__(self): return self.thisptr.axis_canvas_y
        def __set__(self, m_c_y): self.thisptr.axis_canvas_y = m_c_y
    property link:
        def __get__(self): return [self.thisptr.link[0], self.thisptr.link[1], self.thisptr.link[2], self.thisptr.link[3]]
    property dist:
        def __get__(self): return [self.thisptr.dist[0], self.thisptr.dist[1], self.thisptr.dist[2], self.thisptr.dist[3]]
        
cdef extern from "Road.h":
    cdef cppclass Road:
        Road(int, int, int, int, int, int) except +
        void cal_length_slope(float, float, float, float)
        void judge_jam()
        void cal_ave_velocity()
        
        int id1
        int id2
        int lane1
        int lane2
        float length
        float slope
        int light_time
        int green_time
        int flow
        int jamOrNot
        float ave_velocity

cdef class py_Road:
    cdef Road* thisptr
    def __cinit__(self, m_id1, m_id2, m_l1, m_l2, m_t, m_gt):
        self.thisptr = new Road(m_id1, m_id2, m_l1, m_l2, m_t, m_gt)
    def __dealloc__(self):
        del self.thisptr
        
    def __cal_length_slope__(self, x1, x2, y1, y2):
        self.thisptr.cal_length_slope(x1, x2, y1, y2)    
    def __cal_ave_velocity__(self):
        self.thisptr.cal_ave_velocity()
    def __judge_jam__(self):
        self.thisptr.judge_jam()
        
    property id1:
        def __get__(self): return self.thisptr.id1
        def __set__(self, m_id1): self.thisptr.id1 = m_id1
    property id2:
        def __get__(self): return self.thisptr.id2
        def __set__(self, m_id2): self.thisptr.id2 = m_id2
    property lane1:
        def __get__(self): return self.thisptr.lane1
        def __set__(self, l1): self.thisptr.lane1 = l1
    property lane2:
        def __get__(self): return self.thisptr.lane2
        def __set__(self, l2): self.thisptr.lane2 = l2
    property light_time:
        def __get__(self): return self.thisptr.light_time
        def __set__(self, t): self.thisptr.light_time = t
    property green_time:
        def __get__(self): return self.thisptr.green_time
        def __set__(self, gt): self.thisptr.green_time = gt
    property length:
        def __get__(self): return self.thisptr.length
        def __set__(self, l): self.thisptr.length = l
    property slope:
        def __get__(self): return self.thisptr.slope
        def __set__(self, s): self.thisptr.slope = s
    property flow:
        def __get__(self): return self.thisptr.flow
        def __set__(self, f): self.thisptr.flow = f
    property jamOrNot:
        def __get__(self): return self.thisptr.jamOrNot
        def __set__(self, j): self.thisptr.jamOrNot = j
    property ave_velocity:
        def __get__(self): return self.thisptr.ave_velocity
        def __set__(self, av): self.thisptr.ave_velocity = av
        
cdef extern from "Vehicle.h":
    cdef cppclass Vehicle:
        Vehicle(int, int, int, int, float*, float*, float*, int, int, float, int, int) except +
        void calc_pos(float)
        void calc_vel(float)
        float calc_distance(float, float, float, float)
        float generate_acc_velocity(int, float*, float, float)
        void depend_speed_up_down(int, float*, float)
        void detect_jam()
        void detect_collision()
        void is_arrived()
        void is_temp_arrived()
        void shortest_path(float*, int)
        
        int size
        float* origin
        float* destination
        int ori_id
        int des_id
        int change
        int change_threshold
        int temp_origin_id
        int temp_destin_id
        float* temp_origin
        float* temp_destin
        float* position
        int arrived
        int tmp_arrived
        int ID
        float* header_pos
        int header_size
        float header_velo
        int road
        int plan_route_length
#        int* path
#        int* route
#        int* plan_route
        float slope
        float velocity
        float max_velocity
        float acc_velocity
        float max_acc_velo
        float b_risk
        float b_comm
        float b_cons
        int speed_up_down
        float head_dist
        float cross_dist
        float desti_dist
        float secure_dist
        float departure_time

cdef class py_Vehicle:
    cdef Vehicle* thisptr
    def __cinit__(self, m_ID, m_size, m_o_id, m_d_id, m_d, m_p, m_hp, m_hs, m_hv, m_sd, m_ch, m_ch_th):
        self.thisptr = new Vehicle(m_ID, m_size, m_o_id, m_d_id, [m_d[0], m_d[1]], [m_p[0], m_p[1]], [m_hp[0], m_hp[1]], 
                                   m_hs, m_hv, m_sd, m_ch, m_ch_th)
    def __dealloc__(self):
        del self.thisptr
    
    def __calc_pos__(self, time):
        self.thisptr.calc_pos(time)
    def __calc_vel__(self, time):
        self.thisptr.calc_vel(time)
    def __calc_distance(self, x1, y1, x2, y2):
        self.thisptr.calc_distance(x1, y1, x2, y2)
    def __generate_acc_velocity__(self, red_light, m_hp, m_hv, m_ha):
        self.thisptr.generate_acc_velocity(red_light, [m_hp[0], m_hp[1]], m_hv, m_ha)
    def __depend_speed_up_down__(self, red_light, m_hp, m_hv):
        self.thisptr.depend_speed_up_down(red_light, [m_hp[0], m_hp[1]], m_hv)
    def __is_arrived__(self):
        self.thisptr.is_arrived()
    def __is_temp_arrived__(self):
        self.thisptr.is_temp_arrived()
    def __shortest_path__(self, m, numOfNodes):
        cdef array.array mat = array.array('f', m)
        self.thisptr.shortest_path(mat.data.as_floats, numOfNodes)
    def __detect_jam__(self):
        self.thisptr.detect_jam()
    def __detect_collision__(self):
        self.thisptr.detect_collision()
        
    property size:
        def __get__(self): return self.thisptr.size
        def __set__(self, m_size): self.thisptr.size = m_size
    property origin:
        def __get__(self): return [self.thisptr.origin[0], self.thisptr.origin[1]]
    property destination:
        def __get__(self): return [self.thisptr.destination[0], self.thisptr.destination[1]]
    property ori_id:
        def __get__(self): return self.thisptr.ori_id
        def __set__(self, m_o_id): self.thisptr.ori_id = m_o_id
    property des_id:
        def __get__(self): return self.thisptr.des_id
        def __set__(self, m_d_id): self.thisptr.des_id = m_d_id
    property change:
        def __get__(self): return self.thisptr.change
        def __set__(self, m_ch): self.thisptr.change = m_ch
    property change_threshold:
        def __get__(self): return self.thisptr.change_threshold
        def __set__(self, m_ch_th): self.thisptr.change_threshold = m_ch_th
    property temp_origin:
        def __get__(self): return [self.thisptr.temp_origin[0], self.thisptr.temp_origin[1]]
        def __set__(self, m_tempOri): 
            self.thisptr.temp_origin[0] = m_tempOri[0]
            self.thisptr.temp_origin[1] = m_tempOri[1]
    property temp_destin:
        def __get__(self): return [self.thisptr.temp_destin[0], self.thisptr.temp_destin[1]]
        def __set__(self, m_tempDes): 
            self.thisptr.temp_destin[0] = m_tempDes[0]
            self.thisptr.temp_destin[1] = m_tempDes[1]
    property temp_origin_id:
        def __get__(self): return self.thisptr.temp_origin_id
        def __set__(self, m_tempOri_id): self.thisptr.temp_origin_id = m_tempOri_id
    property temp_destin_id:
        def __get__(self): return self.thisptr.temp_destin_id
        def __set__(self, m_tempDes_id): self.thisptr.temp_destin_id = m_tempDes_id
    property position:
        def __get__(self): return [self.thisptr.position[0], self.thisptr.position[1]]
        def __set__(self, p): 
            self.thisptr.position[0] = p[0]
            self.thisptr.position[1] = p[1]
    property arrived:
        def __get__(self): return self.thisptr.arrived
        def __set__(self, m_a): self.thisptr.arrived = m_a
    property temp_arrived:
        def __get__(self): return self.thisptr.tmp_arrived
        def __set__(self, m_ta): self.thisptr.tmp_arrived = m_ta
    property ID:
        def __get__(self): return self.thisptr.ID
        def __set__(self, m_ID): self.thisptr.ID = m_ID
    property header_pos:
        def __get__(self): return [self.thisptr.header_pos[0], self.thisptr.header_pos[1]]
        def __set__(self, hp):
            self.thisptr.header_pos[0] = hp[0]
            self.thisptr.header_pos[1] = hp[1]
    property header_size:
        def __get__(self): return self.thisptr.header_size
        def __set__(self, m_hs): self.thisptr.header_size = m_hs
    property header_velo:
        def __get__(self): return self.thisptr.header_velo
        def __set__(self, m_hv): self.thisptr.header_velo = m_hv
    property road:
        def __get__(self): return self.thisptr.road
        def __set__(self, m_r): self.thisptr.road = m_r
    property plan_route_length:
        def __get__(self): return self.thisptr.plan_route_length
    property slope:
        def __get__(self): return self.thisptr.slope
        def __set__(self, m_s): self.thisptr.slope = m_s
    property velocity:
        def __get__(self): return self.thisptr.velocity
        def __set__(self, m_v): self.thisptr.velocity = m_v
    property max_velocity:
        def __get__(self): return self.thisptr.max_velocity
        def __set__(self, m_max_v): self.thisptr.max_velocity = m_max_v
    property acc_velocity:
        def __get__(self): return self.thisptr.acc_velocity
        def __set__(self, m_acc_v): self.thisptr.acc_velocity = m_acc_v
    property max_acc_velo:
        def __get__(self): return self.thisptr.max_acc_velo
        def __set__(self, m_max_a): self.thisptr.max_acc_velo = m_max_a
    property b_risk:
        def __get__(self): return self.thisptr.b_risk
        def __set__(self, m_b_risk): self.thisptr.b_risk = m_b_risk
    property b_comm:
        def __get__(self): return self.thisptr.b_comm
        def __set__(self, m_b_comm): self.thisptr.b_comm = m_b_comm
    property b_cons:
        def __get__(self): return self.thisptr.b_cons
        def __set__(self, m_c_cons): self.thisptr.b_cons = m_c_cons
    property speed_up_down:
        def __get__(self): return self.thisptr.speed_up_down
        def __set__(self, m_sud): self.thisptr.speed_up_down = m_sud
    property head_dist:
        def __get__(self): return self.thisptr.head_dist
        def __set__(self, m_hd): self.thisptr.head_dist = m_hd
    property cross_dist:
        def __get__(self): return self.thisptr.cross_dist
        def __set__(self, m_cd): self.thisptr.cross_dist = m_cd
    property desti_dist:
        def __get__(self): return self.thisptr.desti_dist
        def __set__(self, m_dd): self.thisptr.desti_dist = m_dd
    property secure_dist:
        def __get__(self): return self.thisptr.secure_dist
        def __set__(self, m_sd): self.thisptr.secure_dist = m_sd
    property departure_time:
        def __get__(self): return self.thisptr.departure_time
        def __set__(self, m_dt): self.thisptr.departure_time = m_dt

cdef extern from "OD.h":
    cdef cppclass OD:
        OD(int, int, int)
        void generateODseries()
        
        int num
        int max
        int min
    
cdef class py_OD:
    cdef OD* thisptr
    def __cinit__(self, m_num, m_max, m_min):
        self.thisptr = new OD(m_num, m_max, m_min)
    def __generateODseries__(self):
        self.thisptr.generateODseries()
        
    property num:
        def __get__(self): return self.thisptr.num
        def __set__(self, m_num): self.thisptr.num = m_num
    property max:
        def __get__(self): return self.thisptr.max
        def __set__(self, m_max): self.thisptr.max = m_max
    property min:
        def __get__(self): return self.thisptr.min
        def __set__(self, m_min): self.thisptr.min = m_min

cdef extern from "ITS.h":
    cdef cppclass ITS:
        ITS(int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, float) except +
        void predict_flow()
        void predict_flow_parameter_tuning(int)
        void predict_flow_data_updating(int, int, int, int, int, int, int, int, int, int)
        
        int* preFlowDataNum
        float preFlowLearnRate
        int preFlow
        float* preFlowWeight
        float** preFlowAlpha
        int** preFlowData

cdef class py_ITS:
    cdef ITS* thisptr
    def __cinit__(self, n0, n1, n2, n3, n4, t0, t1, t2, t3, t4, l0, l1, l2, l3, p0, p1, p2, p3, p4, lr):
        self.thisptr = new ITS(n0, n1, n2, n3, n4, t0, t1, t2, t3, t4, l0, l1, l2, l3, p0, p1, p2, p3, p4, lr)
    def __dealloc__(self):
        del self.thisptr
    
    def __predict_flow__(self):
        self.thisptr.predict_flow()
    def __predict_flow_parameter_tuning__(self, real_data):
        self.thisptr.predict_flow_parameter_tuning(real_data)
    def __predict_flow_data_updating__(self, real_data, l0, l1, l2, l3, p0, p1, p2, p3, p4):
        self.thisptr.predict_flow_data_updating(real_data, l0, l1, l2, l3, p0, p1, p2, p3, p4)
    
    property preFlow:
        def __get__(self): return self.thisptr.preFlow
        def __set__(self, m_pF): self.thisptr.preFlow = m_pF
