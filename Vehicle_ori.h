#ifndef _VEHICLE_H
#define _VEHICLE_H

#include <stdio.h>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <vector>

using namespace std;

class Vehicle
{
public:
 	Vehicle(int m_ID, int m_size, int m_o_id, int m_d_id, float* m_d, float* m_p, float* m_hp, int m_hs, int m_hv, float m_sd, int m_change, int m_ch_th)
	{
		position    = new float [2];
		header_pos  = new float [2];
		origin      = new float [2];
		destination = new float [2];
		temp_origin = new float [2];
		temp_destin = new float [2];
		for (int i = 0; i < 2; i++)
		{
			position[i]    = m_p[i];
			origin[i]      = m_p[i];
			destination[i] = m_d[i];
			header_pos[i]  = m_hp[i];
			temp_origin[i] = 0.0;
			temp_destin[i] = 0.0;
		}

// initial data fields
		road          = -1;
		ID            = m_ID;
		size          = m_size;
		ori_id        = m_o_id;
		des_id        = m_d_id;
		arrived       = 0;
		tmp_arrived   = 0;
		velocity      = 0;
		speed_up_down = 1;
		slope         = 0;
		secure_dist   = m_sd;
		change        = m_change;
		change_threshold = m_ch_th;
    
		header_size   = m_hs;
		header_velo   = m_hv;

// no need to be initial unless the vehicle start at a node
		temp_origin_id = -1;
		temp_destin_id = -1;
		departure_time = -1.0f;

// random to get max_v(below 90) and max_acc_v(below 3.5)
		srand((unsigned)time(NULL));

		max_velocity = rand() % 90;
		max_acc_velo = float((double(100)+rand()%100) / (double(100))) * 3;

// calculate the acc_velocity function parameters
		b_comm = log(1+max_acc_velo) / max_velocity;
		b_risk = b_comm * 1.2;
		b_cons = log(1+0.6*max_acc_velo) / max_velocity;
	}
  ~Vehicle()
	{
		delete [] position;
		delete [] header_pos;
		delete [] origin;
		delete [] destination;
        delete [] temp_origin;
        delete [] temp_destin;
        vector<int>().swap(plan_route);
        vector<int>().swap(route);
        vector<int>().swap(path);
	}

// we can get the slope of the road (road[0]-->road[1]) [see Road.h] to update the position of the vehicle. Anti-clockwise is used.
	void calc_pos(float time);
	void calc_vel(float time);
	float generate_acc_velocity(int red_light, float* m_hp, float m_hv, float m_ha);
	void depend_speed_up_down(int red_light, float* m_hp, float m_hv);

// whether arrived or not
    void is_arrived();
	void is_temp_arrived();

// path choice 
	void shortest_path(float* mat, int N);

	float calc_distance(float x1, float y1, float x2, float y2);

	void detect_jam();
	void detect_collision();

// the data field of Vehicle
	int size;              // size=1: motor OR bike; size=2:car; size=3:bus OR truck, etc
	float* origin;
	float* destination;
	int ori_id;
	int des_id;

	int change;            // 0: never change route plan
                               // 1: change at every cross
                               // 2: change at cross depends on the threshold
	int change_threshold;

	int temp_origin_id;
	float* temp_origin;
	int temp_destin_id;
	float* temp_destin;
	float* position;       // position at present
	int arrived;
	int tmp_arrived;
	int ID;                // label the running vehicles at THAT road
		         // when vehicles switches a road(i.e. from road 1 to road 2), it is troublesome to reset the label of vehicles
		         // on road 1, so we only remember the first car ID and set the maximum vehicle at a road to 1000. When vehicle
		         // (ID first_ID) switches, road_first := second_ID, then recycle the first_ID and allocate for comming vehicle.
	float* header_pos;     // position of the header vehicle at present
	int header_size;       // size of the header vehicle at present
	float header_velo;

// !! IMPORTANT
	int road;            // the index of the road where the vehicle is on
	vector<int> path;    // the path from road[1] to desination
	vector<int> route;   // the whole route of O-D
	vector<int>::iterator it;
	vector<int> plan_route;
	int plan_route_length;
// !! IMPORTANT
  
	float slope;         // the slope of the road present
	float velocity;      // the velocity of the vehicle
	float max_velocity;
	float acc_velocity;  // the accelerated velocity of the vehicle
	float max_acc_velo;
	float b_risk;
	float b_comm;
	float b_cons;
	int speed_up_down;   // whether to speed up or to speed down; it has several levels from -4 to 1
		       // case:     means:
		       //   -3, -4       the header vehicle is close(-4, the header vehicle is too slow)
  	       //   -2, -1       the cross is ahead in 3 seconds ride(-2,red: slow until stop, -1,green: slow)
		       //        0       the destination is 100 meters ahead
		       //        1       the header vehicle is not that close
		       //        2       the header vehilce is further
		       //        3       the way is free

	float head_dist;     // the distance to header vehicle (leading vehicle)
	float cross_dist;    // the distance to the cross (intersection)
	float desti_dist;    // the 
	float secure_dist;   // the secure-distance to keep with the header vehicle
	float departure_time;
};

void Vehicle::shortest_path(float* mat, int N)
{
//	FILE* p = fopen("record.txt", "w");

	float** matrix = new float* [N];
	for (int i = 0; i < N; i++)
	{
		matrix[i] = new float [N];
		for (int j = 0; j < N; j++)
		{
			matrix[i][j] = mat[N*i+j];
//			fprintf(p, "%10f ", matrix[i][j]);
		}
//		fprintf(p, "\n");
	}

	int* pre = new int [N];
	pre[ori_id] = -1;
	for (int i = 0; i < N; i++)
	{
		if (i != ori_id)
		{
			if (matrix[ori_id][i] < 10000.0)
				pre[i] = ori_id;
			else
				pre[i] = -1;
		}
	}
	
//	fprintf(p, "ori_pre: ");
//	for (int i = 0; i < N; i++)
//		fprintf(p, "%d ", pre[i]);
//	fprintf(p, "\n");

	int* label = new int [N];
	for (int i = 0; i < N; i++)
		label[i] = 0;
	label[ori_id] = 1;
	int all_label = 0;

	while (all_label == 0)
	{
		float min = 10000.0;
		int id = -1;
		for (int i = 0; i < N; i++)
		{
			if (min > matrix[ori_id][i] && label[i] == 0)
			{
				min = matrix[ori_id][i];
				id = i;
			}
		}
		label[id] = 1;
//		fprintf(p, "point: %d\n", id);
		
		for (int i = 0; i < N; i++)
		{
			if (i != ori_id)
			{
				if (matrix[ori_id][i] > matrix[ori_id][id]+matrix[id][i])
				{
					matrix[ori_id][i] = matrix[ori_id][id]+matrix[id][i];
					pre[i] = id;
				}
			}
		}

		int i;
		for (i = 0; i < N; i++)
			if (label[i] == 0)
				break;

		if (i==N)
			all_label = 1;
	}

//	fprintf(p, "pre: ");
//	for (int i = 0; i < N; i++)
//		fprintf(p, "%d ", pre[i]);
//	fprintf(p, "\n");

	int temp = des_id;
	plan_route.push_back(des_id);
	plan_route_length = 1;
	if (pre[temp] != ori_id)
	{
		do
		{
//			fprintf(p, "%d-->", pre[temp]);
			plan_route.push_back(pre[temp]);
			temp = pre[temp];
			plan_route_length++;
		}
		while (pre[temp] != ori_id);
	}
//	plan_route.push_back(ori_id);
//	fclose(p);

	temp_origin_id = ori_id;
	it = plan_route.end()-1;                    // NOTICE: use the iterator here!
	temp_destin_id = *it;

	for (int i = 0; i < 2; i++)
		temp_origin[i] = origin[i];

	if (plan_route_length == 1)
		for (int i = 0; i < 2; i++)
			temp_destin[i] = destination[i];

	plan_route.erase(it--);
    
}	

void Vehicle::depend_speed_up_down(int red_light, float* m_hp, float m_hv)
{
/* ----------------parse----------------
  red_light : if the red light will begin in 4 seconds AND will not end is 4 seconds
  c_x, c_y  : get the position of the cross from main function
   -----------------end----------------- */
	header_pos[0] = m_hp[0];
	header_pos[1] = m_hp[1];
	header_velo = m_hv;
	
	desti_dist = calc_distance(position[0], position[1], destination[0], destination[1]);
	
	if (header_pos[0] == -1.0 && header_pos[1] == -1.0)
    //no leading vehicle
		head_dist = desti_dist;
	else
    //leading vehicle = header_
		head_dist  = calc_distance(position[0], position[1], header_pos[0], header_pos[1]);

	if (plan_route_length == 1)
    
		cross_dist = desti_dist;
	else
		cross_dist = calc_distance(position[0], position[1], temp_destin[0], temp_destin[1]);

	speed_up_down = 2;
	if (head_dist < secure_dist)
	{
		if (header_velo < 20)
			speed_up_down = -4;
		else
			speed_up_down = -3;
	}
	else if (cross_dist < 50 || desti_dist < 50)
	{
		if (cross_dist < desti_dist)
		{
			if (red_light == 1)
				speed_up_down = -2;
			else
				speed_up_down = -1;
		}
		else
			speed_up_down =0;
	}
	else
	{
		if (head_dist > 5 * secure_dist)
			speed_up_down = 3;
		else if (head_dist > 2 * secure_dist)
			speed_up_down = 2;
		else if (head_dist > secure_dist)
			speed_up_down = 1;
	}
}

float Vehicle::generate_acc_velocity(int red_light, float* m_hp, float m_hv, float m_ha)
{
	int last_pattern = speed_up_down;
	depend_speed_up_down(red_light, m_hp, m_hv);
  
	if (speed_up_down == 0)
	{
		if (last_pattern != 0)
		{
			if (velocity > 0)
				acc_velocity = pow(velocity,2) * 0.5 / desti_dist;
			else
			{
				velocity = 10;
				acc_velocity = pow(velocity,2) * 0.5 / desti_dist;
			}
		}
	}
	else if (speed_up_down == -1)
	{
		if (last_pattern != -1)
		{
			acc_velocity = 0.34375 * pow(velocity, 2) / cross_dist;
		}
	}
	else if (speed_up_down == -2)
	{
		if (last_pattern != -2)
		{
			acc_velocity = pow(velocity, 2) / cross_dist;
		}
	}
	else if (speed_up_down == -4)
	{
		if (last_pattern != -4)
		{
			if (head_dist > 0)
				acc_velocity = pow(velocity, 2) / head_dist;
			else
			{
				velocity = 0;
				acc_velocity = 0;
			}
		}
	}
	else if (speed_up_down == -3 || speed_up_down == 3)
	{
		float risk_acc = 0.0f;
		float comm_acc = 0.0f;
		float cons_acc = 0.0f;
		// random to get 3 weights;
		srand((unsigned)time(NULL));
		float weight1 = float(rand()%10 / (double(10)));
		float weight2 = float(rand()%10 / (double(10)));
		float weight3 = float(rand()%10 / (double(10)));
		float total_weight = weight1 + weight2 + weight3;
		weight1 = weight1 / total_weight;
		weight2 = weight2 / total_weight;
		weight3 = weight3 / total_weight;
		float delta_v = max_velocity - velocity;
		if (velocity < 0.17 * max_velocity)
		risk_acc = max_acc_velo;
		else 
		risk_acc = exp(b_risk * delta_v) - 1;
		comm_acc = exp(b_comm * delta_v) - 1;
		cons_acc = exp(b_cons * delta_v) - 1;

		acc_velocity = weight1 * risk_acc + weight2 * comm_acc + weight3 * cons_acc;
	}
	else if (speed_up_down == 1)
		acc_velocity = 0.5;
	else 
		acc_velocity = 2;
    
    return m_ha;
}

void Vehicle::calc_vel(float time)
{
//  generate_acc_velocity();
	if (speed_up_down > 0)
		velocity = velocity + acc_velocity * time;
	else 
		velocity = velocity - acc_velocity * time;

	if (velocity < 0.0)
		velocity = 0.0;
}

void Vehicle::calc_pos(float time)
{
	float s = velocity * time;
	float k = abs(tan(slope/180*3.14159265));
	float delta_x = sqrt(1/(1+pow(k,2))) * s;
	float delta_y = k * delta_x;
	if (slope <= 90) 
	{
		position[0] += delta_x;
		position[1] += delta_y;
	}
	else if (slope <= 180) 
	{
		position[0] -= delta_x;
		position[1] += delta_y;
	}
	else if (slope <= 270) 
	{
		position[0] -= delta_x;
		position[1] -= delta_y;
	}
	else
	{
		position[0] += delta_x;
		position[1] -= delta_y;
	}
}

void Vehicle::is_arrived()
{
	if (desti_dist < 5.0)
	{
		arrived = 1;
		position[0] = destination[0];
		position[1] = destination[1];
	}
	else 
		arrived = 0;
}

void Vehicle::is_temp_arrived()
{
	if (cross_dist < 5.0)
	{
		tmp_arrived = 1;
		position[0] = temp_destin[0];
		position[1] = temp_destin[1];

		if (plan_route_length > 0)
		{
			plan_route_length--;
			temp_origin_id = temp_destin_id;
			temp_destin_id = *it;
			
			for (int i = 0; i < 2; i++)
				temp_origin[i] = temp_destin[i];

			plan_route.erase(it--);
		}
	}
	else
		tmp_arrived = 0;
}

float Vehicle::calc_distance(float x1, float y1, float x2, float y2)
{
	float distance = sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
	return distance;
}

void Vehicle::detect_jam()
{}

void Vehicle::detect_collision()
{}

#endif
