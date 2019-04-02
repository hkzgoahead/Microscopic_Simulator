#ifndef _ITS_H
#define _ITS_H

#include <stdio.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include "Node.h"
#include "Road.h"
#include "Vehicle.h"
#include "OD.h"

class ITS
{
public:
	ITS(int m_num0, int m_num1, int m_num2, int m_num3, int m_num4, int m_t0, int m_t1, int m_t2, int m_t3, int m_t4, int m_l0, int m_l1, int m_l2, int m_l3, int m_p0, int m_p1, int m_p2, int m_p3, int m_p4, float m_lr)
  {
		if (m_num0 != 0 && m_num1 != 0 && m_num2 != 0 && m_num3 != 0 && m_num4 != 0)
		{
    	preFlowDataNum = new int [5];
    	preFlowDataNum[0] = m_num0;
    	preFlowDataNum[1] = m_num1;
    	preFlowDataNum[2] = m_num2;
    	preFlowDataNum[3] = m_num3;
    	preFlowDataNum[4] = m_num4;

   		preFlowWeight = new float [5]; 
    	preFlowAlpha  = new float* [5];
    	preFlowData   = new int* [5];
    
    	float temp_WeightSum = 0.0f;
    	srand((unsigned)time(NULL));
    	for (int i = 0; i < 5; i++)
    	{
      	preFlowWeight[i] = rand()%10000;
      	preFlowWeight[i] /= 10000; 
      	temp_WeightSum += preFlowWeight[i];

      	preFlowAlpha[i] = new float [preFlowDataNum[i]];
      	float temp_AlphaSum = 0.0f;
      	for (int j = 0; j < preFlowDataNum[i]; j++)
      	{
					preFlowAlpha[i][j] = rand()%10000;
					preFlowAlpha[i][j] /= 10000;
					temp_AlphaSum += preFlowAlpha[i][j];
      	}
      	for (int j = 0; j < preFlowDataNum[i]; j++)
      	{
        	preFlowAlpha[i][j] /= temp_AlphaSum;
     		}

      	if (i == 4)
     		{
					sort(preFlowAlpha[i], preFlowAlpha[i]+preFlowDataNum[i]);
      	}
      	preFlowData[i] = new int [preFlowDataNum[i]];
    	}
    	preFlowData[0][0] = m_t0;
    	preFlowData[0][1] = m_t1;
    	preFlowData[0][2] = m_t2;
    	preFlowData[0][3] = m_t3;
    	preFlowData[0][4] = m_t4;
    	preFlowData[1][0] = 0;
    	preFlowData[1][1] = 0;
    	preFlowData[1][2] = 0;
    	preFlowData[2][0] = m_l0;
    	preFlowData[2][1] = m_l1;
    	preFlowData[2][2] = m_l2;
    	preFlowData[2][3] = m_l3;
    	preFlowData[3][0] = m_p0;
    	preFlowData[3][1] = m_p1;
    	preFlowData[3][2] = m_p2;
    	preFlowData[3][3] = m_p3;
    	preFlowData[3][4] = m_p4;
    	for (int i = 2; i >= 0; i--)
    	{
      	preFlowData[4][i] = preFlowData[0][i+2] - preFlowData[0][i+1];
    	}
	
    	for (int i = 0; i < 5; i++)
    	{
      	preFlowWeight[i] /= temp_WeightSum;
    	}
			preFlowLearnRate = m_lr;
  	}
	}
  ~ITS() {}

  void predict_flow()
  {
    float sum = 0.0f;
    float temp_sum = 0.0f;
    for (int i = 0; i < 5; i++)
    {
      temp_sum = 0.0f;
      for (int j = 0; j < preFlowDataNum[i]; j++)
      {
				temp_sum += preFlowAlpha[i][j] * preFlowData[i][j];
      }
			if (i == 4)
				temp_sum *= 0.1;
      sum += preFlowWeight[i] * temp_sum;
    }
		if (sum-int(sum) >= 0.5)
			preFlow = int(sum)+1;
		else
			preFlow = int(sum);
  }

  void predict_flow_parameter_tuning(int real_data)
  {
	  int e = real_data - preFlow;
		float error = float(e);
		if (abs(error)/float(real_data) < 0.20 || error < 0.0)
			preFlowLearnRate = 0.0005;

		for (int i = 0; i < 5; i++)
		{
			preFlowWeight[i] *= 1+preFlowLearnRate*error;
		}
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < preFlowDataNum[i]; j++)
			{
				preFlowAlpha[i][j] *= 1+preFlowWeight[i]*preFlowLearnRate*error;
			}
			if (i==4)
			{
				sort(preFlowAlpha[i], preFlowAlpha[i]+preFlowDataNum[i]);
			}
		}
	}

  void predict_flow_data_updating(int real_data, int m_l0, int m_l1, int m_l2, int m_l3, int m_p0, int m_p1, int m_p2, int m_p3, int m_p4)
  {
    for (int i = 0; i < 5; i++)
    {
      if (i != 2 && i != 3)
      {
				for (int j = 0; j < preFlowDataNum[i]-1; j++)
				{
					preFlowData[i][j] = preFlowData[i][j+1];
        }
      }
    }
    preFlowData[0][preFlowDataNum[0]-1] = real_data;
    preFlowData[1][preFlowDataNum[1]-1] = real_data - preFlow;
    preFlowData[4][preFlowDataNum[4]-1] = real_data - preFlowData[0][preFlowDataNum[0]-2];
    
    preFlowData[2][0] = m_l0;
    preFlowData[2][1] = m_l1;
    preFlowData[2][2] = m_l2;
    preFlowData[2][3] = m_l3;
    preFlowData[3][0] = m_p0;
    preFlowData[3][1] = m_p1;
    preFlowData[3][2] = m_p2;
    preFlowData[3][3] = m_p3;
    preFlowData[3][4] = m_p4;
  }
  
// data for traffic flow prediction
  int* preFlowDataNum;
  float preFlowLearnRate;
  int preFlow;
  float* preFlowWeight;
  float** preFlowAlpha;
  int** preFlowData;
};


#endif
