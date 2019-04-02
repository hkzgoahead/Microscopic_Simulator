#ifndef _OD_H
#define _OD_H

#include <stdio.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>

class OD
{
public:
	OD(int m_num, int m_max, int m_min)
	{
		num = m_num;
		max = m_max;
		min = m_min;
	}
	~OD() {}

	int factorial(int n)
	{
		int temp;
		if (n <= 2)
			temp = (unsigned)n;
		else
		{
			temp = 2;
			for (int i = 3; i <= n; i++)
			{
				temp *= i;
			}
		}
		return temp;
	}
	
	void swap(int* a, int* b)
	{
		int temp = *a;
		*a = *b;
		*b = temp;
	}

// generate a new OD series (data.ivml)
	void generateODseries()
	{
		int** OD = new int* [num];
		int*  O  = new int  [num];
		srand((unsigned)time(NULL));
		for (int i = 0; i < num; i++)
		{
			OD[i] = new int [num];
			O[i]  = 0;
			for (int j = 0; j < num; j++)
			{
				if (j != i)
				{
					OD[i][j] = rand()%(max-min) + min;
					O[i] += OD[i][j];
				}
				else 
					OD[i][j] = 0;
			}
		}

		FILE* f;
		f = fopen("./data.ivml", "w");
		fprintf(f, "vehicle [\n");
		fprintf(f, "  total %d\n", num);
		for (int i = 0; i < num; i++)
		{
			int* series = new int [O[i]];
			int k = 0;
			for (int j = 0; j < num; j++)
			{
				for (int m = 0; m < OD[i][j]; m++)
				{
					series[k++] = j;
				}
			}

			srand((unsigned)time(NULL));
			int temp;
			int r1 = 0;
			if (O[i] <= 8)
			{
				temp = factorial(O[i]);
				r1 = rand()%temp;
			}
			else
			{
				temp = 40320;
			  r1 = rand()%temp;
		  }

			int trigger1 = 0;
			int len = O[i];
			while (len > 0)
			{
				while (next_permutation(series, series+len))
				{
					if (trigger1 == r1)
						break;
					trigger1++;
				}
				len = len - 4;
			}
			int loc1;
			int loc2;
			for (int j = 0; j < int(O[i]/4.0); j++)
			{
				loc1 = rand()%O[i];
				loc2 = rand()%O[i];
				swap(&(series[loc1]), &(series[loc2]));
			}

			fprintf(f, "  OD [\n");
			fprintf(f, "    origin %d\n", i);
			fprintf(f, "    destination ");
			for (int j = 0; j < O[i]; j++)
			{
				fprintf(f, "%d ", series[j]);
			}
			fprintf(f, "\n]\n");

			delete [] series;
		}
		
		fclose(f);
		delete [] O;
		delete [] OD;
	}

	int num;
	int max;
	int min;
};

#endif
