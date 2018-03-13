#include <stdio.h>
#include <math.h>
#include <stdlib.h>



struct Voltage {
	int min1;
	int min2;
	int max1;
	int max2;
	int mid_point;
	int delta;
	int zeroCross[3];
	int numOfMax;
	int numOfMin;
	int max1_index;
	int max2_index;
	int min1_index;
	int min2_index;
};

#define num_size 20

void sample_ADC(int buffer[num_size], int size, struct Voltage *d);
void volt_zeroDetector(int buffer[num_size], struct Voltage *d);


int main()
{
	int arr[num_size] = {55, 62, 68, 80, 97, 100, 92, 82, 60, 46, 40, 50, 64, 77, 88, 98, 79, 66, 60, 55 }; //{ 55, 45, 36, 25, 17, 10, 16, 24, 35, 56, 70, 63, 51, 42, 33, 24, 8, 17, 25, 35 };
	
	struct Voltage V = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
	int i = 0;

	sample_ADC(arr, num_size, &V);
	//volt_zeroDetector(arr, &V);

	printf("min: %d\n", V.min1);
	printf("min2: %d\n", V.min2);
	printf("max1: %d\n", V.max1);
	printf("max2: %d\n", V.max2);
	printf("mid: %d\n", V.mid_point);
	printf("delta: %d\n", V.delta);
	for (i = 0; i < 3; i++)
	{
		printf("zeros: %d\n", V.zeroCross[i]);
	}
	printf("#ofMax: %d\n", V.numOfMax);
	printf("#ofMin: %d\n", V.numOfMin);
	printf("max1_index: %d\n", V.max1_index);
	printf("max2_index: %d\n", V.max2_index);
	printf("min1_index: %d\n", V.min1_index);
	printf("min2_index: %d\n", V.min2_index);



	getchar();

	
	return 0;
}

void sample_ADC(int buffer[num_size], int size, struct Voltage *d) 
{
	int min1_ = 0;
	int min2_ = 0;
	int max1_ = 0;
	int max2_ = 0;
	int delta_ = 0;
	int mid_ = 0;
	int i = 0;
	int peakAvg = 0;
	int minAvg = 0;

	min1_ = buffer[0];
	min2_ = buffer[0];
	max1_ = buffer[0];
	max2_ = buffer[0];

	for (i = 0; i <= size>>1; i++)
	{
		if (buffer[i] > max1_)
		{
			max1_ = buffer[i];
			d->max1_index = i;
		}
		if (buffer[i] < min1_)
		{
			min1_ = buffer[i];
			d->min1_index = i;
		}
	}

	for (i = size>>1; i < size; i++)
	{
		if (buffer[i] > max2_)
		{
			max2_ = buffer[i];
			d->max2_index = i;
		}
		if (buffer[i] < min2_)
		{
			min2_ = buffer[i];
			d->min2_index = i;
		}
	}

	d->min1 = min1_;
	d->min2 = min2_;
	d->max1 = max1_;
	d->max2 = max2_;

	if (d->max1 == max1_ && d->max1 != 0)
	{
		d->numOfMax++;
	}
	if (d->max2 == max2_ && d->max2 != 0 && d->max1_index != d->max2_index)
	{
		d->numOfMax++;
	}
	if (d->min1 == min1_ && d->min1 != 0)
	{
		d->numOfMin++;
	}
	if (d->min2 == min2_ && d->min2 != 0 && d->min1_index != d->min2_index)
	{
		d->numOfMin++;
	}

	if (d->numOfMax == 2 && d->numOfMin == 1)
	{
		d->min2 = 0;
		d->min2_index = 0;
		peakAvg = (d->max1 + d->max2) >> 1;
		delta_ = peakAvg - d->min1;
		d->delta = delta_;

		mid_ = d->delta >> 1; //bit shift for dividing by 2 to save CPU cycles
		if (mid_ != 0)
		{
			mid_ = mid_ + min1_;
		}
		d->mid_point = mid_;

	}

	if (d->numOfMax == 1 && d->numOfMin == 2)
	{
		d->max1 = 0;
		d->max1_index = 0;
		minAvg = (d->min1 + d->min2) >> 1;
		delta_ = d->max2 - minAvg;
		d->delta = delta_;

		mid_ = d->delta >> 1; //bit shift for dividing by 2 to save CPU cycles
		if (mid_ != 0)
		{
			mid_ = mid_ + minAvg;
		}
		d->mid_point = mid_;

	}

	//delta_ = max_ - min_;
	//d->delta = delta_;

	
	
}

void volt_zeroDetector(int buffer[num_size], struct Voltage *d)
{
	int diff, cdiff, prevDiff = 0;
	int i = 0;
	int count = 0;
	int zero = 0;


	//set diff to a value to compare too
	if (buffer[0] > d->mid_point)
	{
		diff = buffer[0] - d->mid_point;
	}
	else // buffer[0] < d->midpoint
	{
		diff = d->mid_point - buffer[0];
	}

	//Run through buffer to find smallest differences between mid_point and value in the buffer at
	//an index. The value in the buffer at that index that results in the smallest difference is designated as the zero
	//cross. Buffer should be long enough to get 2 good zero crosses.


}