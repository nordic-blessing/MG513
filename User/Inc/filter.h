#ifndef __FILTER_H__
#define __FILTER_H__

#include "main.h"

#define FILTER_SIZE 5 // 滤波器的大小

float lowPassFilter(float new_value);           //低通滤波器
float movingAverageFilter(float new_value);     //滑动平均滤波器
float medianFilter(float new_value);            //中值滤波器

#endif //__FILTER_H__
