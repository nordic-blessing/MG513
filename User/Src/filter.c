#include "filter.h"
#include "stdlib.h"

float buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 10;

//------低通滤波器------//
float alpha = 0.5; // 滤波器系数
float filtered = 0; // 初始化滤波后的值
float lowPassFilter(float new_value) {
    filtered = alpha * new_value + (1 - alpha) * filtered;
    return filtered;
}

//----滑动平均滤波器----//
float buffer_sum = 0; // 用于存储累加值
float movingAverageFilter(float new_value) {
    // 从累加值中减去将被替换的老值
    buffer_sum -= buffer[buffer_index];
    // 将新值添加到缓冲区并更新累加值
    buffer[buffer_index] = new_value;
    buffer_sum += new_value;
    // 更新索引
    buffer_index = (buffer_index + 1) % FILTER_SIZE;

    // 返回更新后的平均值
    return buffer_sum / FILTER_SIZE;
}


//------中值滤波器------//
int compareFloats(const void *a, const void *b) {
    float f1 = *(float*)a;
    float f2 = *(float*)b;

    // 返回负数、零或正数，使得较小的数排在前面
    return (f1 < f2) ? -1 : (f1 > f2) ? 1 : 0;
}

float medianFilter(float new_value) {
    // 将新值添加到缓冲区
    buffer[buffer_index] = new_value;
    // 更新索引
    buffer_index = (buffer_index + 1) % FILTER_SIZE;

    // 对缓冲区数据进行排序
    qsort(buffer, FILTER_SIZE, sizeof(float), compareFloats);
    // 返回中值
    return buffer[FILTER_SIZE / 2];
}