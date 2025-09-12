/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-10 15:16:59
 * @LastEditors  : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @LastEditTime : 2024-12-10 16:13:26
 * @FilePath     : /calib_reflectivity/include/Spline.h
 * @Description  : 插值
 * 
 * Copyright (c) 2024 by vitalchem, All Rights Reserved. 
 */
#pragma once
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

// 二次样条
struct SplineQ {
    float* a;
    float* b;
    float* c;
    const float* x;
    int size;
};

// 三次样条
struct SplineC {
    float* a;
    float* b;
    float* c;
    float* d;
    const float* x;
    int size;
};

float min_x = 0.0f;
float min_y = 0.0f;
float max_x = 1600.0f;
float max_y = 1.0f;

// 线性外推函数
float linearExtrapolate(const float* x, const float* y, int n, float x_val) {
    if (x_val < x[0]) {
        float t = (x_val - min_x) / (x[0] - min_x); // 占比
        float result = min_y + t * (y[0] - min_y); // 比例法
        return std::clamp(result, 0.0f, 1.0f);
    }
    if (x_val > x[n - 1]) {
        //float result = y[n-1] + (y[n-1] - y[0]) / (x[n-1] - x[0]) * (x_val - x[n-1]); // 斜率法
        float result = y[n - 1] + (x_val - x[n - 1]) / (max_x - x[n - 1]) * (max_y - y[n - 1]); // 比例法
        return std::clamp(result, 0.0f, 1.0f);
    }
    return -1.f; // 无效值
}

// 线性插值
float linearInterp(const float* x, const float* y, int n, float x_val) {
    if (x_val < x[0] || x_val > x[n - 1]) {
        return linearExtrapolate(x, y, n, x_val);
    }

    int left = 0, right = n - 1;
    while (right - left > 1) {
        int mid = (left + right) >> 1;
        if (x_val < x[mid]) {
            right = mid;
        }
        else {
            left = mid;
        }
    }

    float t = (x_val - x[left]) / (x[right] - x[left]);
    return y[left] + t * (y[right] - y[left]);
}

// 二次样条的构建
SplineQ splineQuadratic(const float* x, const float* y, int n) {
    float* a = new float[n];
    float* b = new float[n - 1];
    float* c = new float[n - 1];

    for (int i = 0; i < n; ++i) {
        a[i] = y[i];
    }

    for (int i = 0; i < n - 1; ++i) {
        b[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    }

    for (int i = 1; i < n - 1; ++i) {
        c[i] = (b[i] - b[i - 1]) / (x[i + 1] - x[i - 1]);
    }
    c[0] = c[n - 2] = 0;

    return { a, b, c, x, n };
}

// 二次样条插值
float splineEvalQ(const SplineQ& spline, float x_val) {
    const float* a = spline.a;
    const float* b = spline.b;
    const float* c = spline.c;
    const float* x = spline.x;
    int n = spline.size;

    if (x_val < x[0] || x_val > x[n - 1]) {
        return linearExtrapolate(x, a, n, x_val);
    }

    int left = 0, right = n - 1;
    while (right - left > 1) {
        int mid = (left + right) / 2;
        if (x_val < x[mid]) {
            right = mid;
        }
        else {
            left = mid;
        }
    }

    float dx = x_val - x[left];
    return a[left] + b[left] * dx + c[left] * dx * dx;
}

// 三次样条的构建
SplineC splineCubic(const float* x, const float* y, int n) {
    float* a = new float[n];
    float* b = new float[n - 1];
    float* c = new float[n];
    float* d = new float[n - 1];
    float* h = new float[n - 1];
    float* alpha = new float[n - 1];
    float* l = new float[n];
    float* mu = new float[n];
    float* z = new float[n];

    for (int i = 0; i < n; ++i) {
        a[i] = y[i];
    }

    for (int i = 0; i < n - 1; ++i) {
        h[i] = x[i + 1] - x[i];
    }

    for (int i = 1; i < n - 1; ++i) {
        alpha[i] = (3.0f / h[i]) * (a[i + 1] - a[i]) - (3.0f / h[i - 1]) * (a[i] - a[i - 1]);
    }

    l[0] = 1.0f;
    mu[0] = z[0] = 0.0f;

    for (int i = 1; i < n - 1; ++i) {
        l[i] = 2.0f * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0f;
    z[n - 1] = 0.0f;
    c[n - 1] = 0.0f;

    for (int j = n - 2; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0f * c[j]) / 3.0f;
        d[j] = (c[j + 1] - c[j]) / (3.0f * h[j]);
    }

    delete[] h;
    delete[] alpha;
    delete[] l;
    delete[] mu;
    delete[] z;

    return { a, b, c, d, x, n };
}

// 三次样条插值
float splineEvalC(const SplineC& spline, float x_val) {
    const float* a = spline.a;
    const float* b = spline.b;
    const float* c = spline.c;
    const float* d = spline.d;
    const float* x = spline.x;
    int n = spline.size;

    if (x_val < x[0] || x_val > x[n - 1]) {
        return linearExtrapolate(x, a, n, x_val);
    }

    int left = 0, right = n - 1;
    while (right - left > 1) {
        int mid = (left + right) / 2;
        if (x_val < x[mid]) {
            right = mid;
        }
        else {
            left = mid;
        }
    }

    float dx = x_val - x[left];
    return a[left] + b[left] * dx + c[left] * dx * dx + d[left] * dx * dx * dx;
}
