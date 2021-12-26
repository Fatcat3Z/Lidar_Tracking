//
// Created by FATCAT.STARK on 2021/12/25.
//

#include "../include/kalman_filter.h"


vec KalmanFilter::predict() {
    _states = _transformF * _states;                                        // 状态预测
    _covP = _transformF * _covP * _transformF.transpose() + _pronoiseQ;     // 预测协方差更新
    lastResult = _states;
    return _states;
}

vec KalmanFilter::correct(const vec &observations, const bool &flag) {
    if(!flag)
        _observations = lastResult;
    else
        _observations = observations;
    vec vec_r = _observations - (_observA * _states);
    matrix matrixC = _observA * _covP * _observA.transpose() + _observR;
    matrix kalmanK = _covP * _observA.transpose() * matrixC.inverse();      // 卡尔曼增益

    _states = _states + kalmanK * vec_r;                                    // 状态向量更新
    _covP = _covP - kalmanK * matrixC * kalmanK.transpose();                // 协方差矩阵更新
    matrix matrixD = _observA * _covP * _observA.transpose();
    float factor = 0.7;
    _observR = factor * _observR + (1-factor) * (vec_r * vec_r.transpose() - matrixD);     // Sage-Husa更新噪声矩阵
    lastResult = _states;
    return _states;
}



