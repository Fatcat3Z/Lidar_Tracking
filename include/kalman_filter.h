//
// Created by FATCAT.STARK on 2021/12/25.
//

#ifndef PLACE_RECOGNIZATION_KALMAN_FILTER_H
#define PLACE_RECOGNIZATION_KALMAN_FILTER_H
#include <pcl/point_types.h>

typedef Eigen::Matrix<float, 6, 1> vec;
typedef Eigen::Matrix<float, 6, 6> matrix;

class KalmanFilter{
public:
    KalmanFilter(){
        vec initeye(6), initzero(6);
        initeye << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        initzero << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        _deltat = 0.1;
        _states = initzero;
        _observations = initzero;
        _covP = 3 * initeye.asDiagonal();
        _transformF << Eigen::Matrix3f::Identity() , _deltat * Eigen::Matrix3f::Identity(),
                Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Identity();
        _observA = initeye.asDiagonal();
        _pronoiseQ  = initeye.asDiagonal();
        _observR = initeye.asDiagonal();
        lastResult = initzero;
    };
    vec predict();
    vec correct(const vec& observations, const bool& flag);
    vec lastResult;            // 上一帧结果
private:
    float _deltat;              // 传感器频率
    vec _states;                // 状态向量
    vec _observations;          // 观测向量
    matrix _covP;               // 状态协方差矩阵
    matrix _transformF;         // 状态转移矩阵
    matrix _observA;            // 观测方程
    matrix _pronoiseQ;          // 过程噪声
    matrix _observR;            // 观测噪声


};
#endif //PLACE_RECOGNIZATION_KALMAN_FILTER_H

