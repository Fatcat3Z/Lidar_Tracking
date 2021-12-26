//
// Created by FATCAT.STARK on 2021/12/25.
//
#include "../include/track.h"


float Tracker::calculateIOU(const vec &prediction, const vec &detection) {
    float w = detection[3];
    float l = detection[4];
    float h = detection[5];

    float prediction_x1 = prediction[0] + w / 2;
    float prediction_y1 = prediction[1] + l / 2;
    float prediction_z1 = prediction[2] + h / 2;

    float prediction_x2 = prediction[0] - w / 2;
    float prediction_y2 = prediction[1] - l / 2;
    float prediction_z2 = prediction[2] - h / 2;

    float detection_x1 = detection[0] + w / 2;
    float detection_y1 = detection[1] + l / 2;
    float detection_z1 = detection[2] + h / 2;

    float detection_x2 = detection[0] - w / 2;
    float detection_y2 = detection[1] - l / 2;
    float detection_z2 = detection[2] - h / 2;

    float in_w = fmin(prediction_x1, detection_x1) - fmax(prediction_x2, detection_x2);
    float in_l = fmin(prediction_y1, detection_y1) - fmax(prediction_y2, detection_y2);
    float in_h = fmin(prediction_z1, detection_z1) - fmax(prediction_z2, detection_z2);
    float inter_area = in_w < 0 || in_l < 0 || in_h < 0 ? 0  : in_w * in_l * in_h;
    float union_area = 2 * w * l * h - inter_area;
    float iou = inter_area == 0 ? -1 : inter_area / union_area;

    return iou;
}

void Tracker::update(const std::vector<vec>& detections) {
    if(tracks.empty()){
        for(const auto& detect: detections){
            vec detection;
            detection << detect[0] , detect[1], detect[2], 0, 0, 0;
            Track track(detection, _trackid);
            _trackid++;
            tracks.push_back(track);
        }
    }
    /*------计算检测和预测的IOU--------*/
    int kalmannum = tracks.size();
    int detectnum = detections.size();
    float ioucost[kalmannum][detectnum];
    for(int i= 0; i < kalmannum; i++){
        for(int j= 0; j < detectnum; j++){
            ioucost[i][j] = -calculateIOU(tracks[i]._prediction, detections[j]);
        }
    }
    /*------用匈牙利算法对齐检测目标和卡尔曼预测目标--------*/
    std::vector<int> assignment(kalmannum, -1);

    std::vector<int> un_assigned_tracks;
    for(int i= 0; i< kalmannum; i++){
        if(assignment[i] != -1){
            if(-ioucost[i][assignment[i]] < _iou_thresh || ioucost[i][assignment[i]] == -1){   // 如果IOU小于阈值，则取消追踪
                assignment[i] = -1;
                un_assigned_tracks.push_back(i);
            }
        }else{
            tracks[i]._skipped_frames += 1;                    // 当前帧没有检测到，跳过
        }
    }
    /*---------如果长时间没有检测到目标，对其进行删除----------*/
    std::vector<int> del_tracks;
    for(int i= 0; i< tracks.size(); i++){
        if(tracks[i]._skipped_frames > _max_frames_to_skip)
            del_tracks.push_back(i);
    }
    if(!del_tracks.empty()){
        for(const int& id: del_tracks){
            if(id < tracks.size()){
                tracks.erase(tracks.begin() + id);
                assignment.erase(assignment.begin() + id);
            }
        }
    }

    /*-----------对于未对齐的检测，初始化跟踪-------------*/
    std::vector<int> un_assigned_detects;
    for(int i= 0; i< detections.size(); i++){
        if(!std::count(assignment.begin(), assignment.end(), i))
            un_assigned_detects.push_back(i);
    }
    if(!un_assigned_detects.empty()){
        for(int un_assigned_detect : un_assigned_detects){
            vec detectionnew;
            detectionnew << detections[un_assigned_detect][0],
                            detections[un_assigned_detect][1],
                            detections[un_assigned_detect][2], 0, 0, 0;
            Track track(detectionnew, _trackid);
            tracks.push_back(track);
        }
    }

    /*-------对成功分配的结果用卡尔曼滤波进行更新---------*/
    for(int i= 0; i< assignment.size(); i++){
        if(!frame)  tracks[i]._kf.predict();
        if(assignment[i] != -1){
            tracks[i]._skipped_frames = 0;
            vec updateinfo;
            updateinfo << detections[assignment[i]][0],
                          detections[assignment[i]][1],
                          detections[assignment[i]][2], 8, 0, 0;
            tracks[i]._prediction = tracks[i]._kf.correct(updateinfo, true);
        }else{
            tracks[i]._prediction = tracks[i]._kf.correct(tracks[i]._kf.lastResult, false);
        }

        tracks[i]._trace.push_back(tracks[i]._prediction);
        tracks[i]._kf.lastResult = tracks[i]._prediction;
        tracks[i]._kf.predict();
        tracks[i]._dimentions = Eigen::Vector3f(detections[assignment[i]][3],
                                                 detections[assignment[i]][4],
                                                 detections[assignment[i]][5]);
    }
}

