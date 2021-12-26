//
// Created by FATCAT.STARK on 2021/12/25.
//

#ifndef PLACE_RECOGNIZATION_TRACK_H
#define PLACE_RECOGNIZATION_TRACK_H
#include "kalman_filter.h"
#include <utility>
#include <vector>
#define MAXNUM 1000

struct Track{
    Track(vec prediction, int trackid):_track_id(trackid), _skipped_frames(0), _prediction(std::move(prediction)){}
    int _track_id;
    int _skipped_frames;
    vec _prediction;
    KalmanFilter _kf;
    std::vector<vec> _trace;
    Eigen::Vector3f _dimentions;
};
class Tracker{
public:
    Tracker(float dist_thresh, int max_frame_to_skip, float max_trace_length, int trackid):_iou_thresh(dist_thresh),
                                                                                           _max_frames_to_skip(max_frame_to_skip),
                                                                                           _max_trace_length(max_trace_length),
                                                                                           _trackid(trackid),
                                                                                           frame(0){}
    float calculateIOU(const vec& prediction, const vec& detection);

    void update(const std::vector<vec>& detections);
    int  frame;
    std::vector<Track> tracks;
    int _trackid;
private:
    float _iou_thresh;
    int _max_frames_to_skip;
    float _max_trace_length;



};
#endif //PLACE_RECOGNIZATION_TRACK_H
