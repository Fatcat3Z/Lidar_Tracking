//
// Created by FATCAT.STARK on 2021/12/25.
//
#include "include/kalman_filter.h"
#include "include/track.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc,char *argv[]){
    /*-------设置参数--------*/
    std::string modelpath = argv[1];
    int skipframe = std::stoi(argv[2]);
    std::string datapath = argv[3];
    /*-------加载模型-------*/
    std::vector<std::string> ptsfiles;

    Tracker tracker(0.1, 5, 0, 0);

    std::vector<std::vector<float>> boxes;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(" cloud viewer"));
    viewer->setBackgroundColor(0, 0, 0);
//    viewer->setCameraPosition(-29.570503, -52.226951, 51.029257, 0.540905, 0.478015, 0.692043);
    for(auto ptsfile: ptsfiles){
        /*-------使用模型进行检测，获得检测结果-------*/

        std::vector<vec> detectinfos;
        /*------如果能够实现检测，用检测的值进行更新------*/
        if(!detectinfos.empty()){
            if(tracker.frame % skipframe == 0){
                tracker.update(detectinfos);
            }
        }
        for(const auto& track: tracker.tracks){
            if(track._prediction[0] == 0)   continue;
            Eigen::Vector3f position(track._prediction[0],
                                     track._prediction[1],
                                     track._prediction[2]);
            Eigen::Quaternionf quat(0, 0, 0, 1);
            viewer->addCube(position, quat, 1, 1, 1.73, "People " + std::to_string(track._track_id));
            if(track._trace.size() > 1){
                for(int i= 0; i< track._trace.size()-1; i++){
                    pcl::PointXYZ lastpos(track._trace[i][0],
                                          track._trace[i][1],
                                          track._trace[i][2]);
                    pcl::PointXYZ nowpos(track._trace[i+1][0],
                                         track._trace[i+1][1],
                                         track._trace[i+1][2]);
                    viewer->addLine<pcl::PointXYZ> (lastpos, nowpos);
                }
            }
        }
        tracker.frame += 1;
    }
    viewer->addCoordinateSystem(1.0);
    viewer->spin();

    return 0;
}
