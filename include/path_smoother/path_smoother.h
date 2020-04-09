#ifndef PATH_SMOOTHER_PATH_SMOOTHER_H
#define PATH_SMOOTHER_PATH_SMOOTHER_H

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

#include "basic_define/pose2d.h"
#include "basic_define/vec2d.h"
#include "dynamic_voronoi.h"
#include "constants.h"


class PathSmoother {
public:
    PathSmoother() {};

    PathSmoother(cv::Mat map_img, const std::vector<Vec2d>& cv_path);

    void smoothPath();

    std::vector<Pose2d> getOriginalPath() {return original_path_;}

    std::vector<Pose2d> getSmoothedPath() {return smoothed_path_;}

private:

    Vec2d obstacleTerm(Vec2d xi);//障碍物项，用于约束路径远离障碍物

    Vec2d curvatureTerm(Vec2d xi0, Vec2d xi1, Vec2d xi2);//曲率项，用于保证可转弯性及通行性

    //平滑项，用于将节点等距分布并尽量保持同一个方向
    Vec2d smoothnessTerm(Vec2d xim, Vec2d xi, Vec2d xip);

    Vec2d voronoiTerm(Vec2d xi);

    bool isOnGrid(Vec2d vec);

    /// maximum possible curvature of the non-holonomic vehicle
    float kappaMax_ = 1.f / (Constants::min_turn_radius * 1.1);
    /// maximum distance to obstacles that is penalized
    float obsDMax_ = 5*Constants::minRoadWidth;
    /// maximum distance for obstacles to influence the voronoi field
    float vorObsDMax_ = 5*Constants::minRoadWidth;
    //obsDMax的作用更显著，应该 vorObsDMax >= obsDMax。首先要不碰撞障碍物，在不碰的前提下，调整离障碍物的距离

    /// falloff rate for the voronoi field
    float alpha_ = 0.1;
    float wObstacle_ = 0.2;
    float wVoronoi_ = 0.2;
    float wCurvature_ = 0.2;
    float wSmoothness_ = 0.2;

    DynamicVoronoi voronoi_;
    cv::Mat map_img_;
    int map_width_;
    int map_height_;
    std::vector<Pose2d> original_path_;
    std::vector<Pose2d> smoothed_path_;
};

#endif // PATH_SMOOTHER_PATH_SMOOTHER_H
