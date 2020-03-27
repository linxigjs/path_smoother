
#include "path_smoother/path_smoother.h"
#include "basic_define/math_utils.h"


PathSmoother::PathSmoother(const cv::Mat& map_img, const std::vector<Vec2d>& cv_path) {
  map_height_ = map_img.rows;
  map_width_ = map_img.cols;
  if(map_img.channels() != 1) {
    map_img_ = map_img.clone();
    cv::cvtColor(map_img, map_img, CV_BGR2GRAY);
  } else {
    cv::cvtColor(map_img, map_img_, CV_GRAY2BGR);
  }
  voronoi_.buildVoronoiFromImage(map_img);

  //transform path from cv coordinate system to OccupancyGrid coordinate system?
  for(const auto& pt : cv_path) {
    original_path_.emplace_back(pt.x(), pt.y(), 0);
    cv::circle(map_img_, cv::Point(pt.x(), pt.y()), 1, cv::Scalar(0,255,0), -1);
  }
//  cv::imshow("original path", map_img_);
//  cv::waitKey();
}

void PathSmoother::smoothPath() {
  int iterations = 0;
  smoothed_path_ = original_path_;
  float totalWeight = wSmoothness_ + wCurvature_ + wVoronoi_ + wObstacle_;

  //Todo:make sure the cycle end condition
  while (iterations < Constants::max_iterations) {
    for (int i = 1; i < smoothed_path_.size() - 1; ++i) {
      //后面1个点，当前点，前面1个点
      Vec2d xim1(smoothed_path_[i - 1].x(), smoothed_path_[i - 1].y());
      Vec2d xi(smoothed_path_[i].x(), smoothed_path_[i].y());
      Vec2d xip1(smoothed_path_[i + 1].x(), smoothed_path_[i + 1].y());
      Vec2d correction;

      correction = correction - obstacleTerm(xi);
      if (!isOnGrid(xi + correction)) { continue; }

      correction = correction - smoothnessTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction)) { continue; }

      correction = correction - curvatureTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction)) { continue; }

      correction = correction - voronoiTerm(xi);
      if (!isOnGrid(xi + correction)) { continue; }

//      Vec2d delta_change = alpha_ * correction/totalWeight;
//      std::cout <<"iterations=" << iterations <<", point index=" << i << ", delta_change=(" << delta_change.x() << ", " << delta_change.y() << ")" << std::endl;

      xi = xi + alpha_ * correction/totalWeight;
      smoothed_path_[i].set_x(xi.x());
      smoothed_path_[i].set_y(xi.y());
      Vec2d Dxi = xi - xim1;
      smoothed_path_[i - 1].set_theta(std::atan2(Dxi.y(), Dxi.x()));
    }
    iterations++;
  }

  for(const auto& po : smoothed_path_) {
    cv::circle(map_img_, cv::Point(po.x(), po.y()), 1, cv::Scalar(0,0,255), -1);
  }
  cv::imshow("smoothed path", map_img_);
  cv::waitKey();
}

//返回离最近障碍的梯度方向
Vec2d PathSmoother::obstacleTerm(Vec2d xi) {
  Vec2d gradient;
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi_.getDistance(xi.x(), xi.y());
  // the vector determining where the obstacle is
  int x = (int)xi.x();
  int y = (int)xi.y();
  // if the node is within the map
  if (x < map_width_ && x >= 0 && y < map_height_ && y >= 0) {
    //从当前点xi到最近障碍点的向量
    Vec2d obsVct(xi.x() - voronoi_.GetClosetObstacleCoor(xi).x(),
                 xi.y() - voronoi_.GetClosetObstacleCoor(xi).y());
    //obsDst本应该等于obsVct向量的模，但是相差较大（超过1m），而且不一定谁更大。
//    std::cout << "(==) dis to closest obs = " << obsDst << ", Vector Mod = " << obsVct.length() << std::endl;
    // the closest obstacle is closer than desired correct the path for that
    // obsDMax = 2m
    if (obsDst < obsDMax_ && obsDst > 1e-6) {
      gradient = wObstacle_ * 2 * (obsDst - obsDMax_) * obsVct / obsDst;
      return gradient;
    }
  }
  return gradient;
}

Vec2d PathSmoother::voronoiTerm(Vec2d xi) {
  Vec2d gradient;
  float obsDst = voronoi_.getDistance(xi.x(), xi.y());
  Vec2d obsVct(xi.x() - voronoi_.GetClosetObstacleCoor(xi).x(),
               xi.y() - voronoi_.GetClosetObstacleCoor(xi).y());

  double edgDst = 0.0;
  Vec2i closest_edge_pt = voronoi_.GetClosestVoronoiEdgePoint(xi, edgDst);
  Vec2d edgVct(xi.x() - closest_edge_pt.x(), xi.y() - closest_edge_pt.y());

  if (obsDst < vorObsDMax_ && obsDst > 1e-6) {
    if (edgDst > 0) {
      Vec2d PobsDst_Pxi = obsVct / obsDst;
      Vec2d PedgDst_Pxi = edgVct / edgDst;
//      float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst - vorObsDMax, 2) /
//                              (std::pow(vorObsDMax, 2) * (obsDst + alpha) * std::pow(edgDst + obsDst, 2));
      float PvorPtn_PedgDst = (alpha_ / alpha_ + obsDst) *
                              (pow(obsDst - vorObsDMax_, 2) / pow(vorObsDMax_, 2)) * (obsDst / pow(obsDst + edgDst, 2));

//      float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) * ((edgDst + 2 * vorObsDMax + alpha)
//                                                                         * obsDst + (vorObsDMax + 2 * alpha) * edgDst + alpha * vorObsDMax))
//                              / (std::pow(vorObsDMax, 2) * std::pow(obsDst + alpha, 2) * std::pow(obsDst + edgDst, 2));
      float PvorPtn_PobsDst = (alpha_ / (alpha_ + obsDst)) *
                              (edgDst / (edgDst + obsDst)) * ((obsDst - vorObsDMax_) / pow(vorObsDMax_, 2))
                              * (-(obsDst - vorObsDMax_) / (alpha_ + obsDst) - (obsDst - vorObsDMax_) / (obsDst + edgDst) + 2);
      gradient = wVoronoi_ * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi) * 100;
//      std::cout << "Smoother::voronoiTerm() 1, point(" << xi.x() << ", " << xi.y() << "), obsDst=" << obsDst
//                << ", edgDst=" << edgDst << ", gradient=["
//                << gradient.x() << ", " << gradient.y() << "]" << std::endl;
      return gradient;
    }
//    std::cout << "Smoother::voronoiTerm() 2, point(" << xi.x() << ", " << xi.y() << "), obsDst="
//              << obsDst << ", edgDst=" << edgDst << std::endl;
    return gradient;
  }
//  std::cout << "Smoother::voronoiTerm() 3, obsDst=" << obsDst << std::endl;
  return gradient;
}

//返回梯度方向
Vec2d PathSmoother::curvatureTerm(Vec2d xim1, Vec2d xi, Vec2d xip1) {
  Vec2d gradient;
  // the vectors between the nodes
  Vec2d Dxi = xi - xim1;
  Vec2d Dxip1 = xip1 - xi;
  // orthogonal complements vector
  Vec2d p1, p2;

  float absDxi = Dxi.Length();
  float absDxip1 = Dxip1.Length();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0) {
    float Dphi = std::acos(Clamp<float>(Dxi.InnerProd(Dxip1) / (absDxi * absDxip1), -1, 1));
    float kappa = Dphi / absDxi;

    if (kappa <= kappaMax_) {
      Vec2d zeros;
//      std::cout << "curvatureTerm is 0 because kappa(" << kappa << ") < kappamax(" << kappaMax << ")" << std::endl;
      return zeros;
    } else {
      //代入原文公式(2)与(3)之间的公式. 参考：
      // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for 
      //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
      float absDxi1Inv = 1 / absDxi;
      float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
      float u = -absDxi1Inv * PDphi_PcosDphi;
      p1 = xi.ort(-xip1) / (absDxi * absDxip1);//公式(4)
      p2 = -xip1.ort(xi) / (absDxi * absDxip1);
      float s = Dphi / (absDxi * absDxi);
      Vec2d ones(1, 1);
      Vec2d ki = u * (-p1 - p2) - (s * ones);
      Vec2d kim1 = u * p2 - (s * ones);
      Vec2d kip1 = u * p1;
      gradient = wCurvature_ * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

      if (std::isnan(gradient.x()) || std::isnan(gradient.y())) {
//        std::cout << "nan values in curvature term" << std::endl;
        Vec2d zeros;
//        std::cout << "curvatureTerm is 0 because gradient is non" << std::endl;
        return zeros;
      }
      else {
//        std::cout << "curvatureTerm is (" << gradient.x() << ", " << gradient.y() << ")" << std::endl;
        return gradient;
      }
    }
  } else {
    std::cout << "abs values not larger than 0" << std::endl;
    std::cout << absDxi << absDxip1 << std::endl;
    Vec2d zeros;
    std::cout << "curvatureTerm is 0 because abs values not larger than 0" << std::endl;
    return zeros;
  }
}

Vec2d PathSmoother::smoothnessTerm(Vec2d xim, Vec2d xi, Vec2d xip) {
  // 下面是我按照《Practical search techniques in path planning for autonomous driving》
  // 文章中的公式手动求导后，改动的代码，效果很好
  return wSmoothness_ * (-4) * (xip - 2*xi + xim);
}

bool PathSmoother::isOnGrid(Vec2d vec) {
  if (vec.x() >= 0 && vec.x() < map_width_ &&
      vec.y() >= 0 && vec.y() < map_height_) {
    return true;
  }
  return false;
}
