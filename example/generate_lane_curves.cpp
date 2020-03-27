//
// Created by goujs on 20-3-18.
//

#include <opencv2/opencv.hpp>
#include <random>
using namespace cv;
using namespace std;

int generate_arc2() {
  int height = 80, width = 80;
  double r = 60.0, cen_x = width * 0.5, cen_y = -r + height*0.5;
  double lane_width = 20;
  cv::Mat map_img = Mat::zeros(Size(height, width), CV_8UC1);
  circle(map_img, Point(cen_x, cen_y), r, Scalar(255), lane_width);
  imshow("binary map", map_img);
  waitKey();
  imwrite("middle_lane.png", map_img);
  for(int x=0; x<width; x+=2) {
    double y = sqrt(pow(r,2) - pow(x-cen_x,2)) + cen_y;
    cout << "(" << x << ", " << y << ")" << endl;
  }
  return 0;
}

void generate_arc() {
  int height = 200, width = 200;
  double r = 120.0, cen_x = width * 0.5, cen_y = -r + height*0.6;
  double lane_width = 40;
  cv::Mat map_img = Mat::zeros(Size(height, width), CV_8UC1);
  circle(map_img, Point(cen_x, cen_y), r, Scalar(255), lane_width);
  imshow("binary map", map_img);
  waitKey();
  imwrite("circle_lane.png", map_img);
  std::vector<double> xs, ys;
  int step = 4;
  default_random_engine eng;
  uniform_real_distribution<double> randd(-1.5, 1.5);
  for(int x=0; x<width; x+=step) {
    double y = sqrt(pow(r,2) - pow(x-cen_x,2)) + cen_y;
    cout << "(" << x << ", " << y << ")" << endl;
    xs.emplace_back(x + randd(eng));
    ys.emplace_back(y + randd(eng));
  }
  for(auto e : xs) {
    cout << e << ", ";
  }
  std::cout << std::endl;
  for(auto e : ys) {
    cout << e << ", ";
  }
  std::cout << std::endl;
}

void generate_sin() {
  int height = 500, width = 800, step = 10;
  double lane_width = 40;
  cv::Mat map_img = Mat::zeros(Size(width, height), CV_8UC1);
  double x_unit = 4*M_PI/width;
  std::vector<double> xs, ys;
  default_random_engine eng;
  uniform_real_distribution<double> randd(-3, 3);
  for(int x=0; x<width; x+=step) {
    double y = (sin(x_unit * x + M_PI_2) + 3) * 80;
    circle(map_img, Point(x,y), lane_width, Scalar(255), -1);
    xs.emplace_back(x + randd(eng));
    ys.emplace_back(y + randd(eng));
  }

  imshow("binary map", map_img);
  waitKey();
  imwrite("sin_lane.png", map_img);
  for(auto e : xs) {
    cout << e << ", ";
  }
  std::cout << std::endl;
  for(auto e : ys) {
    cout << e << ", ";
  }
  std::cout << std::endl;
}

int main() {
//  generate_arc();
  generate_sin();
  return 0;
}