In the trajectory planning task of autonomous driving, we usually use center line of the road as a reference line to help planning. However, the curvature may be large in curve lanes. So I build this project learning from [karlkurzer/path_planner](https://github.com/karlkurzer/path_planner). There are Hybrid A* searching, smoothing, Rviz visualization in his project but I'm mainly  focus on smoothing. The main changes are the calculation of smooth cost and voronoi cost. And I use OpenCV to visualize the result.
 
I put an example to generate image maps, "generate_lane_curves". And another more important example to test the method, "path_smoother_example".
 
If you want a good result, some adjustment for parameters is needed.
 
A typical result is show below. Green curve is the original path without smoothing, actually it is the center line plus a random noise. Red curve is the smoothed path.

![result_image](https://github.com/linxigjs/path_smoother/blob/master/map_imgs/smoothed%20path_screenshot.png)
