
#include "path_smoother/dynamic_voronoi.h"


DynamicVoronoi::DynamicVoronoi() {
  data = nullptr;
  gridMap = nullptr;
  edge_points_.clear();
  closest_edge_points_.clear();
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) {
      delete[] data[x];
    }
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) {
      delete[] gridMap[x];
    }
    delete[] gridMap;
  }
}

void DynamicVoronoi::buildVoronoiFromImage(const cv::Mat& map_img) {
  sizeX = map_img.cols;
  sizeY = map_img.rows;
  bool** binMap;
  binMap = new bool*[sizeX];
  for (int x = 0; x < sizeX; x++) {
    binMap[x] = new bool[sizeY];
  }
  for (int x = 0; x < sizeX; ++x) {   //这里的x,y符合opencv坐标系
    for (int y = 0; y < sizeY; ++y) {
      binMap[x][y] = static_cast<int>(map_img.at<uchar>(y, x)) < 1 ? true : false;
    }
  }

  initializeMap(sizeX, sizeY, binMap);
  update();
  prune();
//  visualize();
  CollectVoronoiEdgePoints();
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];

  if (initGridMap) {
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;

  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;   //跨过点(x,y)
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;  //嵌套循环，break 语句会停止执行最内层的循环
              }
            }
          } //在这层for循环内，因为isSurrounded只被此处设置一次，故break后值不会再变
          if (isSurrounded) {     //九宫格全是被占据的格子
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);     //九宫格至少有一个不被占据的格子
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}

void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) {
    return;
  }
  addList.push_back(Vec2i(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) {
    return;
  }
  removeList.push_back(Vec2i(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(std::vector<Vec2i> points) {
  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x();
    int y = lastObstacles[i].y();
    bool v = gridMap[x][y];
    if (v) {  //如果(x,y)被占用了，不处理，怀疑这里逻辑反了。要移除旧的障碍物，这里应该是(!v)表示没被占用就不处理，占用了就移除
      continue;
    }
    removeObstacle(x,y);  //如果没被占用，remove obstacle
  }  

  lastObstacles.clear();
  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x();
    int y = points[i].y();
    bool v = gridMap[x][y];
    if (v) {
      continue;
    }
    setObstacle(x,y);   //全设置为占用
    lastObstacles.push_back(points[i]);
  }  
}

void DynamicVoronoi::update(bool updateRealDist) {
  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    Vec2i p = open.pop();
    int x = p.x();
    int y = p.y();
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) {
      continue;
    }

    if (c.needsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) {
          continue;
        }
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) {
            continue;
          }
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) {
            continue;
          }
          dataCell nc = data[nx][ny];
          if (nc.obstX != invalidObstData && !nc.needsRaise) {  //nc有最近障碍物 且 不raise
            if(!isOccupied(nc.obstX, nc.obstY, data[nc.obstX][nc.obstY])) { //如果nc原来的最近障碍物消失了
              open.push(nc.sqdist, Vec2i(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;   //需要raise，并清理掉原来的最近障碍物信息
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {    //如果nc原来的最近障碍物还存在
              if(nc.queueing != fwQueued){  //这里没看懂？
                open.push(nc.sqdist, Vec2i(nx,ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    //nc有最近障碍物 且 nc原来的最近障碍物还存在
    else if (c.obstX != invalidObstData && isOccupied(c.obstX, c.obstY, data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;		
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) {    //距离相等
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX, nc.obstY, data[nc.obstX][nc.obstY])==false)
                overwrite = true;   //如果nc没有最近障碍物 或 nc原来的最近障碍物消失了
            }
            if (overwrite) {  //更新nc最近障碍物
              open.push(newSqDistance, Vec2i(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              checkVoro(x,y,nx,ny,c,nc);  //c对应(x,y)在data中存储的数据，nc对应(nx,ny)在data中存储的数据
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) {
    return data[x][y].dist;
  }
  else {
    return -INFINITY;
  }
}

bool DynamicVoronoi::isVoronoi( int x, int y ) {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    Vec2i p = addList[i];
    int x = p.x();
    int y = p.y();
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, Vec2i(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    Vec2i p = removeList[i];
    int x = p.x();
    int y = p.y();
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, Vec2i(x,y));
    if (updateRealDist) c.dist  = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}

void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {
  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { 
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) {   //点(x,y)到nc的最近障碍物的距离 < c到其最近障碍物的距离
        return;
      }

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) {  //点(nx,ny)到c的最近障碍物的距离 < nc到其最近障碍物的距离
        return;
      }

      //which cell is added to the Voronoi diagram?
      // (点(x,y)到nc的最近障碍物的距离 - c到其最近障碍物的距离) <= (点(nx,ny)到c的最近障碍物的距离 < nc到其最近障碍物的距离)
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(Vec2i(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(Vec2i(nx,ny));
        }
      }
    }
  }
}

void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) {
      continue;
    }
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) {
        continue;
      }
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) {
        continue;
      }
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(Vec2i(nx,ny));
      }
    }
  }
}

bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.ppm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  //fputc()执行3次，其实是依次对一个像素的RGB颜色赋值
  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (isVoronoi(x,y)) {  //画Voronoi边
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else if (data[x][y].sqdist==0) {  //填充障碍物
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {    //填充Voronoi区块内部
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}

void DynamicVoronoi::CollectVoronoiEdgePoints() {
  if(!edge_points_.empty()) {
    edge_points_.clear();
    closest_edge_points_.clear();
  }
  for(int y = sizeY-1; y >=0; y--){
    for(int x = 0; x<sizeX; x++){
      if(isVoronoi(x,y)){  //在Voronoi边上
        edge_points_.emplace_back(x, y);
        closest_edge_points_.emplace(ComputeIndex(Vec2i(x,y)), std::make_pair(Vec2i(x,y), 0));
      }
    }
  }
}

Vec2i DynamicVoronoi::GetClosestVoronoiEdgePoint(Vec2d xi, double& closest_dis) {
  Vec2i closest_pt;
  auto iter = closest_edge_points_.find(ComputeIndex(xi));
  if (closest_edge_points_.find(ComputeIndex(xi)) != closest_edge_points_.end()) {
    closest_pt = (*iter).second.first;
    closest_dis = (*iter).second.second;
    return closest_pt;
  }

  int closest_dis_sq = INT_MAX;
  for(const auto& pt : edge_points_) {
    int tmp_sq = pow((int)(xi.x() - pt.x()), 2) + pow((int)(xi.y() - pt.y()), 2);
    if(tmp_sq < closest_dis_sq) {
      closest_dis_sq = tmp_sq;
      closest_pt = pt;
    }
  }
  closest_dis = sqrt(static_cast<double>(closest_dis_sq));
  closest_edge_points_.emplace(ComputeIndex(xi), std::make_pair(closest_pt, closest_dis));
  return closest_pt;
}

Vec2i DynamicVoronoi::GetClosetObstacleCoor(const Vec2d& p) const {
  int x = data[(int)p.x()][(int)p.y()].obstX;
  int y = data[(int)p.x()][(int)p.y()].obstY;
  return Vec2i(x,y);
}

void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    Vec2i p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x();
    int y = p.y();

    if (data[x][y].voronoi==occupied) {
      continue;
    }
    if (data[x][y].voronoi==freeQueued) {
      continue;
    }

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, Vec2i(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, Vec2i(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, Vec2i(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, Vec2i(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }

  while(!open.empty()) {
    Vec2i p = open.pop();
    dataCell c = data[p.x()][p.y()];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x(),p.y());
    if (r==pruned) {
      c.voronoi = voronoiPrune;
    }
    else if (r==keep) {
      c.voronoi = voronoiKeep;
    }
    else { // r==retry
      c.voronoi = voronoiRetry;
      pruneQueue.push(p);
    }
    data[p.x()][p.y()] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        Vec2i p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x()][p.y()].sqdist, p);
      }
    }
  }
}

DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];
  int nx, ny;
  int dx, dy;
  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) {
            voroCountFour++;
          }
        }
        if (b && !(dx && dy) ) {
          count++;
        }
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;

  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}

std::string DynamicVoronoi::ComputeIndex(const Vec2i& pi) const {
  return std::to_string(pi.x()) + "_" + std::to_string(pi.y());
}

std::string DynamicVoronoi::ComputeIndex(const Vec2d& pd) const {
  return std::to_string(static_cast<int>(pd.x())) + "_" + std::to_string(static_cast<int>(pd.y()));
}