#ifndef CROSS_H
#define CROSS_H

#include <list>
#include <map>
#include <queue>
#include <vector>
#include "car.h"

class Car;
enum CarDriveDir { RIGHT, LEFT, STRIGHT };
class Cross {
 public:
  Cross();
  Cross(int id, std::vector<int> roads);
  void debug_params();

  // for drive
  int get_dir(int from, int to);
  bool match_traffic_rules(Car &car);
  void clear();
  void neighbor_road_info(int road_id, std::vector<std::vector<double>> &infos);

 public:
  // #(id, roadId, roadId, roadId, roadId)
  int id;
  std::vector<int> roads;  // 连接的road
  std::vector<int> dirs;  // 连接的道路的拓扑方向 0:右上1:右下2:左下3:左上
  std::vector<int> sorted_roads;  // 道路按照id从小到大排序
  std::list<int> in_init_cars;  // 初始当前路口要出发的车，begin_time 升序
  double layer = 0;  // 当前路口在整张地图的第几层，最中间为第一层
  double x = 0, y = 0;  // 当前cross的物理坐标
};

#endif