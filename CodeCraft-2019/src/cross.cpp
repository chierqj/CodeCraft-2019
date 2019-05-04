#include "cross.h"
#include <algorithm>
#include <iostream>
#include <list>
#include <vector>
#include "car.h"
#include "data.h"
#include "debug.h"
#include "judger.h"
#include "road.h"

/************************Cross initial and debug***********************/
Cross::Cross() {}
Cross::Cross(int id, std::vector<int> roads) {
  this->id = id;
  this->roads = roads;
}
void Cross::debug_params() { DEBUG(this->id, this->roads); }
/***************************Cross 判题器部分****************************/
void Cross::clear() {}
int Cross::get_dir(int from, int to) {
  int from_index = 0, to_index = 0;
  for (int i = 0; i < (int)this->roads.size(); i++) {
    if (this->roads[i] == from) {
      from_index = i;
    }
    if (this->roads[i] == to) {
      to_index = i;
    }
  }
  // transform distance to diraction
  int dis = 0;
  if (to_index > from_index) {
    dis = to_index - from_index;
  } else {
    dis = 4 - from_index + to_index;
  }
  dis++;
  if (dis > 3) {
    dis -= 3;
  }
  dis--;
  return dis;
}
bool Cross::match_traffic_rules(Car &car) {
  int now_car_from = car.local_road_id;
  for (auto road_id : this->roads) {
    if (road_id != -1 && road_id != car.local_road_id) {
      Road &road = ROADS[ID_ROAD[road_id]];
      int dir = (road.to == this->id ? 0 : 1);
      int car_id = road.get_high_level(dir);
      if (car_id != -1) {
        Car foo_car = CARS[ID_CAR[car_id]];
        if (foo_car.priority < car.priority) {
          continue;
        }
        int foo_car_from = foo_car.local_road_id;
        int foo_car_to = foo_car.ans_roads[foo_car.next_road_id];
        if (car.to == car.ed) {
          if (foo_car.priority > car.priority && foo_car.to != foo_car.ed) {
            int tmp_dir = this->get_dir(now_car_from, foo_car_from);
            int foo_car_dir = this->get_dir(foo_car_from, foo_car_to);
            if (tmp_dir == foo_car_dir && tmp_dir != STRIGHT) {
              return false;
            }
          }
        } else {
          int now_car_to = car.ans_roads[car.next_road_id];
          int now_car_dir = this->get_dir(now_car_from, now_car_to);
          if (foo_car.to == foo_car.ed) {
            int tmp_dir = this->get_dir(now_car_from, foo_car_from);
            if (now_car_dir == LEFT && tmp_dir == RIGHT) {
              return false;
            }
            if (now_car_dir == RIGHT && tmp_dir == LEFT) {
              return false;
            }
          } else {
            int foo_car_dir = this->get_dir(foo_car_from, foo_car_to);
            if (foo_car_to == now_car_to) {
              if (foo_car.priority > car.priority ||
                  foo_car_dir > now_car_dir) {
                return false;
              }
            }
          }
        }
      }
    }
  }
  return true;
}
/***************************Cross 获取信息部分****************************/
void Cross::neighbor_road_info(int road_id,
                               std::vector<std::vector<double>> &infos) {
  for (auto foo : this->roads) {
    if (foo != road_id && foo != -1) {
      Road &road = ROADS[ID_ROAD[foo]];
      int dir = (road.from == this->id ? 0 : 1);
      if (road.is_duplex == 0 && dir != 0) {
        continue;
      }
      std::vector<double> info;
      road.get_gird_info(dir, info);
      infos.emplace_back(info);
    }
  }
}