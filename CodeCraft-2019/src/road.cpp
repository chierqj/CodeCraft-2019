#include "road.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "data.h"
#include "debug.h"

/************************Road initial and debug*************************/
Road::Road() {}
Road::Road(int id, int length, int speed, int channel, int from, int to,
           int is_duplex) {
  this->id = id;
  this->length = length;
  this->speed = speed;
  this->channel = channel;
  this->from = from;
  this->to = to;
  this->is_duplex = is_duplex;
}
void Road::debug_params() {
  DEBUG(id, length, speed, channel, from, to, is_duplex);
}
/*******************************Road 判题器部分***************************/
void Road::set_car_status() {
  for (auto pipe : this->girds_from) {
    int pre_car_id = -1;
    while (!pipe.empty()) {
      Car &car = CARS[ID_CAR[pipe.front()]];
      car.confirm_status(pre_car_id);
      pre_car_id = car.id;
      pipe.pop();
    }
  }
  if (this->is_duplex == 1) {
    for (auto pipe : this->girds_to) {
      int pre_car_id = -1;
      while (!pipe.empty()) {
        Car &car = CARS[ID_CAR[pipe.front()]];
        car.confirm_status(pre_car_id);
        pre_car_id = car.id;
        pipe.pop();
      }
    }
  }
}
int Road::last_use_car_id(int now_car_to, int &next_row) {
  if (now_car_to == this->from) {
    next_row = 0;
    for (auto &pipe : this->girds_from) {
      next_row++;
      if (pipe.empty()) {
        return 0;
      } else {
        Car &car = CARS[ID_CAR[pipe.back()]];
        if (car.local_gird_col < this->length || car.status == WAIT) {
          return car.id;
        }
      }
    }
    return (*(this->girds_from.rbegin())).back();
  } else {
    next_row = 0;
    for (auto &pipe : this->girds_to) {
      next_row++;
      if (pipe.empty()) {
        return 0;
      } else {
        Car &car = CARS[ID_CAR[pipe.back()]];
        if (car.local_gird_col < this->length || car.status == WAIT) {
          return car.id;
        }
      }
    }
    return (*(this->girds_to.rbegin())).back();
  }
}
void Road::adjust_pipe(Car car) {
  if (car.to == this->to) {
    int pre_car_id = -1;
    std::queue<int> foo = this->girds_from[car.local_gird_row - 1];
    while (!foo.empty()) {
      Car &car = CARS[ID_CAR[foo.front()]];
      if (car.status == WAIT) {
        car.confirm_status(pre_car_id);
      }
      pre_car_id = car.id;
      foo.pop();
    }
  } else {
    int pre_car_id = -1;
    std::queue<int> foo = this->girds_to[car.local_gird_row - 1];
    while (!foo.empty()) {
      Car &car = CARS[ID_CAR[foo.front()]];
      if (car.status == WAIT) {
        car.confirm_status(pre_car_id);
      }
      pre_car_id = car.id;
      foo.pop();
    }
  }
}
void Road::remove_car(Car car) {
  if (car.to == this->to) {
    this->girds_from[car.local_gird_row - 1].pop();
  } else {
    this->girds_to[car.local_gird_row - 1].pop();
  }
}
void Road::add_car(Car car) {
  if (car.to == this->to) {
    this->girds_from[car.local_gird_row - 1].push(car.id);
  } else {
    this->girds_to[car.local_gird_row - 1].push(car.id);
  }
}
void Road::run_car_in_init_list(int time_slice, bool is_priority, int lim_dir,
                                int &on_road_num) {
  std::vector<int> dirs;
  if (lim_dir == 2) {
    dirs = {0, 1};
  } else if (lim_dir == 0) {
    dirs = {0};
  } else {
    dirs = {1};
  }

  // 上路加权vis_cnt
  auto add_on_road_cnt = [&](int car_id) {
    Car &car = CARS[ID_CAR[car_id]];
    int pre_cross_id = car.st;
    for (auto road_id : car.ans_roads) {
      Road &road = ROADS[ID_ROAD[road_id]];
      int dir = (road.from == pre_cross_id ? 0 : 1);
      pre_cross_id = (dir == 0 ? road.to : road.from);
      if (car.if_preset) {
        road.vis_cnt[dir] += PRESET_CAR_ADD_VIS_CNT;
      } else {
        road.vis_cnt[dir] += NORMAL_CAR_ADD_VIS_CNT;
      }
    }
  };

  for (auto i : dirs) {
    std::vector<int> foo;
    for (auto car_id : this->in_init_list[i]) {
      Car &car = CARS[ID_CAR[car_id]];
      if (car.begin_time > time_slice) {
        break;
      }
      if (is_priority) {
        if (car.priority > 0) {
          foo.emplace_back(car_id);
        }
      } else {
        foo.emplace_back(car_id);
      }
    }
    sort(foo.begin(), foo.end(), [](const int &x, const int &y) {
      Car &car1 = CARS[ID_CAR[x]];
      Car &car2 = CARS[ID_CAR[y]];
      if (car1.priority == car2.priority) {
        if (car1.begin_time == car2.begin_time) {
          return car1.id < car2.id;
        }
        return car1.begin_time < car2.begin_time;
      }
      return car1.priority > car2.priority;
    });
    for (auto car_id : foo) {
      Car &car = CARS[ID_CAR[car_id]];
      if (car.run_to_road()) {
        car.on_road_time = time_slice;
        on_road_num++;
        this->in_init_list[i].remove(car_id);
        // 上路车辆对道路的影响
        add_on_road_cnt(car.id);
      }
    }
  }
}
int Road::get_high_level(int dir) {
  std::vector<int> foo;
  if (dir == 0) {
    for (auto &pipe : this->girds_from) {
      if (!pipe.empty()) {
        Car &car = CARS[ID_CAR[pipe.front()]];
        if (car.status == WAIT) {
          foo.emplace_back(car.id);
        }
        continue;
      }
    }
  } else {
    for (auto &pipe : this->girds_to) {
      if (!pipe.empty()) {
        Car &car = CARS[ID_CAR[pipe.front()]];
        if (car.status == WAIT) {
          foo.emplace_back(car.id);
        }
        continue;
      }
    }
  }
  sort(foo.begin(), foo.end(), [](const int &x, const int &y) {
    Car &car1 = CARS[ID_CAR[x]];
    Car &car2 = CARS[ID_CAR[y]];
    if (car1.priority == car2.priority) {
      if (car1.local_gird_col == car2.local_gird_col) {
        return car1.local_gird_row < car2.local_gird_row;
      }
      return car1.local_gird_col < car2.local_gird_col;
    }
    return car1.priority > car2.priority;
  });
  if (foo.empty()) {
    return -1;
  }
  return foo[0];
}
void Road::clear() {
  for (auto &pipe : this->girds_from) {
    while (!pipe.empty()) {
      pipe.pop();
    }
  }
  for (auto &pipe : this->girds_to) {
    while (!pipe.empty()) {
      pipe.pop();
    }
  }
  for (int i = 0; i < 2; i++) {
    this->in_init_list[i].clear();
  }
}
/*************************Road 获取道路信息部分**************************/
void Road::get_gird_info(int dir, std::vector<double> &infos) {
  double on_road_car_num = 0, ave_speed = 0;
  double sum = 0.0;
  if (dir == 0) {
    for (auto pipe : this->girds_from) {
      double min_sp = 1e18, p = 0.0;
      while (!pipe.empty()) {
        Car &car = CARS[ID_CAR[pipe.front()]];
        pipe.pop();
        on_road_car_num += 1.0;
        min_sp = min(min_sp, min(car.speed * 1.0, this->speed * 1.0));
        p = max(p, car.local_gird_col * 1.0);
      }
      ave_speed += min_sp;
      sum += p;
    }
  } else {
    for (auto pipe : this->girds_to) {
      double min_sp = 1e18, p = 0.0;
      while (!pipe.empty()) {
        Car &car = CARS[ID_CAR[pipe.front()]];
        pipe.pop();
        on_road_car_num += 1.0;
        min_sp = min(min_sp, min(car.speed * 1.0, this->speed * 1.0));
        p = max(p, car.local_gird_col * 1.0);
      }
      ave_speed += min_sp;
      sum += p;
    }
  }
  if (on_road_car_num > 0.0) {
    ave_speed /= on_road_car_num;
  }
  double rate = sum / (this->length * this->channel);
  infos = {this->vis_cnt[dir], ave_speed, rate};
}
double Road::dijk_value(int car_id, int dir) {
  //  info = {this->vis_cnt, ave_speed, rate};
  int cross_id = (dir == 0 ? this->to : this->from);
  Cross &cross = CROSSES[ID_CROSS[cross_id]];  // cross为要选择的这条路的出路口
  Car &car = CARS[ID_CAR[car_id]];

  std::vector<double> infos;
  std::vector<std::vector<double> > neighbor_infos;
  this->get_gird_info(dir, infos);
  double now_rate = infos[2];
  double result = (now_rate) * (100 / (double)this->channel);
  return result;
}