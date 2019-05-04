#ifndef ROAD_H
#define ROAD_H

#include <list>
#include <queue>
#include <vector>
#include "car.h"

class Car;

class Road {
 public:
  Road();
  Road(int id, int length, int speed, int channel, int from, int to,
       int is_duplex);
  void debug_params();

  // for judger
  void clear();
  void set_car_status();
  void remove_car(Car car);
  void add_car(Car car);
  void adjust_pipe(Car car);
  int last_use_car_id(int now_car_to, int &next_row);
  void run_car_in_init_list(int time_slice, bool is_priority, int lim_dir,
                            int &on_road_num);
  int get_high_level(int dir);

  // get solty
  double dijk_value(int car_id, int dir);
  void get_gird_info(int dir, std::vector<double> &infos);

 public:
  // #(id, length, speed, channel, from, to, isDuplex)
  int id, length, speed, channel, from, to, is_duplex;

  // for judger
  std::vector<std::queue<int> > girds_from;  // from -> to的道路
  std::vector<std::queue<int> > girds_to;    // to -> from的道路
  std::list<int> in_init_list[2];  // 车库中等待上路的车辆,begin_time升序

  double vis_cnt[2] = {0.0};      // 访问次数
  double locked_cost[2] = {0.0};  // 死锁增加的权重
};

#endif