#ifndef JUDGER_H
#define JUDGER_H

#include <map>
#include <queue>
#include <set>
#include <vector>
#include "car.h"

class Judger {
 public:
  Judger();
  void entry();
  void clear();
  void prepare_clear();
  void prepare_judge();

  // judger
  bool conflict(Car &car);
  void drive_just_current_road();
  void drive_car_init_list(int time_slice, bool is_priority);
  void drive_car_in_wait_state();
  void drive_to_wait_queue();
  bool is_finish();

  // out_info
  void out_debug();
  void out_result();
  void out_locked();

  // dinamic create roads
  void find_circle_roads(std::set<int> &circle_roads);
  void create_roads();
  void save();
  void back();

 public:
  // params
  bool LOCKED = false;        // 是否死锁
  int running_car_num = 0;    // 未到终点的所有车
  int tol_car_num = 0;        // 所有车辆总数
  int now_time = 0;           // 现在时间片
  int all_car_time = 0;       // 所有车的调度时间
  int all_car_tol_time = 0;   // 所有车的累加调度时间
  int priority_num = 0;       // 优先车的数目
  int running_pri_num = 0;    // 未到终点的优先车辆数目
  int priority_time = 0;      // 优先车的调度时间
  int priority_tol_time = 0;  // 优先车的累加调度时间
  int ans_time = 0;           // 加权之后的调度时间
  int ans_tol_time = 0;       // 加权之后的累加调度时间

  std::set<int> locked_car_queue;  // 发生死锁时处于WAIT状态的车辆

  /*
   *  计算系数相关变量
   */
  int first_pri_time = -1;
  double all_max_speed, all_min_speed;
  double priority_max_speed, priority_min_speed;
  double all_last_time, all_first_time;
  double priority_last_time, priority_first_time;
  double all_start_lo, all_end_lo;
  double priority_start_lo, priority_end_lo;
  double fac_a, fac_b;

  int on_road_num = 0;  // 在道路网格上的车辆数目
  int go_on_road = 0;   // 当前时刻入库的车辆数目
  int tol_on_road = 0;  // 从cross入库的车辆总数目

  int tol_length = 0;          // 道路长度之和
  int tol_length_duplex = 0;   // 双车道道路长度之和
  int tol_channel = 0;         // 道路的路宽之和
  int tol_channel_duplex = 0;  // 双车道道路的路宽之和
};

#endif