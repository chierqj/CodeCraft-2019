#ifndef CAR_H
#define CAR_H

#include <map>
#include <set>
#include <vector>

class Road;
enum CarStatus { WAIT, END };

class Car {
 public:
  Car();
  Car(int id, int st, int ed, int speed, int plan_time, int priority,
      int preset);
  void debug_params();
  void out_cross();

  // for judger
  void confirm_status(int pre_car_id);
  void clear();
  bool run_to_road();
  void move_to_next_road();

  // for drive
  void dijkstra(int label, int start);
  void son_fix(int start, int end, int LIMIT, std::vector<int> &roads,
               std::set<int> &circle_cross);
  bool fix(std::set<int> &circle_cross);

 public:
  // id,from,to,speed,planTime, priority, preset
  int id, st, ed, speed, plan_time, priority, preset;

  // for judger
  CarStatus status;
  int from = -1, to = -1;  // 所在当前道路的入口和出口cross_id
  int local_road_id = 0;   // 所在road_id
  int local_gird_row = -1;  // 所在第几个车道(下标从1开始，优先级递增)
  int local_gird_col = -1;  // 所在第几个列(下标从1开始，出路口最小)
  int SV = 0;               // 可行驶最大速度
  int next_road_id = 0;  // 下一条路的所在ans_roads的下标index
  bool if_end = false;   // 是否到达终点
  int on_road_time = 0;  // 上路时刻
  int to_end_time = 0;   // 到达终点时刻

  // for drive
  int begin_time = -1;          // 实际出发时间
  std::vector<int> ans_roads;   // 答案路径
  bool if_preset = false;       // 是否为预制车辆
  bool can_run_to_end = false;  // 是否可以寻找到终点的路径
  bool need_change = false;  // 是否需要改变路径,仅针对预制车辆！！！
  bool if_changed = false;  // 是否改变路径，仅针对预制车辆！！！

  bool fix_lock = false;  // 是否修复过
};

#endif