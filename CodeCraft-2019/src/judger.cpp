#include "judger.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include "car.h"
#include "cross.h"
#include "data.h"
#include "debug.h"
#include "helper.h"
#include "road.h"

Judger::Judger() {
  for_each(ROADS.begin(), ROADS.end(), [](Road &road) {
    road.girds_from.resize(road.channel);
    if (road.is_duplex == 1) {
      road.girds_to.resize(road.channel);
    }
  });
}
/*****************************Judge 预处理部分******************************/
void Judger::clear() {
  this->LOCKED = false;
  this->running_car_num = 0;
  this->tol_car_num = 0;
  this->now_time = 0;
  this->all_car_time = 0;
  this->all_car_tol_time = 0;
  this->priority_num = 0;
  this->running_pri_num = 0;
  this->priority_time = 0;
  this->priority_tol_time = 0;
  this->ans_time = 0;
  this->ans_tol_time = 0;
  this->first_pri_time = -1;
  this->locked_car_queue.clear();
}
void Judger::prepare_clear() {
  this->clear();
  sort(CARS.begin(), CARS.end(), [](const Car &car1, const Car &car2) {
    if (car1.begin_time == car2.begin_time) {
      return car1.id < car2.id;
    }
    return car1.begin_time < car2.begin_time;
  });
  int index = 0;
  for (auto &car : CARS) {
    car.clear();
    ID_CAR[car.id] = index++;
  }

  sort(CROSSES.begin(), CROSSES.end(),
       [](const Cross &cross1, const Cross &cross2) {
         return cross1.id < cross2.id;
       });
  index = 0;
  for (auto &cross : CROSSES) {
    cross.clear();
    cross.sorted_roads = cross.roads;
    sort(cross.sorted_roads.begin(), cross.sorted_roads.end());
    ID_CROSS[cross.id] = index++;
  }

  sort(ROADS.begin(), ROADS.end(), [](const Road &road1, const Road &road2) {
    return road1.id < road2.id;
  });
  index = 0;
  for (auto &road : ROADS) {
    road.clear();
    ID_ROAD[road.id] = index++;
  }
}
void Judger::prepare_judge() {
  for (auto &road : ROADS) {
    this->tol_length += road.length;
    this->tol_channel += road.channel;
    if (road.is_duplex == 1) {
      this->tol_length_duplex += road.length;
      this->tol_channel_duplex += road.channel;
    }
  }

  this->all_max_speed = 0.0;
  this->all_min_speed = 1e8 * 1.0;
  this->priority_max_speed = 0.0;
  this->priority_min_speed = 1e8 * 1.0;

  this->all_last_time = 0.0;
  this->all_first_time = 1e8 * 1.0;
  this->priority_last_time = 0.0;
  this->priority_first_time = 1e8 * 1.0;

  this->all_start_lo = 0;
  this->all_end_lo = 0;
  this->priority_start_lo = 0;
  this->priority_end_lo = 0;

  // all_st, all_ed, p_st, p_ed
  this->priority_num = 0;
  std::set<int> st[4];
  for (auto &car : CARS) {
    this->running_car_num++;
    this->tol_car_num++;
    if (car.priority > 0) {
      if (this->first_pri_time == -1) {
        this->first_pri_time = car.plan_time;
      }

      this->priority_num++;
      this->running_pri_num++;
      this->all_max_speed = max(this->all_max_speed, car.speed * 1.0);
      this->all_min_speed = min(this->all_min_speed, car.speed * 1.0);
      this->priority_max_speed = max(this->priority_max_speed, car.speed * 1.0);
      this->priority_min_speed = min(this->priority_min_speed, car.speed * 1.0);
      this->all_last_time = max(this->all_last_time, car.plan_time * 1.0);
      this->all_first_time = min(this->all_first_time, car.plan_time * 1.0);
      this->priority_last_time =
          max(this->priority_last_time, car.plan_time * 1.0);
      this->priority_first_time =
          min(this->priority_first_time, car.plan_time * 1.0);
      st[0].insert(car.st);
      st[1].insert(car.ed);
      st[2].insert(car.st);
      st[3].insert(car.ed);
    } else {
      this->all_max_speed = max(this->all_max_speed, car.speed * 1.0);
      this->all_min_speed = min(this->all_min_speed, car.speed * 1.0);
      this->all_last_time = max(this->all_last_time, car.plan_time * 1.0);
      this->all_first_time = min(this->all_first_time, car.plan_time * 1.0);
      st[0].insert(car.st);
      st[1].insert(car.ed);
    }
  }

  this->all_start_lo = st[0].size() * 1.0;
  this->all_end_lo = st[1].size() * 1.0;
  this->priority_start_lo = st[2].size() * 1.0;
  this->priority_end_lo = st[3].size() * 1.0;

  double x1 =
      (double)this->tol_car_num * 1.0 / (double)this->priority_num * 1.0;
  double x2 = (double)this->all_max_speed / (double)this->all_min_speed;
  double x3 =
      (double)this->priority_max_speed / (double)this->priority_min_speed;
  double x4 = (double)this->all_last_time / (double)this->all_first_time;
  double x5 =
      (double)this->priority_last_time / (double)this->priority_first_time;
  double x6 =
      (double)this->all_start_lo * 1.0 / (double)this->priority_start_lo * 1.0;
  double x7 =
      (double)this->all_end_lo * 1.0 / (double)this->priority_end_lo * 1.0;

  x1 = floor(x1 * 100000.000f + 0.5) / 100000.000f;
  x2 = floor(x2 * 100000.000f + 0.5) / 100000.000f;
  x3 = floor(x3 * 100000.000f + 0.5) / 100000.000f;
  x4 = floor(x4 * 100000.000f + 0.5) / 100000.000f;
  x5 = floor(x5 * 100000.000f + 0.5) / 100000.000f;
  x6 = floor(x6 * 100000.000f + 0.5) / 100000.000f;
  x7 = floor(x7 * 100000.000f + 0.5) / 100000.000f;

  double x8 = x2 / x3;
  double x9 = x4 / x5;
  x8 = floor(x8 * 100000.000f + 0.5) / 100000.000f;
  x9 = floor(x9 * 100000.000f + 0.5) / 100000.000f;

  this->fac_a = 0.05 * x1 + 0.2375 * (x8 + x9 + x6 + x7);
  this->fac_b = 0.8 * x1 + 0.05 * (x8 + x9 + x6 + x7);
}
/******************************Judge 主体部分*******************************/
bool Judger::conflict(Car &car) {
  Cross &cross = CROSSES[ID_CROSS[car.to]];
  if (cross.match_traffic_rules(car)) {
    return false;
  }
  return true;
}
void Judger::drive_just_current_road() {
  for (auto &road : ROADS) {
    road.set_car_status();
  }
}
void Judger::drive_car_init_list(int time_slice, bool is_priority) {
  for (auto &road : ROADS) {
    road.run_car_in_init_list(time_slice, is_priority, 2, this->on_road_num);
  }
}
void Judger::drive_car_in_wait_state() {
  while (true) {
    this->locked_car_queue.clear();
    this->LOCKED = true;
    bool have_wait = false;
    for (auto &cross : CROSSES) {
      for (auto road_id : cross.sorted_roads) {
        if (road_id == -1) {
          continue;
        }

        Road &road = ROADS[ID_ROAD[road_id]];
        int dir = (road.to == cross.id ? 0 : 1);
        while (true) {
          int car_id = road.get_high_level(dir);
          if (car_id == -1) {
            break;
          }
          Car &car = CARS[ID_CAR[car_id]];
          if (this->conflict(car)) {
            break;
          }

          have_wait = true;
          this->locked_car_queue.insert(car.id);
          int pre_road_id = car.local_road_id;

          car.move_to_next_road();

          if (car.status == END) {
            Road &road_1 = ROADS[ID_ROAD[pre_road_id]];
            road_1.run_car_in_init_list(this->now_time, true, dir,
                                        this->on_road_num);

            // 车辆驶入下一条路，上一条路的vis_cnt--;
            if (car.local_road_id != pre_road_id) {
              int dir = (car.from == road_1.to ? 0 : 1);
              if (car.if_preset) {
                road_1.vis_cnt[dir] -= PRESET_CAR_ADD_VIS_CNT;
              } else {
                road_1.vis_cnt[dir] -= NORMAL_CAR_ADD_VIS_CNT;
              }
            }

            this->locked_car_queue.erase(car.id);
            this->LOCKED = false;
            if (car.if_end == true) {
              this->on_road_num--;
              car.to_end_time = this->now_time;

              if (RELATION.find(car.id) != RELATION.end()) {
                this->all_car_tol_time +=
                    (this->now_time - car.plan_time) * RIGHT_RATE;
              } else {
                this->all_car_tol_time += (this->now_time - car.plan_time);
              }

              this->running_car_num--;
              if (car.priority > 0) {
                this->running_pri_num--;

                if (RELATION.find(car.id) != RELATION.end()) {
                  this->priority_tol_time +=
                      (this->now_time - car.plan_time) * RIGHT_RATE;
                } else {
                  this->priority_tol_time += (this->now_time - car.plan_time);
                }
                if (this->running_pri_num <= 0) {
                  this->priority_time = this->now_time - this->first_pri_time;
                }
              }
            }
          } else {
            break;
          }
        }
      }
    }
    if (!have_wait) {
      this->LOCKED = false;
      break;
    }
    if (this->LOCKED || this->running_car_num <= 0) {
      break;
    }
  }
}
bool Judger::is_finish() {
  if (this->LOCKED || this->running_car_num <= 0) {
    return true;
  }
  return false;
}
/*******************************Judge 输出调试部分***************************/
void Judger::out_debug() {
  for (auto road : ROADS) {
    int index = 0;
    for (auto pipe : road.girds_from) {
      std::cout << road.id << "," << this->now_time << "," << index++ << ",";
      while (!pipe.empty()) {
        int car_id = pipe.front();
        pipe.pop();
        Car car = CARS[ID_CAR[car_id]];
        std::cout << car.local_gird_col - 1 << "," << car.id << ",";
      }
      std::cout << endl;
    }
    index = 0;
    for (auto pipe : road.girds_to) {
      std::cout << road.id << "," << this->now_time << "," << index++ << ",";
      while (!pipe.empty()) {
        int car_id = pipe.front();
        pipe.pop();
        Car car = CARS[ID_CAR[car_id]];
        std::cout << car.local_gird_col - 1 << "," << car.id << ",";
      }
      std::cout << endl;
    }
  }
}
void Judger::out_result() {
  std::cerr << "--- [" << this->now_time << " -> ";
  std::cerr << "end_all = " << this->tol_car_num - this->running_car_num;
  std::cerr << ", left_all = " << this->running_car_num;
  std::cerr << " | end_pri = " << this->priority_num - this->running_pri_num;
  std::cerr << " left_pri = " << this->running_pri_num;
  std::cerr << " | on_road_num = " << this->on_road_num;
  std::cerr << " | go_on_road = " << this->go_on_road;
  std::cerr << " | tol_on_road = " << this->tol_on_road;
  std::cerr << " ] ---\n";
}
void Judger::out_locked() {
  return;
  std::cerr << "\n*********************************************************\n";
  for (auto car_id : this->locked_car_queue) {
    Car &car = CARS[ID_CAR[car_id]];
    DEBUG(car.id, car.from, car.to);
    car.out_cross();
  }
  std::cerr << "*********************************************************\n";
}
/****************************Judge 路径规划，死锁回退***********************/
bool dfs(int u, int fa, std::map<int, bool> &vis, std::vector<int> &crs,
         int &back, std::map<int, int> &pre) {
  if (vis[u] == true) {
    back = fa;
    return false;
  }
  vis[u] = true;

  auto link = [](int r1, int r2) {
    if (LINK[{r1, r2}] == true) {
      return true;
    }
    return false;
  };
  bool flag = true;
  for (auto v : crs) {
    if (v != u && v != fa && link(u, v)) {
      pre[v] = u;
      flag = dfs(v, u, vis, crs, back, pre);
      if (!flag) {
        return false;
      }
    }
  }
  return true;
}
void Judger::find_circle_roads(std::set<int> &circle_cross) {
  std::set<int> foo_st;
  for (auto car_id : this->locked_car_queue) {
    Car &c = CARS[ID_CAR[car_id]];
    foo_st.insert(c.from);
    foo_st.insert(c.to);
    // DEBUG(c.from, c.to);
  }
  std::vector<int> crs;
  for (auto car_id : foo_st) {
    crs.emplace_back(car_id);
  }

  // crs = {405, 112, 96, 100, 421, 470};
  // crs = {100, 1755, 153, 493, 1320, 1011, 69, 979, 487};
  std::map<int, bool> vis;
  std::map<int, int> pre;
  int u = crs[0];
  int back = -1;
  dfs(u, -1, vis, crs, back, pre);
  int e = pre[back];
  circle_cross.insert(back);
  while (e != back) {
    circle_cross.insert(e);
    e = pre[e];
  }
}
void Judger::create_roads() {
  // 遍历路口，获取所有begin_time到达当前时间片的车辆
  std::vector<std::vector<int>> fake_cars;
  for (auto &cross : CROSSES) {
    std::vector<int> wait_to_load;
    for (auto car_id : cross.in_init_cars) {
      Car &car = CARS[ID_CAR[car_id]];
      if (car.begin_time > this->now_time) {
        break;
      }
      wait_to_load.emplace_back(car_id);
    }
    sort(wait_to_load.begin(), wait_to_load.end(),
         [&](const int &x, const int &y) {
           Car &car1 = CARS[ID_CAR[x]];
           Car &car2 = CARS[ID_CAR[y]];
           if (car1.priority == car2.priority) {
             if (car1.speed == car2.speed) {
               return car1.begin_time < car2.begin_time;
             }
             return car1.speed > car2.speed;
           }
           return car1.priority > car2.priority;
         });
    fake_cars.emplace_back(wait_to_load);
  }

  // 所有预制车辆，必须上路，路径在死锁back的时候可以选择重新规划。不超过10%
  std::vector<int> can_cars;
  for (auto cars : fake_cars) {
    for (auto car_id : cars) {
      Car &car = CARS[ID_CAR[car_id]];
      if (car.if_preset) {
        this->go_on_road++;
        can_cars.emplace_back(car_id);
      }
    }
  }

  // 横向遍历cross的车库，存入Fake，一个路口取第一辆，一次便利，保证均匀
  std::vector<int> index(CROSSES.size(), 0);
  std::vector<int> Fake;
  while (true) {
    bool have_car = false;
    for (int i = 0; i < (int)index.size(); i++) {
      int pos = index[i], car_id = -1;
      if (pos < (int)fake_cars[i].size()) {
        have_car = true;
        car_id = fake_cars[i][pos];
        Car &car = CARS[ID_CAR[car_id]];
        if (!car.if_preset) {
          Fake.emplace_back(car.id);
        }
      }
      index[i] = pos + 1;
    }
    if (!have_car) {
      break;
    }
  }

  int LIMIT_NUM = this->tol_length * 0.7;
  auto go_break = [&]() {
    if (this->on_road_num + this->go_on_road >= LIMIT_NUM) {
      return true;
    }
    return false;
  };
  // 先让优先车辆上路
  std::map<int, bool> vis;
  // for (auto car_id : Fake) {
  //   if (go_break()) {
  //     break;
  //   }
  //   Car &car = CARS[ID_CAR[car_id]];
  //   if (!car.if_preset && car.priority != 0) {
  //     vis[car.id] = true;
  //     car.dijkstra(5, car.st);
  //     this->go_on_road++;
  //     can_cars.emplace_back(car_id);
  //   }
  // }
  // 便利Fake的cars，如果满足上路条件(地图阈值和道路占比)，则可以上路，否则延迟
  for (auto car_id : Fake) {
    if (vis[car_id]) {
      continue;
    }
    Car &car = CARS[ID_CAR[car_id]];
    if (go_break()) {
      car.begin_time = this->now_time + 1;
      continue;
    }
    car.dijkstra(5, car.st);
    this->go_on_road++;
    can_cars.emplace_back(car_id);
  }

  // 所有可以上路的汽车，进行入库操作
  for (auto car_id : can_cars) {
    Car &car = CARS[ID_CAR[car_id]];
    if (!car.can_run_to_end || car.local_road_id != 0) {
      DEBUG("cannot to end or car.local_road_id != 0");
    }
    if (car.ans_roads.size() < 1) {
      DEBUG("car.ans_roads == 0");
    }
    this->tol_on_road++;
    Cross &cross = CROSSES[ID_CROSS[car.st]];
    cross.in_init_cars.remove(car_id);
    Road &road = ROADS[ID_ROAD[car.ans_roads[0]]];
    int dir = (road.from == car.st ? 0 : 1);
    road.in_init_list[dir].emplace_back(car_id);
  }
}
void Judger::save() {
  int tl = this->now_time - SAVE_TIME * 5;
  for (int i = 0; i < tl; i += SAVE_TIME) {
    auto it_car = SAVE_CARS.find(i);
    auto it_cross = SAVE_CROSSES.find(i);
    auto it_road = SAVE_ROADS.find(i);
    auto it_judger = SAVE_JUDGER.find(i);
    if (it_car != SAVE_CARS.end()) {
      SAVE_CARS.erase(it_car);
    }
    if (it_cross != SAVE_CROSSES.end()) {
      SAVE_CROSSES.erase(it_cross);
    }
    if (it_road != SAVE_ROADS.end()) {
      SAVE_ROADS.erase(it_road);
    }
    if (it_judger != SAVE_JUDGER.end()) {
      SAVE_JUDGER.erase(it_judger);
    }
  }

  SAVE_CARS[this->now_time].clear();
  SAVE_CROSSES[this->now_time].clear();
  SAVE_ROADS[this->now_time].clear();

  for (auto car : CARS) {
    SAVE_CARS[this->now_time].emplace_back(car);
  }
  for (auto cross : CROSSES) {
    SAVE_CROSSES[this->now_time].emplace_back(cross);
  }
  for (auto road : ROADS) {
    SAVE_ROADS[this->now_time].emplace_back(road);
  }
  SAVE_JUDGER[this->now_time] = (*this);
}
void Judger::back() {
  TOL_LOCK_NUM++;
  std::set<int> circle_cross;
  this->find_circle_roads(circle_cross);
  // 发生死锁时，换上的所有车视为LOCKED_CARS
  std::vector<int> fake_cars;
  for (auto car_id : this->locked_car_queue) {
    Car &car = CARS[ID_CAR[car_id]];
    Road &road = ROADS[ID_ROAD[car.local_road_id]];
    int dir = (road.from == car.from ? 0 : 1);
    std::vector<std::queue<int>> cars =
        (dir == 0 ? road.girds_from : road.girds_to);
    for (auto pipe : cars) {
      while (!pipe.empty()) {
        Car &c = CARS[ID_CAR[pipe.front()]];
        pipe.pop();
        if (c.ans_roads[c.next_road_id] == car.ans_roads[car.next_road_id]) {
          fake_cars.emplace_back(c.id);
        }
      }
    }
  }

  // 筛选
  std::set<int> LOCKED_CARS;
  int up = fake_cars.size();
  for (int i = 0; i < up; i++) {
    if (i % TOL_LOCK_NUM == 0 && TOL_LOCK_NUM >= 3) {
      continue;
    }
    Car &c = CARS[ID_CAR[fake_cars[i]]];
    LOCKED_CARS.insert(c.id);
  }

  // LOCKED_CARS = this->locked_car_queue;
  // fix:修复时间片, back:回退时间片
  int fix_time = max(0, this->now_time - 1);
  while (fix_time % SAVE_TIME != 0) {
    fix_time--;
  }
  int back_time = fix_time - 1;
  while (back_time % SAVE_TIME != 0) {
    back_time--;
  }

  // 回到fix_time开始修复
  int failed = 0;
  std::map<int, std::vector<int>> ANS_ROADS;
  std::set<int> in_circle = circle_cross;
  for (auto his_car : SAVE_CARS[fix_time]) {
    if (LOCKED_CARS.find(his_car.id) != LOCKED_CARS.end()) {
      if (his_car.preset && his_car.priority == 0) {
        CHANED_NUM++;
        if (CHANED_NUM < PRESET_NUM * 0.1 || his_car.need_change) {
          bool flag = his_car.fix(in_circle);
          if (!flag) {
            failed++;
          }
          ANS_ROADS[his_car.id] = his_car.ans_roads;
        }
      } else {
        bool flag = his_car.fix(in_circle);
        if (!flag) {
          failed++;
        }
        ANS_ROADS[his_car.id] = his_car.ans_roads;
      }
    }
  }
  std::cerr << "\n" << now_time << " back to " << back_time << "\n";
  DEBUG(circle_cross);
  DEBUG(LOCKED_CARS.size(), failed);
  std::cerr << "\n";

  // cross,road,judger进行回退
  for (auto car : SAVE_CARS[back_time]) {
    Car &now_car = CARS[ID_CAR[car.id]];
    now_car = car;
    if (ANS_ROADS.find(car.id) != ANS_ROADS.end()) {
      if (now_car.if_preset) {
        now_car.if_changed = true;
        now_car.need_change = true;
      }
      now_car.fix_lock = true;
      now_car.can_run_to_end = true;
      now_car.ans_roads = ANS_ROADS[car.id];
    }
  }
  for (auto cross : SAVE_CROSSES[back_time]) {
    CROSSES[ID_CROSS[cross.id]] = cross;
  }
  for (auto road : SAVE_ROADS[back_time]) {
    ROADS[ID_ROAD[road.id]] = road;
  }
  (*this) = SAVE_JUDGER[back_time];
}
/*******************************Judge main***************************/
void Judger::entry() {
  this->prepare_clear();
  this->prepare_judge();
  DEBUG(this->tol_length, this->tol_length_duplex);
  DEBUG(this->tol_channel, this->tol_channel_duplex);
  // judge main
  this->now_time = 0;
  while (true) {
    this->now_time++;
    // 规划路线
    this->go_on_road = 0;
    this->create_roads();
    // 标记状态 && move
    this->drive_just_current_road();
    // 待出发车辆上路 (优先车辆先上路)
    this->drive_car_init_list(this->now_time, true);
    // 所有车进行调度
    this->drive_car_in_wait_state();
    // 待出发车辆上路 （所有车辆，有优先车辆，优先车辆先上路）
    this->drive_car_init_list(this->now_time, false);

    // 存档
    if (this->now_time % SAVE_TIME == 0) {
      this->save();
    }

    if (this->is_finish()) {
      if (this->LOCKED) {
        this->out_locked();
        this->back();
      } else {
        break;
      }
    }
    if (this->now_time % 10 == 0) {
      this->out_result();
    }
  }

  // return result
  std::cerr << "\n**********************************************************\n";
  if (this->LOCKED) {
    std::cerr << "> [ system locked ]" << std::endl;
    std::vector<int> locked_cross;
    for (auto car_id : locked_car_queue) {
      Car car = CARS[ID_CAR[car_id]];
      locked_cross.emplace_back(car.to);
    }
    DEBUG(locked_car_queue);
    DEBUG(locked_cross);
    std::cerr
        << "\n**********************************************************\n";
  } else {
    this->all_car_time = this->now_time;
    this->ans_time = this->fac_a * this->priority_time + this->all_car_time;
    this->ans_tol_time =
        this->fac_b * this->priority_tol_time + this->all_car_tol_time;

    DEBUG(fac_a, fac_b);
    std::cerr << "> priority: [dispatch_time = " << this->priority_time
              << ", total_time = " << this->priority_tol_time << "]"
              << std::endl;

    std::cerr << "> total:    [dispatch_time = " << this->all_car_time
              << ", total_time = " << this->all_car_tol_time << "]"
              << std::endl;

    std::cerr << "> answer:   [dispatch_time = " << this->ans_time * RIGHT_RATE
              << ", total_time = " << this->ans_tol_time << "]" << std::endl;
  }
  std::cerr << "**********************************************************\n";
}
