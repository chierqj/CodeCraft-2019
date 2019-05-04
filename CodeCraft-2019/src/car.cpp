#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <vector>
#include "cross.h"
#include "data.h"
#include "debug.h"
#include "helper.h"
#include "judger.h"
#include "road.h"

/************************Car initial and debug*************************/
Car::Car() {}
Car::Car(int id, int st, int ed, int speed, int plan_time, int priority,
         int preset) {
  this->id = id;
  this->st = st;
  this->ed = ed;
  this->speed = speed;
  this->plan_time = plan_time;
  this->priority = priority;
  this->preset = preset;
}
void Car::debug_params() { DEBUG(id, begin_time, on_road_time, to_end_time); }
void Car::out_cross() {
  int pre_cross_id = this->st;
  std::cerr << this->id << ": ";
  for (auto road_id : this->ans_roads) {
    std::cerr << pre_cross_id << "->";
    Road &road = ROADS[ID_ROAD[road_id]];
    pre_cross_id = (road.from == pre_cross_id ? road.to : road.from);
  }
  std::cerr << pre_cross_id << "\n";
}
/*******************************Car 判题器部分***************************/
void Car::confirm_status(int pre_car_id) {
  if (pre_car_id == -1) {
    int S = this->local_gird_col - 1;
    if (this->SV <= S) {
      this->status = END;
      this->local_gird_col -= this->SV;
    } else {
      this->status = WAIT;
    }
  } else {
    Car pre_car = CARS[ID_CAR[pre_car_id]];
    int S = this->local_gird_col - pre_car.local_gird_col - 1;
    if (this->SV <= S) {
      this->status = END;
      this->local_gird_col -= this->SV;
    } else {
      this->status = pre_car.status;
      if (this->status == END) {
        this->local_gird_col -= S;
      }
    }
  }
}
bool Car::run_to_road() {
  Road &road = ROADS[ID_ROAD[this->ans_roads[this->next_road_id]]];
  if (this->st == road.from) {
    int row = 0;
    for (auto &pipe : road.girds_from) {
      row++;
      if (pipe.empty()) {
        this->status = END;
        this->from = this->st;
        this->to = road.to;
        this->local_road_id = road.id;
        this->SV = min(this->speed, road.speed);
        this->local_gird_row = row;
        this->local_gird_col = road.length - this->SV + 1;
        this->next_road_id++;
        pipe.push(this->id);
        return true;
      } else {
        Car pre_car = CARS[ID_CAR[pipe.back()]];
        int S2 = road.length - pre_car.local_gird_col;
        int S1 = min(this->speed, road.speed);

        if (S2 == 0) {
          if (pre_car.status == WAIT) {
            return false;
          } else {
            continue;
          }
        }

        if (S1 <= S2) {
          this->status = END;
          this->from = this->st;
          this->to = road.to;
          this->local_road_id = road.id;
          this->SV = min(this->speed, road.speed);
          this->local_gird_row = row;
          this->local_gird_col = road.length - S1 + 1;
          this->next_road_id++;
          pipe.push(this->id);
          return true;
        } else {
          if (pre_car.status == WAIT) {
            return false;
          }
          this->status = END;
          this->from = this->st;
          this->to = road.to;
          this->local_road_id = road.id;
          this->SV = min(this->speed, road.speed);
          this->local_gird_row = row;
          this->local_gird_col = road.length - S2 + 1;
          this->next_road_id++;
          pipe.push(this->id);
          return true;
        }
      }
    }
  } else {
    int row = 0;
    for (auto &pipe : road.girds_to) {
      row++;
      if (pipe.empty()) {
        this->status = END;
        this->from = this->st;
        this->to = road.from;
        this->local_road_id = road.id;
        this->SV = min(this->speed, road.speed);
        this->local_gird_row = row;
        this->local_gird_col = road.length - this->SV + 1;
        this->next_road_id++;
        pipe.push(this->id);
        return true;
      } else {
        Car pre_car = CARS[ID_CAR[pipe.back()]];
        int S2 = road.length - pre_car.local_gird_col;
        int S1 = min(this->speed, road.speed);
        if (S2 == 0) {
          if (pre_car.status == WAIT) {
            return false;
          } else {
            continue;
          }
        }

        if (S1 <= S2) {
          this->status = END;
          this->from = this->st;
          this->to = road.from;
          this->local_road_id = road.id;
          this->SV = min(this->speed, road.speed);
          this->local_gird_row = row;
          this->local_gird_col = road.length - S1 + 1;
          this->next_road_id++;
          pipe.push(this->id);
          return true;
        } else {
          if (pre_car.status == WAIT) {
            return false;
          }
          this->status = END;
          this->from = this->st;
          this->to = road.from;
          this->local_road_id = road.id;
          this->SV = min(this->speed, road.speed);
          this->local_gird_row = row;
          this->local_gird_col = road.length - S2 + 1;
          this->next_road_id++;
          pipe.push(this->id);
          return true;
        }
      }
    }
  }
  return false;
}
void Car::move_to_next_road() {
  Road &now_road = ROADS[ID_ROAD[this->local_road_id]];
  if (this->to == this->ed) {
    now_road.remove_car(*this);
    now_road.adjust_pipe(*this);
    this->if_end = true;
    this->status = END;
    return;
  }

  Road &next_road = ROADS[ID_ROAD[this->ans_roads[this->next_road_id]]];
  int S2 = min(this->speed, next_road.speed) - (this->local_gird_col - 1);
  if (S2 <= 0) {
    this->status = END;
    this->local_gird_col = 1;
    now_road.adjust_pipe(*this);
    return;
  }

  int next_row = 0;
  int last_use_car_id = next_road.last_use_car_id(this->to, next_row);
  if (last_use_car_id == 0) {
    now_road.remove_car(*this);
    now_road.adjust_pipe(*this);
    this->status = END;
    this->from = this->to;
    this->to = (next_road.from == this->from ? next_road.to : next_road.from);
    this->local_road_id = next_road.id;
    this->local_gird_row = next_row;
    this->SV = min(this->speed, next_road.speed);
    int S2 = this->SV - (this->local_gird_col - 1);
    this->local_gird_col = next_road.length - S2 + 1;
    this->next_road_id++;
    next_road.add_car(*this);
  } else {
    Car next_car = CARS[ID_CAR[last_use_car_id]];
    int SV2 = min(this->speed, next_road.speed);
    int S2 = SV2 - (this->local_gird_col - 1);
    int dis = next_road.length - next_car.local_gird_col;
    if (dis <= 0) {
      this->status = next_car.status;
      if (this->status == END) {
        this->local_gird_col = 1;
        now_road.adjust_pipe(*this);
      }
    } else {
      if (S2 <= dis) {
        now_road.remove_car(*this);
        now_road.adjust_pipe(*this);
        this->status = END;
        this->from = this->to;
        this->to =
            (next_road.from == this->from ? next_road.to : next_road.from);
        this->local_road_id = next_road.id;
        this->local_gird_row = next_car.local_gird_row;
        this->local_gird_col = next_road.length - S2 + 1;
        this->SV = SV2;
        this->next_road_id++;
        next_road.add_car(*this);
      } else {
        this->status = next_car.status;
        if (this->status == END) {
          now_road.remove_car(*this);
          now_road.adjust_pipe(*this);
          this->status = END;
          this->from = this->to;
          this->to =
              (next_road.from == this->from ? next_road.to : next_road.from);
          this->local_road_id = next_road.id;
          this->local_gird_row = next_car.local_gird_row;
          this->local_gird_col = next_car.local_gird_col + 1;
          this->SV = SV2;
          this->next_road_id++;
          next_road.add_car(*this);
        }
      }
    }
  }
}
void Car::clear() {
  this->from = -1;
  this->to = -1;
  this->status = END;
  this->local_road_id = 0;
  this->local_gird_row = -1;
  this->local_gird_col = -1;
  this->SV = 0;
  this->next_road_id = 0;
  this->if_end = false;
  this->on_road_time = 0;
  this->to_end_time = 0;
}
/*******************************Car 规划路径部分***************************/
void Car::dijkstra(int label, int start) {
  // 如果车辆可以行驶到终点，代表已经规划过路径了。(在死锁回退时规划好了，直接return)
  if (this->fix_lock) {
    return;
  }
  std::vector<bool> vis(MAX_CROSS_CNT);
  std::vector<int> pre(MAX_CROSS_CNT);
  std::vector<double> dis(MAX_CROSS_CNT, 1e8 * 1.0);

  struct Node {
    int id;
    int pre_road_id;
    double val;
    bool operator<(const Node &r) const { return val > r.val; }
  };

  priority_queue<Node> Q;
  dis[start] = 0.0;
  Q.push(Node{start, -1, 0.0});

  /*
   * pre_node: 上一时刻的状态，包括上一条经过的道路id，以及cross_id
   * road: 当前面临选择的道路
   */
  auto judge = [&](int u, int v, int pre_road_id, Road &road) {
    int dir = (road.to == v ? 0 : 1);
    Cross &next_cross = CROSSES[ID_CROSS[v]];
    double p = log(next_cross.layer);
    p = 1.0;
    double val = road.dijk_value(this->id, dir) / p;
    if (dis[u] + val < dis[v]) {
      dis[v] = dis[u] + val;
      Q.push({v, road.id, dis[v]});
      pre[v] = road.id;
    }
  };

  int ok = 0;
  while (!Q.empty()) {
    Node h = Q.top();
    Q.pop();
    int u = h.id;
    if (u == this->ed) {
      ok = 1;
      break;
    }
    if (vis[u]) {
      continue;
    }
    vis[u] = true;
    for (auto road_id : GRAPH[label][u]) {
      Road &road = ROADS[ID_ROAD[road_id]];
      int v = (road.to == u ? road.from : road.to);
      int dir = (road.from == u ? 0 : 1);
      if (!vis[v] && road.locked_cost[dir] <= LOCK_LIMIT_WEIGHT) {
        judge(u, v, h.pre_road_id, road);
      }
    }
  }

  if (ok == 0) {
    return;
  }

  this->can_run_to_end = true;
  this->ans_roads.clear();
  int e = this->ed;
  while (e != start) {
    int road_id = pre[e];
    Road &road = ROADS[ID_ROAD[road_id]];
    this->ans_roads.emplace_back(road_id);
    if (road.to == e) {
      e = road.from;
    } else {
      e = road.to;
    }
  }
  reverse(this->ans_roads.begin(), this->ans_roads.end());
}
void Car::son_fix(int start, int end, int LIMIT, std::vector<int> &roads,
                  std::set<int> &cirle_cross) {
  std::vector<bool> vis(MAX_CROSS_CNT);
  std::vector<int> pre(MAX_CROSS_CNT);
  std::vector<double> dis(MAX_CROSS_CNT, 1e8 * 1.0);
  struct Node {
    int id;
    int pre_road_id;
    double val;
    bool operator<(const Node &r) const { return val > r.val; }
  };

  priority_queue<Node> Q;
  dis[start] = 0.0;
  Q.push(Node{start, -1, 0.0});

  auto judge = [&](int u, int v, int pre_road_id, Road &road) {
    int dir = (road.to == v ? 0 : 1);
    Cross &next_cross = CROSSES[ID_CROSS[v]];
    double p = log(next_cross.layer);
    p = 1.0;
    double val = road.dijk_value(this->id, dir) / p;
    if (dis[u] + val < dis[v]) {
      dis[v] = dis[u] + val;
      Q.push({v, road.id, dis[v]});
      pre[v] = road.id;
    }
  };

  int ok = 0;
  bool flag = true;
  while (!Q.empty()) {
    Node h = Q.top();
    Q.pop();
    int u = h.id;
    if (u == end) {
      ok = 1;
      break;
    }
    if (vis[u]) {
      continue;
    }
    vis[u] = true;
    for (auto road_id : GRAPH[5][u]) {
      Road &road = ROADS[ID_ROAD[road_id]];
      int v = (road.to == u ? road.from : road.to);
      int dir = (road.from == u ? 0 : 1);
      if (road.id == LIMIT) {
        continue;
      }
      if (flag == true && v != end &&
          cirle_cross.find(v) != cirle_cross.end()) {
        flag = false;
        continue;
      }
      if (!vis[v] && road.locked_cost[dir] <= LOCK_LIMIT_WEIGHT) {
        judge(u, v, h.pre_road_id, road);
      }
    }
  }

  if (ok == 0) {
    this->dijkstra(5, start);
    // DEBUG("son fix error");
    return;
  }

  int e = end;
  roads.clear();
  while (e != start) {
    int road_id = pre[e];
    Road &road = ROADS[ID_ROAD[road_id]];
    roads.emplace_back(road_id);
    if (road.to == e) {
      e = road.from;
    } else {
      e = road.to;
    }
  }
  reverse(roads.begin(), roads.end());
}
bool Car::fix(std::set<int> &circle_cross) {
  if (this->to == -1) {
    int start = this->st;
    int end = this->ed;
    int LIMIT = -1;
    std::vector<int> roads;
    this->son_fix(start, end, LIMIT, roads, circle_cross);
    this->ans_roads = roads;
  } else {
    std::vector<int> his_road(this->ans_roads.begin(),
                              this->ans_roads.begin() + this->next_road_id);
    int start = this->to;
    int end = this->ed;
    int LIMIT = this->local_road_id;
    std::vector<int> vt;
    this->son_fix(start, end, LIMIT, vt, circle_cross);
    his_road.insert(his_road.end(), vt.begin(), vt.end());
    this->ans_roads = his_road;
  }
  return true;
}