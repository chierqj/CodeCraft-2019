#include "solve.h"
#include "car.h"
#include "cross.h"
#include "data.h"
#include "debug.h"
#include "helper.h"
#include "judger.h"
#include "road.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <vector>

Solve::Solve(std::string car_path, std::string road_path,
             std::string cross_path, std::string preset_answer_path,
             std::string answer_path) {
  this->car_path = car_path;
  this->road_path = road_path;
  this->cross_path = cross_path;
  this->preset_answer_path = preset_answer_path;
  this->answer_path = answer_path;
}
/********************Solve 读写文件部分(包括一些变量预处理)*******************/
void read_file(std::vector<std::string> &lines, std::string path) {
  lines.clear();
  fstream fin(path);
  std::string s;
  bool fir = true;
  while (getline(fin, s)) {
    if (fir) {
      fir = false;
      continue;
    }
    lines.emplace_back(s.substr(1, s.size() - 2));
  }
  fin.close();
}
void read_file_1(std::vector<std::string> &lines, std::string path) {
  lines.clear();
  fstream fin(path);
  std::string s;
  bool fir = true;
  while (getline(fin, s)) {
    if (fir) {
      fir = false;
      continue;
    }
    lines.emplace_back(s);
  }
  fin.close();
}
void Solve::input() {
  std::vector<std::string> lines;
  // car input data
  read_file(lines, this->car_path);
  int cnt = 0;
  for_each(lines.begin(), lines.end(), [&](std::string line) {
    std::vector<int> nums;
    split(line, nums, ',');
    Car car(nums[0], nums[1], nums[2], nums[3], nums[4], nums[5], nums[6]);
    CARS.emplace_back(car);
    ID_CAR[nums[0]] = cnt++;
  });

  // cross input data
  read_file(lines, this->cross_path);
  cnt = 0;
  for_each(lines.begin(), lines.end(), [&](std::string line) {
    std::vector<int> nums;
    split(line, nums, ',');
    Cross cross(nums[0], {nums[1], nums[2], nums[3], nums[4]});
    cross.dirs = {-1, -1, -1, -1};
    MAX_CROSS_CNT = max(MAX_CROSS_CNT, cross.id + 7);
    CROSSES.emplace_back(cross);
    ID_CROSS[nums[0]] = cnt++;
  });

  // road input data
  read_file(lines, this->road_path);
  cnt = 0;
  for_each(lines.begin(), lines.end(), [&](std::string line) {
    std::vector<int> nums;
    split(line, nums, ',');
    Road road(nums[0], nums[1], nums[2], nums[3], nums[4], nums[5], nums[6]);
    ROADS.emplace_back(road);
    ID_ROAD[nums[0]] = cnt++;
    LINK[{road.from, road.to}] = true;
    LINK[{road.to, road.from}] = true;
  });

  // preset_answer input data
  read_file(lines, this->preset_answer_path);
  for_each(lines.begin(), lines.end(), [&](std::string line) {
    std::vector<int> nums;
    split(line, nums, ',');
    Car &car = CARS[ID_CAR[nums[0]]];
    car.if_preset = true;
    car.can_run_to_end = true;
    car.begin_time = nums[1];
    PRESET_NUM++;
    std::vector<int> ans_roads;
    for (int i = 2; i < (int)nums.size(); i++) {
      ans_roads.emplace_back(nums[i]);
    }
    car.ans_roads = ans_roads;
  });

  read_file_1(lines, "../config/relation.txt");
  for_each(lines.begin(), lines.end(), [&](std::string line) {
    std::vector<int> nums;
    split(line, nums, ',');
    RELATION.insert(nums[0]);
  });
}
void Solve::write_answer() {
  sort(CARS.begin(), CARS.end(), [](const Car &car1, const Car &car2) {
    return car1.begin_time < car2.begin_time;
  });
  ofstream fout(this->answer_path);
  fout << "#(carId,StartTime,RoadId...)\n";

  for (auto &car : CARS) {
    if (car.if_preset && !car.if_changed) {
      continue;
    }
    fout << "(" << car.id << ", " << car.begin_time;
    for_each(car.ans_roads.begin(), car.ans_roads.end(),
             [&](int v) { fout << ", " << v; });
    fout << ")\n";
  }
  fout.close();
  std::cerr << "program over" << std::endl;
  /********************Solve
   * 读写文件部分(包括一些变量预处理)*********************/
}
/*************************Solve 创建地图以及标坐标************************/
void Solve::label_direction() {
  std::queue<int> Q;
  std::vector<int> vis(MAX_CROSS_CNT, false);

  Cross &st_cross = CROSSES[0];
  for (int i = 0; i < (int)st_cross.roads.size(); i++) {
    if (st_cross.roads[i] != -1) {
      st_cross.dirs[i] = i;
    }
  }
  Q.push(st_cross.id);
  vis[st_cross.id] = true;

  while (!Q.empty()) {
    Cross &h = CROSSES[ID_CROSS[Q.front()]];
    Q.pop();

    for (int i = 0; i < 4; i++) {
      if (h.roads[i] == -1) {
        continue;
      }
      int road_id = h.roads[i];
      Road &road = ROADS[ID_ROAD[road_id]];
      int nxt_cross_id = (road.from == h.id ? road.to : road.from);
      Cross &cross = CROSSES[ID_CROSS[nxt_cross_id]];
      if (vis[cross.id]) {
        continue;
      }
      vis[cross.id] = true;
      vector<int> foo(4);
      vector<int> ds(4);
      int index = 0;
      for (int j = 0; j < 4; j++) {
        if (cross.roads[j] == road_id) {
          index = j;
          break;
        }
      }
      if (index == 0) {
        foo = {0, 1, 2, 3};
      } else if (index == 1) {
        foo = {1, 2, 3, 0};
      } else if (index == 2) {
        foo = {2, 3, 0, 1};
      } else {
        foo = {3, 0, 1, 2};
      }

      if (h.dirs[i] == 0) {
        ds = {2, 3, 0, 1};
      } else if (h.dirs[i] == 1) {
        ds = {3, 0, 1, 2};
      } else if (h.dirs[i] == 2) {
        ds = {0, 1, 2, 3};
      } else {
        ds = {1, 2, 3, 0};
      }

      for (int j = 0; j < 4; j++) {
        cross.dirs[foo[j]] = ds[j];
      }
      for (int j = 0; j < 4; j++) {
        if (cross.roads[j] == -1) {
          cross.dirs[j] = -1;
        }
      }
      Q.push(cross.id);
    }
  }
}
void Solve::label_coordinate() {
  struct Node {
    int cross_id;
    int road_id;
    int dir;  // 0 1 2 3, u r d l
    int x, y;
  };
  std::queue<Node> Q;
  std::vector<bool> vis(MAX_CROSS_CNT, false);
  int start = CROSSES[0].id;
  Q.push(Node{start, -1, -1, 0, 0});
  vis[start] = true;
  std::vector<int> vt[4];
  vt[0] = {1, 3, 0};
  vt[1] = {2, 0, 1};
  vt[2] = {3, 1, 2};
  vt[3] = {0, 2, 3};
  while (!Q.empty()) {
    Node h = Q.front();
    Q.pop();

    Cross &cross_u = CROSSES[ID_CROSS[h.cross_id]];
    for (int i = 0; i < 4; i++) {
      int road_id_v = cross_u.roads[i];
      if (road_id_v == -1 || road_id_v == h.road_id) {
        continue;
      }

      int vx = h.x, vy = h.y;
      int vdir = 0;
      Road &road = ROADS[ID_ROAD[road_id_v]];
      int cd = (road.from == h.cross_id ? road.to : road.from);
      if (vis[cd]) {
        continue;
      }
      vis[cd] = true;
      Cross &cross_v = CROSSES[ID_CROSS[cd]];
      if (h.road_id == -1) {
        vdir = cross_u.dirs[i];
      } else {
        int turn_dir = cross_u.get_dir(h.road_id, road_id_v);
        vdir = vt[h.dir][turn_dir];
      }
      if (vdir == 0) {
        vy++;
      } else if (vdir == 1) {
        vx++;
      } else if (vdir == 2) {
        vy--;
      } else {
        vx--;
      }
      cross_v.x = vx;
      cross_v.y = vy;
      Q.push(Node{cd, road_id_v, vdir, vx, vy});
    }
  }
}
void Solve::find_center() {
  auto find_mid = [&](int label) {
    std::vector<int> vt;
    for (auto &cross : CROSSES) {
      int val = (label == 0 ? cross.x : cross.y);
      vt.emplace_back(val);
    }
    sort(vt.begin(), vt.end());
    vt.erase(unique(vt.begin(), vt.end()), vt.end());
    if (label == 0) {
      all_x = vt;
    } else {
      all_y = vt;
    }
    double mid_val = 0.0;
    int sz = vt.size();
    if (sz % 2 == 0) {
      mid_val = (vt[(sz - 1) / 2] + vt[sz / 2]) / 2.0;
    } else {
      mid_val = vt[sz / 2];
    }
    return mid_val;
  };
  double mid_x = find_mid(0);
  double mid_y = find_mid(1);
  double min_val = 1e8;
  for (auto &cross : CROSSES) {
    double abs_x = cross.x - mid_x;
    double abs_y = cross.y - mid_y;
    double val = abs_x * abs_x + abs_y * abs_y;
    if (val < min_val) {
      min_val = val;
    }
  }
  for (auto &cross : CROSSES) {
    double abs_x = cross.x - mid_x;
    double abs_y = cross.y - mid_y;
    double val = abs(abs_x) + abs(abs_y);
    cross.layer = val + 0.1;
    belong_x[cross.x].emplace_back(cross.id);
    belong_y[cross.y].emplace_back(cross.id);
  }
  for (auto &v : belong_x) {
    sort(v.second.begin(), v.second.end());
  }
  for (auto &v : belong_y) {
    sort(v.second.begin(), v.second.end());
  }
}
void Solve::create_graph() {
  auto create = [&](int label, int u, int r, int d, int l) {
    GRAPH[label].clear();
    GRAPH[label].resize(MAX_CROSS_CNT);
    for (auto &cross : CROSSES) {
      for (int i = 0; i < 4; i++) {
        int road_id = cross.roads[i];
        if (road_id == -1) {
          continue;
        }
        int dir = cross.dirs[i];
        if (dir == u || dir == r || dir == d || dir == l) {
          Road &road = ROADS[ID_ROAD[road_id]];
          if (road.is_duplex == 1 || road.from == cross.id) {
            GRAPH[label][cross.id].emplace_back(road_id);
          }
        }
      }
    }
  };
  create(1, 0, 1, -1, -1);
  create(2, -1, 1, 2, -1);
  create(3, -1, -1, 2, 3);
  create(4, 0, -1, -1, 3);
  create(5, 0, 1, 2, 3);
}
/***********************Solve 处理发车，cross填充部分*******************/
void Solve::adjust_after() {
  for (auto &car : CARS) {
    if (!car.if_preset) {
      car.begin_time = car.plan_time;
    }
  }
  // 出发路口预装载车辆
  sort(CARS.begin(), CARS.end(), [](const Car &car1, const Car &car2) {
    return car1.begin_time < car2.begin_time;
  });
  int last_begin_time = 0;
  for (auto &car : CARS) {
    Cross &cross = CROSSES[ID_CROSS[car.st]];
    cross.in_init_cars.emplace_back(car.id);
    last_begin_time = max(last_begin_time, car.begin_time);
  }
  // 打印最晚发车时间
  DEBUG(last_begin_time);
  Judger judger;
  judger.entry();
}
/*************************Solve main************************/
void Solve::run() {
  this->input();
  this->label_direction();
  this->label_coordinate();
  this->find_center();
  this->create_graph();
  this->adjust_after();
  this->write_answer();
}
