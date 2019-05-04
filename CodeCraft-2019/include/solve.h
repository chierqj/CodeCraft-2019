#ifndef SOLVE_H
#define SOLVE_H

#include <iostream>
#include <map>
#include <vector>
#include "road.h"

class Road;

class Solve {
 public:
  Solve(std::string car_path, std::string road_path, std::string cross_path,
        std::string preset_answer_path, std::string answer_path);
  void run();
  void input();
  void write_answer();

  void create_graph();
  void label_direction();
  void label_coordinate();
  void find_center();
  void adjust_after();
 public:
  std::string car_path;
  std::string road_path;
  std::string cross_path;
  std::string preset_answer_path;
  std::string answer_path;
};

#endif
