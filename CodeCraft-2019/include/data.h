#ifndef DATA_H
#define DATA_H

#include <map>
#include <set>
#include <vector>

#include "car.h"
#include "cross.h"
#include "judger.h"
#include "road.h"

extern int MAX_CROSS_CNT;
extern int CHANED_NUM;
extern int PRESET_NUM;
extern const int RANGE_WIDTH;
extern std::vector<Car> CARS;
extern std::vector<Cross> CROSSES;
extern std::vector<Road> ROADS;
extern std::map<int, int> ID_CAR;
extern std::map<int, int> ID_CROSS;
extern std::map<int, int> ID_ROAD;
extern std::vector<std::vector<int>> GRAPH[6];
extern std::map<int, std::vector<Car>> SAVE_CARS;
extern std::map<int, std::vector<Road>> SAVE_ROADS;
extern std::map<int, std::vector<Cross>> SAVE_CROSSES;
extern std::map<int, Judger> SAVE_JUDGER;
extern std::vector<int> all_x;
extern std::vector<int> all_y;
extern std::map<int, std::vector<int>> belong_x;
extern std::map<int, std::vector<int>> belong_y;

extern const int SEND_CAR_NUM;
extern const int SAVE_TIME;
extern const double LOCK_ADD_WEIGHT;
extern const double LOCK_LIMIT_WEIGHT;
extern const double PRESET_CAR_ADD_VIS_CNT;
extern const double NORMAL_CAR_ADD_VIS_CNT;
extern std::set<int> RELATION;
extern const double RIGHT_RATE;

extern std::map<int, std::map<int, int>> LOCKED_NUM;
extern int TOL_LOCK_NUM;
extern std::map<std::pair<int, int>, bool> LINK;
#endif