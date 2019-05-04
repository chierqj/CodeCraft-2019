#include "data.h"

int MAX_CROSS_CNT = 0;                   // 最大的crossid
int CHANED_NUM = 0;                      // 修改的预制车辆的数目
int PRESET_NUM = 0;                      // 预制车的数目
std::vector<Car> CARS;                   // 输入cars
std::vector<Cross> CROSSES;              // 输入crosses
std::vector<Road> ROADS;                 // 输入roads
std::map<int, int> ID_CAR;               // [key -> value] => [car_id, index]
std::map<int, int> ID_CROSS;             // [key -> value] => [cross_id, index]
std::map<int, int> ID_ROAD;              // [key -> value] => [road_id, index]
std::vector<std::vector<int>> GRAPH[6];  // 1:右上 2:右下 3:左下 4:左上 5:全图

std::vector<int> all_x;  // 所有cross的x坐标集合，从小到大(sorted)
std::vector<int> all_y;  // 所有cross的y坐标集合，从小到大(sorted)
std::map<int, std::vector<int>> belong_x;  // 横坐标为x的cross集合(sorted)
std::map<int, std::vector<int>> belong_y;  // 纵坐标为y的cross集合(sorted)

std::map<int, std::vector<Car>> SAVE_CARS;       // Car历史存档
std::map<int, std::vector<Road>> SAVE_ROADS;     // Road历史存档
std::map<int, std::vector<Cross>> SAVE_CROSSES;  // Cross历史存档
std::map<int, Judger> SAVE_JUDGER;               // Judger历史存档

const int RANGE_WIDTH = 6;    // 局部搜索的范围
const int SEND_CAR_NUM = 60;  // 每个时间片的发车数目
const int SAVE_TIME = 6;      // 每隔多少时间片存档一次
const double LOCK_ADD_WEIGHT = 0.3;  // 每死锁一次，该车在该路上增加的死锁权重
const double LOCK_LIMIT_WEIGHT = 1.0;  // 该车在该路上的死锁权重上线，超过不让走
const double PRESET_CAR_ADD_VIS_CNT = 1.0;  // 预制车辆增加的道路的vis_cnt
const double NORMAL_CAR_ADD_VIS_CNT = 1.0;  // 普通车辆增加的道路的vis_cnt

std::map<int, std::map<int, int>> LOCKED_NUM;  // 某辆车在某个时间片死锁次数
int TOL_LOCK_NUM = 0;                          // 总死锁次数
std::map<std::pair<int, int>, bool> LINK;  // 判断两个路口是否相连

std::set<int> RELATION;

const double RIGHT_RATE = 10;
