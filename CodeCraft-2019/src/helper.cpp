#include "helper.h"
#include <algorithm>
#include <random>

void split(const std::string& s, std::vector<int>& sv, const char flag = ' ') {
  sv.clear();
  std::istringstream iss(s);
  std::string temp;

  while (std::getline(iss, temp, flag)) {
    sv.push_back(stoi(temp));
  }
  return;
}
// 随机数
std::default_random_engine e(time(nullptr));
int random_int(int l, int r) {
  std::uniform_int_distribution<int> u(l, r);
  return u(e);
}
double random_double(double l, double r) {
  std::uniform_real_distribution<double> u(l, r);
  return u(e);
}