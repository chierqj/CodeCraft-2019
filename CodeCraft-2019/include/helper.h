#ifndef HELPER_H
#define HELPER_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

void split(const std::string &s, std::vector<int> &sv, const char flag);
int random_int(int l, int r);
double random_double(double l, double r);

#endif