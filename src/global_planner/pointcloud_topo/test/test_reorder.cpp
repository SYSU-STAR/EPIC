/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-30 10:34:18
 * @LastEditTime: 2023-12-30 10:38:25
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */
#include <iostream>
#include <unordered_set>
#include <vector>
using namespace std;
typedef pair<int, int> PII;
struct PIIHash {
  std::size_t operator()(const PII& p) const {
    std::size_t h1 = std::hash<int>{}(p.first);
    std::size_t h2 = std::hash<int>{}(p.second);
    return h1 ^ h2; // 采用异或运算符来组合哈希值
  }
};

struct PIIEquals {
  bool operator()(const PII& p1, const PII& p2) const {
    return p1.first == p2.first && p1.second == p2.second;
  }
};
void mask_reorder(std::vector<PII>& pvec) {
  std::unordered_set<PII, PIIHash, PIIEquals> set;
  for (auto& p : pvec) {
    set.insert(p);
  }
  pvec.clear();
  unordered_set<int> a;
  for (auto& p : set) {
    a.insert(p.first);
    a.insert(p.second);
  }
  vector<bool> mask(a.size(), false);
  while (!set.empty()) {
    std::unordered_set<PII, PIIHash, PIIEquals> pair_2_remove;
    mask = vector<bool>(mask.size(), false);
    for (auto& p : set) {
      if (mask[p.first] || mask[p.second]) {
        continue;
      }
      pvec.push_back(p);
      pair_2_remove.insert(p);
      mask[p.first] = true;
      mask[p.second] = true;
    }
    for (auto& p : pair_2_remove) {
      set.erase(p);
    }
  }
}
int main(int argc, char** argv) {
  vector<PII> pvec;
  for (int i = 0; i < 8; i++) {
    for (int j = i; j < 8; j++) {
      pvec.emplace_back(i, j);
    }
  }
  for (auto& p : pvec) {
    cout << p.first << " " << p.second << endl;
  }
  cout << "begin mask reorder" << endl;
  mask_reorder(pvec);
  for (auto& p : pvec) {
    cout << p.first << " " << p.second << endl;
  }
}
