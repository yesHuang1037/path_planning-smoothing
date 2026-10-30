#pragma once
#include "point.h"
#include <queue>
#include <vector>

namespace navi_planner {
class BucketPrioQueue {
public:
  bool empty() const;
  void push(int priority, INTPOINT point);
  INTPOINT pop();
private:
  struct Entry {int priority; INTPOINT point;};
  struct Later {
    bool operator()(const Entry &first, const Entry &second) const {
      return first.priority > second.priority;
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, Later> queue_;
};
}
