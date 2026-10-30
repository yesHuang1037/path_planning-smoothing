#include "bucketedqueue.h"
#include <stdexcept>

namespace navi_planner {
bool BucketPrioQueue::empty() const {return queue_.empty();}
void BucketPrioQueue::push(int priority, INTPOINT point) {
  if (priority < 0) throw std::invalid_argument("negative obstacle queue priority");
  queue_.push({priority, point});
}
INTPOINT BucketPrioQueue::pop() {
  if (queue_.empty()) throw std::logic_error("empty obstacle queue");
  const auto point = queue_.top().point;
  queue_.pop();
  return point;
}
}
