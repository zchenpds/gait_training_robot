#ifndef ROBUST_ESTIMATOR_H
#define ROBUST_ESTIMATOR_H

#include <deque>
#include <numeric>

template<class VectorType>
class StancePhaseMEstimator
{
  constexpr static size_t max_size_ = 30;
  std::deque<VectorType> data_;
public:
  StancePhaseMEstimator() {}

  void clear() {data_.clear();}
  bool empty() {return data_.empty();}

  void put(const VectorType & v)
  {
    data_.push_back(v);
    while (data_.size() > max_size_)
      data_.pop_front();
  }

  VectorType getAverage() const
  {
    assert(data_.size());
    VectorType v0;
    v0.setZero();
    return std::accumulate(data_.begin(), data_.end(), v0) / data_.size();
  }
};


#endif // ROBUST_ESTIMATOR_H