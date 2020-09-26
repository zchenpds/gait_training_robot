#include "gait_training_robot/fir_filter.h"

namespace fir_filter {

  FirFilter::FirFilter() {
    for (int i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
      history[i] = 0;
    last_index = 0;
  }

  void FirFilter::put(Float input) {
    history[last_index++] = input;
    last_index %= SAMPLEFILTER_TAP_NUM;
  }

  Float FirFilter::get() {
    Float acc = 0;
    int index = last_index;
    for (int i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
      index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM-1;
      acc += history[index] * filter_taps[i];
    };
    return acc;
  }

};