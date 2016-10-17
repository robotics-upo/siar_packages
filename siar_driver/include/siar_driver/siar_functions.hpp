#ifndef __SIAR_FUNCTIONS_H__
#define __SIAR_FUNCTIONS_H__
#include <math.h>

// template <typename T> T sgn(T value) {
//   return (value > static_cast<T>(0.0)?static_cast<T>(1.0):((value < static_cast<T>(0.0))?static_cast<T>(-1.0):0.0);
// }

template <typename T = double> class DeadZone {
public:
  DeadZone (T pos_dead, T neg_dead):_pos_dead(pos_dead),_neg_dead(neg_dead){}
  DeadZone (T pos_dead = 1.0):_pos_dead(pos_dead),_neg_dead(-pos_dead){}
  
  inline double apply(T value) const {
    T ret_val = 0.0;
    if (value > static_cast<T>(0.0) && value > _pos_dead) {
      ret_val = value;
    } else if (value < static_cast<T>(0.0) && value < _neg_dead) {
      ret_val = value;
    }
    return ret_val;
  }
  
  double _pos_dead, _neg_dead;
};

template <typename T = double> class Saturate {
public:
  Saturate (T low, T high):_low(low), _high(high) {}
  Saturate (T high = 1.0):_low(-high), _high(high) {}
  
  inline double apply(T value) const {
    T ret_val = (value > _high)?_high:value;
    return (ret_val < _low)?_low:value;
  }
  
  double _low,_high;
};

#endif