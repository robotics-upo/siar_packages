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
  
  inline T apply(T value) const {
    T ret_val = (value > _high)?_high:value;
    return (ret_val < _low)?_low:value;
  }
  
  inline T getWidth() const {
    return _high - _low;
  }
  
  double _low,_high;
};

inline uint16_t from_two_bytes(uint8_t b1, uint8_t b2) {
  uint16_t ret_val = b1;
  ret_val = ret_val << 8;
  return ret_val + b2;
}

inline int16_t from_two_bytes_signed(uint8_t b1, uint8_t b2) {
  uint16_t aux = from_two_bytes(b1, b2);
  
  
  int16_t ret;
  if (aux > 32768) {
    int32_t a;
    a = -65536 + aux;
    ret = a;
  } else {
    ret = aux;
  }
//   ROS_INFO("from_two_bytes_signed b1 = %u \t b2=%u \t aux =%u\t ret=%d", b1, b2, aux, ret);
  
  return ret;
}

#endif