#ifndef CALIB_H
#define CALIB_H

#define max(x, y) x > y ? x : y
// linear regression: calibration parameters
float calA[] = {1.000, 1.000, 1.051, 0.965, 1.014, 0.975, 1.011};
float calB[] = {0.000, 0.000, 0.235, 0.516, 0.759, 1.009, 1.245};

uint16_t convert(uint32_t power, uint16_t nodeid, uint16_t ticks) {
  if (nodeid < 7) {
    float newpower = 0.2462 * (1024.0 * calA[nodeid] * power / ticks - calB[nodeid] * 16.0);
    return (uint16_t) max(0.0, newpower);
  }
  else
    return (uint16_t) 0.2462 * (1024.0 / ticks) *  power;
}

#endif

  /* if (nodeid < 7) { */
  /*   float newpower = calA[nodeid] * power - calB[nodeid] * ticks / 64.0; */
  /*   return (uint16_t) max(0.0, 1024.0 * newpower / ticks); */
  /* } */
  /* else */
  /*   return (uint16_t) power; */
