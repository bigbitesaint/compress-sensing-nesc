
#include <stdio.h>
#include <math.h>
#include "box_muller.h"
// return random variable ~ [0,1]
#define PI 3.1415926

float fastlog2 (float x)
{
  union { float f; uint32_t i; } vx = { x };
  union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | ((uint32_t)(0x7e) << 23) };
  float y = vx.i;
  y *= 1.0 / (1lu << 23);
 
  return
    y - 124.22544637f - 1.498030302f * mx.f - 1.72587999f / (0.3520887068f + mx.f);
}

float fastlog (float x)
{
  return 0.69314718f * fastlog2 (x);
}


float
fastpow2 (float p)
{
  union { float f; uint32_t i; } vp = { p };
  int sign = (vp.i >> 31);
  int w = p;
  float z = p - w + sign;
  union { uint32_t i; float f; } v = { (1lu << 23) * (p + 121.2740838f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z) };
  return v.f;
}
 
inline float
fastexp (float p)
{
  return fastpow2 (1.442695040f * p);
}


float Q_rsqrt( float number )
{
        long i;
        float x2, y;
        const float threehalfs = 1.5F;
 
        x2 = number * 0.5F;
        y  = number;
        i  = * ( long * ) &y;                       // evil floating point bit level hacking
        i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
        y  = * ( float * ) &i;
        y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
 
        return y;
}


// mu = mean
// sigma = dev
// a,b are returned values
void norm(float mu,float sigma, float *a, float *b) 
{
   float rv1 = *a, rv2 = *b;
   //float t = sqrt(-2*log(rv1));
	float t = 1/(Q_rsqrt(-2*fastlog(rv1)));
   *a = t*tlp_cos[(uint16_t)(rv2*100.0)];
   *b = t*tlp_sin[(uint16_t)(rv2*100.0)];
   *a = mu + sigma*(*a);
   *b = mu + sigma*(*b);
}


void build_tlp()
{
	int i;
	for (i=0;i<100; ++i)
	{
		tlp_cos[i] = cosf(2*PI*((float)i/100.0f));
		tlp_sin[i] = sinf(2*PI*((float)i/100.0f));
	}
}
