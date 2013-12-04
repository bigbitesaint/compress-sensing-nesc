#ifndef _BOX_MULLER_H_
#define _BOX_MULLER_H_

float tlp_cos[100]={0};
float tlp_sin[100]={0};

void norm(float mu,float sigma, float *a, float *b) ;
void build_tlp();
#endif
