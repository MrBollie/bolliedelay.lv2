#ifndef __BOLLIEFILTER_H__
#define __BOLLIEFILTER_H__

#define PI 3.141592

typedef struct bfilter {
    double  rate;
    float   freq;
    float   Q;
    float   a0;
    float   a1;
    float   a2;
    float   b0;
    float   b1;
    float   b2;
    float   in_buf[3];
    float   processed_buf[3];
    unsigned int fill_count;
} BollieFilter;

void bf_init(BollieFilter*);
void bf_reset(BollieFilter*); 
float bf_lcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf); 

float bf_hcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf); 
    

#endif
