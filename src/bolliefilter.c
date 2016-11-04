#include "bolliefilter.h"
#include <math.h>

/**
* Initializes a BollieFilter object.
* \param bf Pointer to a BollieFilter object.
*/
void bf_init(BollieFilter* bf) {
    for (unsigned int i = 0 ; i < 3 ; ++i) {
        bf->in_buf[i] = 0;
        bf->processed_buf[i] = 0;
    }
    bf->fill_count = 0;
    bf->freq = 0;
    bf->Q = 0;
}


/**
* Resets a BollieFilter object.
*/
void bf_reset(BollieFilter* bf) {
    bf_init(bf);
}


/**
* Processes a frame using a low cut filter.
* \param in     Input sample
* \param freq   Filter cut off frequency
* \param Q      Filter quality
* \param rate   Current sampling rate
* \param bf     Pointer to the BollieFilter object
* \return       Output sample
* \todo         Validating parameters
*/
float bf_lcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf) {

    // Precalculate if needed.
    if (freq != bf->freq || Q != bf->Q || rate != bf->rate) {
        bf->freq = freq;
        bf->Q = Q;
        bf->rate = rate;
        float w0 = 2 * PI * bf->freq / bf->rate;
        float alpha = sin(w0) / (2*bf->Q);
        bf->a0 = 1+alpha;
        bf->a1 = -2 * cos(w0);
        bf->a2 = 1-alpha;
        bf->b0 = (1 + cos(w0)) / 2;
        bf->b1 = -(1 + cos(w0));
        bf->b2 = (1 + cos(w0)) / 2; 
    }

    // Filter roll
    bf->in_buf[2] = bf->in_buf[1];
    bf->in_buf[1] = bf->in_buf[0];
    bf->in_buf[0] = in;

    bf->processed_buf[2] = bf->processed_buf[1];
    bf->processed_buf[1] = bf->processed_buf[0];

    // See if we need to fill the buffers first
    if (bf->fill_count < 3) {
        bf->processed_buf[0] = in;
        bf->fill_count++;
        return 0;
    }

    return bf->processed_buf[0] =             
            (bf->b0 / bf->a0 * bf->in_buf[0]) +
            (bf->b1 / bf->a0 * bf->in_buf[1]) +
            (bf->b2 / bf->a0 * bf->in_buf[2]) -
            (bf->a1 / bf->a0 * bf->processed_buf[1]) -
            (bf->a2 / bf->a0 * bf->processed_buf[2]);
}


/**
* Processes a frame using a high cut filter.
* \param in     Input sample
* \param freq   Filter cut off frequency
* \param Q      Filter quality
* \param rate   Current sampling rate
* \param bf     Pointer to the BollieFilter object
* \return       Output sample
*/
float bf_hcf(const float in, const float freq, const float Q, 
    double rate, BollieFilter* bf) {

    // Precalculate if needed.
    if (freq != bf->freq || Q != bf->Q || rate != bf->rate) {
        bf->freq = freq;
        bf->Q = Q;
        bf->rate = rate;
        float w0 = 2 * PI * bf->freq / bf->rate;
        float alpha = sin(w0) / (2*bf->Q);
        bf->a0 = 1+alpha;
        bf->a1 = -2 * cos(w0);
        bf->a2 = 1-alpha;
        bf->b0 = (1 - cos(w0)) / 2;
        bf->b1 = 1 - cos(w0);
        bf->b2 = (1 - cos(w0)) / 2; 
    }

    // Filter roll
    bf->in_buf[2] = bf->in_buf[1];
    bf->in_buf[1] = bf->in_buf[0];
    bf->in_buf[0] = in;

    bf->processed_buf[2] = bf->processed_buf[1];
    bf->processed_buf[1] = bf->processed_buf[0];

    // See if we need to fill the buffers first
    if (bf->fill_count < 3) {
        bf->processed_buf[0] = in;
        bf->fill_count++;
        return 0;
    }

    return bf->processed_buf[0] =             
            (bf->b0 / bf->a0 * bf->in_buf[0]) +
            (bf->b1 / bf->a0 * bf->in_buf[1]) +
            (bf->b2 / bf->a0 * bf->in_buf[2]) -
            (bf->a1 / bf->a0 * bf->processed_buf[1]) -
            (bf->a2 / bf->a0 * bf->processed_buf[2]);
}

