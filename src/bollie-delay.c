/**
    Bollie Delay - (c) 2016 Thomas Ebeling https://ca9.eu

    This file is part of bolliedelay.lv2

    bolliedelay.lv2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    bolliedelay.lv2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
* \file bollie-delay.c
* \author Bollie
* \date 05 Nov 2016
* \brief An LV2 tempo delay plugin with filters and tapping.
*/

#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "bolliefilter.h"

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

#define URI "https://ca9.eu/lv2/bolliedelay"

#define MAX_TAPE_LEN 1920001


/**
* Make a bool type available. ;)
*/
typedef enum { false, true } bool;


/**
* Enumeration of LV2 ports
*/
typedef enum {
    BDL_TEMPO_BPM   = 0,
    BDL_TAP         = 1,
    BDL_MIX         = 2,
    BDL_DECAY       = 3,
    BDL_CROSSF      = 4,
    BDL_LOW_ON      = 5,
    BDL_LOW_F       = 6,
    BDL_LOW_Q       = 7,
    BDL_HIGH_ON      = 8,
    BDL_HIGH_F      = 9,
    BDL_HIGH_Q      = 10,
    BDL_DIV_L       = 11,
    BDL_DIV_R       = 12,
    BDL_INPUT_L     = 13,
    BDL_INPUT_R     = 14,
    BDL_OUTPUT_L    = 15,
    BDL_OUTPUT_R    = 16
} PortIdx;


/**
* Struct for THE BollieDelay instance, the host is going to use.
*/
typedef struct {
    float* tempo;               ///< Tempo in BPM
    const float* tap;           ///< Control port for tapping
    const float* mix;           ///< mix/blend in percentage
    const float* decay;         ///< decay in percentage
    const float* crossf;        ///< crossfeed (0-50) between inputs
    const float* low_on;        ///< LCF: 0=off, 1=on
    const float* low_f;         ///< LCF cut off frequency
    const float* low_q;         ///< LCF quality
    const float* high_on;       ///< HCF: 0=off, 1=on
    const float* high_f;        ///< HCF cuf off frequency
    const float* high_q;        ///< HCF quality
    const float* div_l;         ///< Divider enum "left"(0) input
    const float* div_r;         ///< Divider enum "right"(1) input
    const float* input_l;       ///< input0, left side
    const float* input_r;       ///< input1, right side
    float* output_l;            ///< output1, left side
    float* output_r;            ///< output2, right side
    double rate;                ///< Current sample rate

    float buffer_l[MAX_TAPE_LEN];   ///< delay buffer left
    float buffer_r[MAX_TAPE_LEN];   ///< delay buffer right

    BollieFilter filter_low_l;      ///< LCF left
    BollieFilter filter_low_r;      ///< LCF right
    BollieFilter filter_high_l;     ///< HCF left
    BollieFilter filter_high_r;     ///< HCF right

    int d_samples_l; /**< Storing the max. number of samples for the current 
                            delay time, left */
    int d_samples_r; /**< Storing the max. number of samples for the current 
                            delay time, left */


    float cur_tempo;    ///< state variable for current tempo set by tempo (above)
    float cur_div_l;    ///< state var for current division, left side
    float cur_div_r;    ///< state var for current division, right side
    int wl_pos;         ///< current write position, left side
    int wr_pos;         ///< current write position, right side
    int rl_pos;         ///< current read position, left side
    int rr_pos;         ///< current read position, right side
    int start_tap;      ///< when did the last tap happen (ms since epoch)
} BollieDelay;


/**
* Instantiates the plugin
* Allocates memory for the BollieDelay object and returns a pointer as
* LV2Handle.
*/
static LV2_Handle instantiate(const LV2_Descriptor * descriptor, double rate,
    const char* bundle_path, const LV2_Feature* const* features) {
    
    BollieDelay *self = (BollieDelay*)calloc(1, sizeof(BollieDelay));

    // Memorize sample rate for calculation
    self->rate = rate;

    return (LV2_Handle)self;
}


/**
* Used by the host to connect the ports of this plugin.
* \param instance current LV2_Handle (will be cast to BollieDelay*)
* \param port LV2 port index, maches the enum above.
* \param data Pointer to the actual port data.
*/
static void connect_port(LV2_Handle instance, uint32_t port, void *data) {
    BollieDelay *self = (BollieDelay*)instance;

    switch ((PortIdx)port) {
        case BDL_TEMPO_BPM:
            self->tempo = data;
            break;
        case BDL_TAP:
            self->tap = data;
            break;
        case BDL_MIX:
            self->mix = data;
            break;
        case BDL_DECAY:
            self->decay = data;
            break;
        case BDL_CROSSF:
            self->crossf = data;
            break;
        case BDL_LOW_ON:
            self->low_on = data;
            break;
        case BDL_LOW_F:
            self->low_f = data;
            break;
        case BDL_LOW_Q:
            self->low_q = data;
            break;
        case BDL_HIGH_ON:
            self->high_on = data;
            break;
        case BDL_HIGH_F:
            self->high_f = data;
            break;
        case BDL_HIGH_Q:
            self->high_q = data;
            break;
        case BDL_DIV_L:
            self->div_l = data;
            break;
        case BDL_DIV_R:
            self->div_r = data;
            break;
        case BDL_INPUT_L:
            self->input_l = data;
            break;
        case BDL_INPUT_R:
            self->input_r = data;
            break;
        case BDL_OUTPUT_L:
            self->output_l = data;
            break;
        case BDL_OUTPUT_R:
            self->output_r = data;
            break;
    }
}
    

/**
* This has to reset all the internal states of the plugin
* \param instance pointer to current plugin instance
*/
static void activate(LV2_Handle instance) {
    BollieDelay* self = (BollieDelay*)instance;
    // Let's remove all that noise
    for (int i = 0 ; i < MAX_TAPE_LEN ; ++i) {
        self->buffer_l[i] = 0;
        self->buffer_r[i] = 0;
    }

    // Initialize number of samples needed
    self->d_samples_l = 0;
    self->d_samples_r = 0;

    // Clear the filters
    bf_reset(&self->filter_low_l);
    bf_reset(&self->filter_low_r);
    bf_reset(&self->filter_high_l);
    bf_reset(&self->filter_high_r);

    // Reset the positions & state variables
    self->wl_pos = 0;
    self->wr_pos = 0;
    self->rl_pos = 0;
    self->rr_pos = 0;
    self->cur_tempo = 0;
    self->cur_div_l = 0;
    self->cur_div_r = 0;

    // Reset tapping
    self->start_tap = 0;
}


/**
* Handles a tap on the tap button and calculates time differences
* \param self pointer to current plugin instance
* \return Beats per minute
*/
static float handle_tap(BollieDelay* self) {

    float d = 0;

    struct timeval t_cur;
    gettimeofday(&t_cur, 0);

    // convert it to milliseconds
    long int t_cur_ms = floor(t_cur.tv_sec * 1000 + t_cur.tv_usec / 1000);

    // If start tep is memorized, do some calculations
    if (self->start_tap) {
        d = t_cur_ms - self->start_tap;

        // Reset if we exceed the maximum delay time
        if (d <= 50 || d > 10000 ) {
            d = 0;
        }
    }
    self->start_tap = t_cur_ms;
    return 60000 / d;   // convert to bpm
}


/**
* Calculates number of samples used for divided delay times.
* \param self pointer to current plugin instance.
* \param div  divider
* \return number of samples needed for the delay buffer
* \todo divider enum
*/
static int calc_delay_samples(BollieDelay* self, int div) {
    // Calculate the samples needed 
    float d = 60 / *self->tempo * self->rate;
    switch(div) {
        case 1:
        d = d * 2/3;
            break;
        case 2:
        d = d / 2;
            break;
        case 3:
        d = d / 4 * 3;
            break;
        case 4:
        d = d / 3;
            break;
        case 5:
            d = d / 4;
            break;
    }
    return floor(d);
}


/**
* Main process function of the plugin.
* \param instance  handle of the current plugin
* \param n_samples number of samples in this current input block.
*/
static void run(LV2_Handle instance, uint32_t n_samples) {
    BollieDelay* self = (BollieDelay*)instance;

    // First some TAP handling
    if (*(self->tap) > 0) {
        long int d = handle_tap(self);
        if (d > 0) 
            *self->tempo = (float)d;
    }

    // Calculate the number of samples for the currently set delay time, if 
    // parameters change
    if (*self->tempo != self->cur_tempo ||
        *self->div_l != self->cur_div_l ||
        *self->div_r != self->cur_div_r
    ) {
        self->d_samples_l = calc_delay_samples(self, *self->div_l);
        self->d_samples_r = calc_delay_samples(self, *self->div_r);
        self->cur_tempo = *self->tempo;
        self->cur_div_l = *self->div_l;
        self->cur_div_r = *self->div_r;

        /* The buffer always needs to be one sample bigger than the delay time.
        In order to not exceed MAX_TAPE_LEN, cut the number of samples, if 
        needed */
        if (self->d_samples_l+1 > MAX_TAPE_LEN)
            self->d_samples_l = MAX_TAPE_LEN-1;

        if (self->d_samples_r+1 > MAX_TAPE_LEN)
            self->d_samples_r = MAX_TAPE_LEN-1;
    }

    // Loop over the block of audio we got
    for (unsigned int i = 0 ; i < n_samples ; i++) {
        // Derive new position
        self->rl_pos = self->wl_pos - self->d_samples_l;
        self->rr_pos = self->wr_pos - self->d_samples_r;

        // Derive the read position and rewind if neccessary
        if (self->rl_pos < 0)
            self->rl_pos = self->d_samples_l+1 + self->rl_pos;

        if (self->rr_pos < 0)
            self->rr_pos = self->d_samples_r+1 + self->rr_pos;
        
        // Old samples
        float old_s_l = self->buffer_l[self->rl_pos];
        float old_s_r = self->buffer_r[self->rr_pos];

        // Current samples
        float cur_fs_l = self->input_l[i];
        float cur_fs_r = self->input_r[i];

        // Apply the low cut filter if enabled
        if (*self->low_on) {
            cur_fs_l = bf_lcf(
                cur_fs_l, 
                *self->low_f, 
                *self->low_q, 
                self->rate, 
                &self->filter_low_l
            );
            cur_fs_r = bf_lcf(
                cur_fs_r, 
                *self->low_f, 
                *self->low_q, 
                self->rate, 
                &self->filter_low_r
            );
        }
 
        // Apply the high cut filter if enabled
        if (*self->high_on) {
            cur_fs_l = bf_hcf(
                cur_fs_l, 
                *self->high_f, 
                *self->high_q, 
                self->rate, 
                &self->filter_high_l
            );
            cur_fs_r = bf_hcf(
                cur_fs_r, 
                *self->high_f, 
                *self->high_q, 
                self->rate, 
                &self->filter_high_r
            );
        }
 

        // Left Channel
        self->buffer_l[self->wl_pos] = 
            cur_fs_l                            // current filtered sample
            + old_s_r * *self->crossf / 100     // mix with crossfeed
            + old_s_l * *self->decay / 100;     // mix with decayed old sample

        // Right channel (s. above)
        self->buffer_r[self->wr_pos] = 
            cur_fs_r
            + old_s_l * *self->crossf / 100
            + old_s_r * *self->decay / 100;

        // Now copy samples from read pos of the buffer to the output buffer
        
        // Left channel
        self->output_l[i] = self->buffer_l[self->rl_pos] * *(self->mix) / 100 
            + self->input_l[i] * (100 - *self->mix) / 100;

        // Same for right channel
        self->output_r[i] = self->buffer_r[self->rr_pos] * *(self->mix) / 100 
            + self->input_r[i] * (100 - *self->mix) / 100;

        // Iterate write position
        self->wl_pos 
            = (self->wl_pos+1 >= self->d_samples_l+1 ? 0 : self->wl_pos+1);
        self->wr_pos 
            = (self->wr_pos+1 >= self->d_samples_r+1 ? 0 : self->wr_pos+1);
    }
}


/**
* Called, when the host deactivates the plugin.
*/
static void deactivate(LV2_Handle instance) {
}


/**
* Cleanup, freeing memory and stuff
*/
static void cleanup(LV2_Handle instance) {
    free(instance);
}


/**
* extension stuff for additional interfaces
*/
static const void* extension_data(const char* uri) {
    return NULL;
}


/**
* Descriptor linking our methods.
*/
static const LV2_Descriptor descriptor = {
    URI,
    instantiate,
    connect_port,
    activate,
    run,
    deactivate,
    cleanup,
    extension_data
};


/**
* Symbol export using the descriptor above
*/
LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    switch (index) {
        case 0:  return &descriptor;
        default: return NULL;
    }
}
