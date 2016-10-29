#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"

#define URI "https://ca9.eu/lv2/bolliedelay"

#define MAX_TAPE_LEN 1920001

typedef enum {
    BDL_DELAY       = 0,
    BDL_TAP         = 1,
    BDL_MIX         = 2,
    BDL_DECAY       = 3,
    BDL_CROSSF      = 4,
    BDL_INPUT_L     = 5,
    BDL_INPUT_R     = 6,
    BDL_OUTPUT_L    = 7,
    BDL_OUTPUT_R    = 8
} PortIdx;

typedef struct {
    float* delay;
    const float* tap;
    const float* mix;
    const float* decay;
    const float* crossf;
    const float* input_l;
    const float* input_r;
    float* output_l;
    float* output_r;
    float buffer_l[MAX_TAPE_LEN];
    float buffer_r[MAX_TAPE_LEN];
    double rate;
    int wl_pos;
    int wr_pos;
    int rl_pos;
    int rr_pos;
    long int start_tap;
} BollieDelay;

/**
* Instantiates the plugin
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
*/
static void connect_port(LV2_Handle instance, uint32_t port, void *data) {
    BollieDelay *self = (BollieDelay*)instance;

    switch ((PortIdx)port) {
        case BDL_DELAY:
            self->delay = data;
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
        case BDL_INPUT_L:
            self->input_l = data;
            break;
        case BDL_INPUT_R:
            self->input_r = data;
            break;
        case BDL_OUTPUT_L:
            self->output_r = data;
            break;
        case BDL_OUTPUT_R:
            self->output_l = data;
            break;
    }
}
    

/**
* This has to reset all the internal states of the plugin
*/
static void activate(LV2_Handle instance) {
    BollieDelay* self = (BollieDelay*)instance;
    // Let's remove all that noise
    for (int i = 0 ; i < MAX_TAPE_LEN ; ++i) {
        self->buffer_l[i] = 0;
        self->buffer_r[i] = 0;
    }

    // Reset the positions
    self->wl_pos = 0;
    self->wr_pos = 0;
    self->rl_pos = 0;
    self->rr_pos = 0;

    // Reset tapping
    self->start_tap = 0;
}


/**
* Handles a tap on the tap button and calculates time differences
*/
static long int handle_tap(BollieDelay* self) {

    long int d = 0;

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

    return d;
}


/**
* Main process function of the plugin.
*/
static void run(LV2_Handle instance, uint32_t n_samples) {
    BollieDelay* self = (BollieDelay*)instance;

    // First some TAP handling
    if (*(self->tap) > 0) {
        long int d = handle_tap(self);
        if (d > 0) 
            *self->delay = (float)d;
    }

    // Calculate the number of samples for the currently set delay time
    int delay_samples = floor(*self->delay / 1000 * self->rate);
    //delay_samples = 48500;
    if (delay_samples > MAX_TAPE_LEN)
        delay_samples = MAX_TAPE_LEN;

    // Loop over the block of audio we got
    for(int i = 0 ; i < n_samples ; i++) {

        // Derive the read position
        self->rl_pos = self->wl_pos - delay_samples;
        self->rr_pos = self->wr_pos - delay_samples;
        
        // Rewind if neccessary
        if (self->rl_pos < 0)
            self->rl_pos = delay_samples + self->rl_pos;
        if (self->rr_pos < 0)
            self->rr_pos = delay_samples + self->rr_pos;

        // Low Pass
        float cur_fr_l = self->input_l[i];
        float cur_fr_r = self->input_r[i];
        float a = 0.5;

        if (i>0) {
            cur_fr_l = a * cur_fr_l + (1-a) * self->input_l[i-1];
            cur_fr_r = a * cur_fr_r + (1-a) * self->input_r[i-1];
        }
            
        // Copy input to buffer and mix it with the previous content
        float old_fr_l = self->buffer_l[self->wl_pos]; // previous frame left
        float old_fr_r = self->buffer_r[self->wr_pos]; // previous frame right
        self->buffer_l[self->wl_pos] = 
            self->input_l[i] 
            + old_fr_r * *self->crossf / 100
            + old_fr_l * *self->decay / 100;

        self->buffer_r[self->wr_pos] = 
            self->input_r[i] 
            + old_fr_l * *self->crossf / 100
            + old_fr_r * *self->decay / 100;

        self->output_l[i] = self->buffer_l[self->rl_pos] * *(self->mix) / 100 
            + self->input_l[i] * (100 - *self->mix) / 100;

        self->output_r[i] = self->buffer_r[self->rr_pos] * *(self->mix) / 100 
            + self->input_r[i] * (100 - *self->mix) / 100;

        // Iterate write position
        self->wl_pos = (self->wl_pos+1 >= delay_samples ? 0 : self->wl_pos+1);
        self->wr_pos = (self->wr_pos+1 >= delay_samples ? 0 : self->wr_pos+1);
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

LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    switch (index) {
        case 0:  return &descriptor;
        default: return NULL;
    }
}
