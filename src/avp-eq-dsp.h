/* AVP EQ - pure DSP core (no OBS dependencies, so it can be unit-tested on its own).
 * Eight RBJ-cookbook biquad bands in series plus an output gain. */
#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AVP_EQ_BANDS 8
#define AVP_EQ_MAX_CH 8

enum avp_band_type {
	AVP_BAND_OFF = 0,
	AVP_BAND_HIGHPASS = 1,
	AVP_BAND_LOWPASS = 2,
	AVP_BAND_PEAK = 3,
	AVP_BAND_LOWSHELF = 4,
	AVP_BAND_HIGHSHELF = 5,
	AVP_BAND_NOTCH = 6,
};

struct avp_band {
	int type;
	double freq;
	double gain_db;
	double q;
};

struct avp_coef {
	int on;
	double b0, b1, b2, a1, a2; /* normalised so that a0 == 1 */
};

struct avp_eq_state {
	double z1[AVP_EQ_MAX_CH][AVP_EQ_BANDS];
	double z2[AVP_EQ_MAX_CH][AVP_EQ_BANDS];
};

/* Design one band for the sample rate fs. Out-of-range values are clamped. */
void avp_design_band(struct avp_coef *out, const struct avp_band *band, double fs);

/* Filter n samples of one channel in place through all active bands, then apply out_gain (linear). */
void avp_eq_process(const struct avp_coef coefs[AVP_EQ_BANDS], struct avp_eq_state *st, int channel, float *x,
		    size_t n, double out_gain);

/* Magnitude (linear) of the whole cascade at frequency f; used by the tests. */
double avp_eq_magnitude(const struct avp_coef coefs[AVP_EQ_BANDS], double f, double fs);

#ifdef __cplusplus
}
#endif
