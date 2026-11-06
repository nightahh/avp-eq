#include "avp-eq-dsp.h"

#include <math.h>

#define AVP_PI 3.14159265358979323846

static double clampd(double v, double lo, double hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

void avp_design_band(struct avp_coef *out, const struct avp_band *band, double fs)
{
	double b0, b1, b2, a0, a1, a2;

	out->on = 0;
	out->b0 = 1.0;
	out->b1 = out->b2 = out->a1 = out->a2 = 0.0;

	if (band->type <= AVP_BAND_OFF || band->type > AVP_BAND_NOTCH || fs < 8000.0)
		return;

	double f = clampd(band->freq, 10.0, fs * 0.45);
	double q = clampd(band->q, 0.1, 30.0);
	double g = clampd(band->gain_db, -24.0, 24.0);
	double w0 = 2.0 * AVP_PI * f / fs;
	double cw = cos(w0), sw = sin(w0);
	double alpha = sw / (2.0 * q);
	double A = pow(10.0, g / 40.0);

	switch (band->type) {
	case AVP_BAND_HIGHPASS:
		b0 = (1.0 + cw) / 2.0;
		b1 = -(1.0 + cw);
		b2 = b0;
		a0 = 1.0 + alpha;
		a1 = -2.0 * cw;
		a2 = 1.0 - alpha;
		break;
	case AVP_BAND_LOWPASS:
		b0 = (1.0 - cw) / 2.0;
		b1 = 1.0 - cw;
		b2 = b0;
		a0 = 1.0 + alpha;
		a1 = -2.0 * cw;
		a2 = 1.0 - alpha;
		break;
	case AVP_BAND_PEAK:
		b0 = 1.0 + alpha * A;
		b1 = -2.0 * cw;
		b2 = 1.0 - alpha * A;
		a0 = 1.0 + alpha / A;
		a1 = -2.0 * cw;
		a2 = 1.0 - alpha / A;
		break;
	case AVP_BAND_NOTCH:
		b0 = 1.0;
		b1 = -2.0 * cw;
		b2 = 1.0;
		a0 = 1.0 + alpha;
		a1 = -2.0 * cw;
		a2 = 1.0 - alpha;
		break;
	case AVP_BAND_LOWSHELF: {
		double beta = 2.0 * sqrt(A) * alpha;
		b0 = A * ((A + 1.0) - (A - 1.0) * cw + beta);
		b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cw);
		b2 = A * ((A + 1.0) - (A - 1.0) * cw - beta);
		a0 = (A + 1.0) + (A - 1.0) * cw + beta;
		a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cw);
		a2 = (A + 1.0) + (A - 1.0) * cw - beta;
		break;
	}
	default: { /* AVP_BAND_HIGHSHELF */
		double beta = 2.0 * sqrt(A) * alpha;
		b0 = A * ((A + 1.0) + (A - 1.0) * cw + beta);
		b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cw);
		b2 = A * ((A + 1.0) + (A - 1.0) * cw - beta);
		a0 = (A + 1.0) - (A - 1.0) * cw + beta;
		a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cw);
		a2 = (A + 1.0) - (A - 1.0) * cw - beta;
		break;
	}
	}

	out->on = 1;
	out->b0 = b0 / a0;
	out->b1 = b1 / a0;
	out->b2 = b2 / a0;
	out->a1 = a1 / a0;
	out->a2 = a2 / a0;
}

void avp_eq_process(const struct avp_coef coefs[AVP_EQ_BANDS], struct avp_eq_state *st, int channel, float *x,
		    size_t n, double out_gain)
{
	if (channel < 0 || channel >= AVP_EQ_MAX_CH)
		return;

	for (int b = 0; b < AVP_EQ_BANDS; b++) {
		const struct avp_coef *c = &coefs[b];
		if (!c->on)
			continue;
		double z1 = st->z1[channel][b], z2 = st->z2[channel][b];
		for (size_t i = 0; i < n; i++) {
			double in = (double)x[i];
			double out = c->b0 * in + z1;
			z1 = c->b1 * in - c->a1 * out + z2;
			z2 = c->b2 * in - c->a2 * out;
			x[i] = (float)out;
		}
		/* flush denormals so a long silence cannot slow the audio thread down */
		if (fabs(z1) < 1e-30)
			z1 = 0.0;
		if (fabs(z2) < 1e-30)
			z2 = 0.0;
		st->z1[channel][b] = z1;
		st->z2[channel][b] = z2;
	}

	if (out_gain != 1.0)
		for (size_t i = 0; i < n; i++)
			x[i] = (float)((double)x[i] * out_gain);
}

double avp_eq_magnitude(const struct avp_coef coefs[AVP_EQ_BANDS], double f, double fs)
{
	double w = 2.0 * AVP_PI * f / fs;
	double cw = cos(w), sw = sin(w), c2 = cos(2.0 * w), s2 = sin(2.0 * w);
	double mag = 1.0;
	for (int b = 0; b < AVP_EQ_BANDS; b++) {
		const struct avp_coef *c = &coefs[b];
		if (!c->on)
			continue;
		double nr = c->b0 + c->b1 * cw + c->b2 * c2, ni = -(c->b1 * sw + c->b2 * s2);
		double dr = 1.0 + c->a1 * cw + c->a2 * c2, di = -(c->a1 * sw + c->a2 * s2);
		mag *= sqrt((nr * nr + ni * ni) / (dr * dr + di * di));
	}
	return mag;
}
