/* AVP EQ - an 8-band parametric equalizer as an OBS audio filter.
 * Add it from a source's Filters dialog: Audio Filters > "AVP EQ".
 * Auto Voice Profile fills in the bands for you, but they can be edited by hand. */

#include <obs-module.h>
#include <media-io/audio-io.h>
#include <util/threading.h>
#include <util/bmem.h>
#include <plugin-support.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "avp-eq-dsp.h"

struct avp_eq_filter {
	obs_source_t *context;
	pthread_mutex_t lock;
	uint32_t sample_rate;
	size_t channels;
	double out_gain;
	struct avp_coef coefs[AVP_EQ_BANDS];
	struct avp_eq_state state;
};

static const double default_freq[AVP_EQ_BANDS] = {80.0, 120.0, 280.0, 500.0, 1500.0, 3500.0, 6500.0, 10000.0};

static void make_key(char *buf, size_t n, int band, const char *what)
{
	snprintf(buf, n, "b%d_%s", band + 1, what);
}

static const char *avp_eq_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return obs_module_text("AVPEQ");
}

static void avp_eq_update(void *data, obs_data_t *settings)
{
	struct avp_eq_filter *f = data;
	struct avp_coef next[AVP_EQ_BANDS];
	char key[24];

	audio_t *audio = obs_get_audio();
	uint32_t rate = audio ? audio_output_get_sample_rate(audio) : 48000;
	size_t channels = audio ? audio_output_get_channels(audio) : 2;
	if (channels > AVP_EQ_MAX_CH)
		channels = AVP_EQ_MAX_CH;

	for (int b = 0; b < AVP_EQ_BANDS; b++) {
		struct avp_band band;
		make_key(key, sizeof(key), b, "type");
		band.type = (int)obs_data_get_int(settings, key);
		make_key(key, sizeof(key), b, "freq");
		band.freq = obs_data_get_double(settings, key);
		make_key(key, sizeof(key), b, "gain");
		band.gain_db = obs_data_get_double(settings, key);
		make_key(key, sizeof(key), b, "q");
		band.q = obs_data_get_double(settings, key);
		avp_design_band(&next[b], &band, (double)rate);
	}
	double out_gain = pow(10.0, obs_data_get_double(settings, "out_db") / 20.0);

	pthread_mutex_lock(&f->lock);
	memcpy(f->coefs, next, sizeof(next));
	f->sample_rate = rate;
	f->channels = channels;
	f->out_gain = out_gain;
	pthread_mutex_unlock(&f->lock);
}

static void *avp_eq_create(obs_data_t *settings, obs_source_t *source)
{
	struct avp_eq_filter *f = bzalloc(sizeof(*f));
	f->context = source;
	pthread_mutex_init(&f->lock, NULL);
	f->out_gain = 1.0;
	avp_eq_update(f, settings);
	return f;
}

static void avp_eq_destroy(void *data)
{
	struct avp_eq_filter *f = data;
	pthread_mutex_destroy(&f->lock);
	bfree(f);
}

static void avp_eq_defaults(obs_data_t *settings)
{
	char key[24];
	obs_data_set_default_double(settings, "out_db", 0.0);
	for (int b = 0; b < AVP_EQ_BANDS; b++) {
		make_key(key, sizeof(key), b, "type");
		obs_data_set_default_int(settings, key, b == 0 ? AVP_BAND_HIGHPASS : AVP_BAND_OFF);
		make_key(key, sizeof(key), b, "freq");
		obs_data_set_default_double(settings, key, default_freq[b]);
		make_key(key, sizeof(key), b, "gain");
		obs_data_set_default_double(settings, key, 0.0);
		make_key(key, sizeof(key), b, "q");
		obs_data_set_default_double(settings, key, b == 0 ? 0.707 : 1.0);
	}
}

static obs_properties_t *avp_eq_properties(void *unused)
{
	UNUSED_PARAMETER(unused);
	obs_properties_t *props = obs_properties_create();
	char key[24], label[32], gkey[24];

	obs_properties_add_float_slider(props, "out_db", obs_module_text("OutputGain"), -24.0, 24.0, 0.1);

	for (int b = 0; b < AVP_EQ_BANDS; b++) {
		obs_properties_t *grp = obs_properties_create();

		make_key(key, sizeof(key), b, "type");
		obs_property_t *p = obs_properties_add_list(grp, key, obs_module_text("Type"), OBS_COMBO_TYPE_LIST,
							    OBS_COMBO_FORMAT_INT);
		obs_property_list_add_int(p, obs_module_text("TypeOff"), AVP_BAND_OFF);
		obs_property_list_add_int(p, obs_module_text("TypeHighpass"), AVP_BAND_HIGHPASS);
		obs_property_list_add_int(p, obs_module_text("TypeLowpass"), AVP_BAND_LOWPASS);
		obs_property_list_add_int(p, obs_module_text("TypePeak"), AVP_BAND_PEAK);
		obs_property_list_add_int(p, obs_module_text("TypeLowshelf"), AVP_BAND_LOWSHELF);
		obs_property_list_add_int(p, obs_module_text("TypeHighshelf"), AVP_BAND_HIGHSHELF);
		obs_property_list_add_int(p, obs_module_text("TypeNotch"), AVP_BAND_NOTCH);

		make_key(key, sizeof(key), b, "freq");
		obs_properties_add_float_slider(grp, key, obs_module_text("Frequency"), 20.0, 20000.0, 1.0);
		make_key(key, sizeof(key), b, "gain");
		obs_properties_add_float_slider(grp, key, obs_module_text("Gain"), -24.0, 24.0, 0.1);
		make_key(key, sizeof(key), b, "q");
		obs_properties_add_float_slider(grp, key, obs_module_text("Q"), 0.1, 30.0, 0.05);

		snprintf(gkey, sizeof(gkey), "grp%d", b + 1);
		snprintf(label, sizeof(label), "%s %d", obs_module_text("Band"), b + 1);
		obs_properties_add_group(props, gkey, label, OBS_GROUP_NORMAL, grp);
	}
	return props;
}

static struct obs_audio_data *avp_eq_filter_audio(void *data, struct obs_audio_data *audio)
{
	struct avp_eq_filter *f = data;

	/* never block the audio thread: if the settings are being changed, skip one block */
	if (pthread_mutex_trylock(&f->lock) != 0)
		return audio;

	for (size_t ch = 0; ch < f->channels && ch < MAX_AV_PLANES; ch++) {
		if (!audio->data[ch])
			continue;
		avp_eq_process(f->coefs, &f->state, (int)ch, (float *)audio->data[ch], audio->frames, f->out_gain);
	}

	pthread_mutex_unlock(&f->lock);
	return audio;
}

struct obs_source_info avp_eq_filter_info = {
	.id = "avp_eq_filter",
	.type = OBS_SOURCE_TYPE_FILTER,
	.output_flags = OBS_SOURCE_AUDIO,
	.get_name = avp_eq_name,
	.create = avp_eq_create,
	.destroy = avp_eq_destroy,
	.update = avp_eq_update,
	.get_defaults = avp_eq_defaults,
	.get_properties = avp_eq_properties,
	.filter_audio = avp_eq_filter_audio,
};
