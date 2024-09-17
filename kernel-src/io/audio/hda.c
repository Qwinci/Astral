#include <kernel/pci.h>
#include <logging.h>
#include <kernel/hda.h>
#include <kernel/alloc.h>
#include <kernel/pmm.h>
#include <kernel/oss.h>
#include <uhda/uhda.h>

typedef struct {
	uint16_t vendor;
	uint16_t device;
} uhda_pci_device_id_t;

static const uhda_pci_device_id_t UHDA_PCI_IDS[] = {UHDA_MATCHING_DEVICES};

static const UhdaOutput *find_headphone_output(const UhdaOutputGroup *output_group) {
	const UhdaOutput * const* outputs;
	size_t output_count;
	uhda_output_group_get_outputs(output_group, &outputs, &output_count);

	for (size_t i = 0; i < output_count; ++i) {
		const UhdaOutput *output = outputs[i];

		UhdaOutputInfo info = uhda_output_get_info(output);
		if (info.type != UHDA_OUTPUT_TYPE_LINE_OUT &&
			info.type != UHDA_OUTPUT_TYPE_HEADPHONE) {
			continue;
		}

		bool presence;
		UhdaStatus status = uhda_output_get_presence(output, &presence);
		if (status == UHDA_STATUS_UNSUPPORTED) {
			return output;
		}
		else {
			__assert(status == UHDA_STATUS_SUCCESS);
			if (presence) {
				return output;
			}
		}
	}

	return NULL;
}

static const UhdaOutput *find_speaker_output(const UhdaOutputGroup *output_group) {
	const UhdaOutput * const* outputs;
	size_t output_count;
	uhda_output_group_get_outputs(output_group, &outputs, &output_count);

	for (size_t i = 0; i < output_count; ++i) {
		const UhdaOutput *output = outputs[i];

		UhdaOutputInfo info = uhda_output_get_info(output);
		if (info.type != UHDA_OUTPUT_TYPE_SPEAKER) {
			continue;
		}

		bool presence;
		UhdaStatus status = uhda_output_get_presence(output, &presence);
		if (status == UHDA_STATUS_UNSUPPORTED) {
			return output;
		}
		else {
			__assert(status == UHDA_STATUS_SUCCESS);
			if (presence) {
				return output;
			}
		}
	}

	return NULL;
}

typedef struct {
	UhdaController *ctrl;
	const UhdaOutput *output;
	UhdaPath *path;
	uint32_t sample_rate;
	uint32_t channels;
	uint32_t bits;
	UhdaFormat fmt;
	ossdesc_t desc;
	semaphore_t sem;
} hda_output_t;

static int hda_set_rate(ossdesc_t *desc, int rate) {
	hda_output_t *output = desc->private;
	output->sample_rate = (uint32_t)rate;
	// todo
	return rate;
}

static int hda_set_channels(ossdesc_t *desc, int channels) {
	hda_output_t *output = desc->private;
	output->channels = (uint32_t)channels;
	// todo
	return channels;
}

static int hda_set_fmt(ossdesc_t *desc, int fmt) {
	hda_output_t *output = desc->private;
	output->fmt = UHDA_FORMAT_PCM16;
	output->bits = 16;
	return AFMT_S16_LE;
}

static void hda_trip_fn(void *arg, uint32_t remaining) {
	hda_output_t *output = arg;
	semaphore_signal_limit(&output->sem, 1);
}

static int hda_queue(ossdesc_t *desc, iovec_iterator_t *iovec_iterator, size_t size, size_t *writep) {
	hda_output_t *output = desc->private;

	UhdaStream** streams;
	size_t stream_count;
	uhda_get_output_streams(output->ctrl, &streams, &stream_count);
	__assert(stream_count);

	UhdaStream *stream = streams[0];

	UhdaStreamStatus stream_status = uhda_stream_get_status(stream);
	if (stream_status == UHDA_STREAM_STATUS_UNINITIALIZED) {
		if (!desc->fragmentcount) {
			desc->fragmentcount = 2;
		}
		if (!desc->fragmentsize) {
			size_t bytes_per_second = output->sample_rate * output->channels * (output->bits / 8);
			size_t bytes_per_ms = bytes_per_second / 1000;
			desc->fragmentsize = (int) bytes_per_ms * 50;
		}

		UhdaStreamParams params = {
			.sample_rate = output->sample_rate,
			.channels = output->channels,
			.fmt = output->fmt
		};

		UhdaStatus status = uhda_path_setup(
			output->path,
			&params,
			stream);
		__assert(status == UHDA_STATUS_SUCCESS);
		status = uhda_path_set_volume(output->path, 50);
		__assert(status == UHDA_STATUS_SUCCESS);

		status = uhda_stream_setup(
			stream,
			&params,
			desc->fragmentcount * desc->fragmentsize,
			NULL,
			NULL,
			desc->fragmentsize,
			hda_trip_fn,
			output);
		__assert(status == UHDA_STATUS_SUCCESS);

		stream_status = UHDA_STREAM_STATUS_PAUSED;
	}

	int ret = 0;

	size_t done = 0;
	while (done < size) {
		uint32_t space;
		uhda_stream_get_remaining(stream, &space);

		if (space == desc->fragmentcount * desc->fragmentsize) {
			int err = semaphore_wait(&output->sem, true);
			if (err == SCHED_WAKEUP_REASON_INTERRUPTED) {
				ret = EINTR;
				break;
			}

			continue;
		}

		void *page;
		size_t page_offset, page_remaining;
		ret = iovec_iterator_next_page(iovec_iterator, &page_offset, &page_remaining, &page);
		if (ret) {
			break;
		}

		__assert(page);

		uint32_t to_copy = desc->fragmentcount * desc->fragmentsize - space;
		if (to_copy > page_remaining) {
			to_copy = page_remaining;
		}

		uhda_stream_queue_data(stream, (void *)((uintptr_t)MAKE_HHDM(page) + page_offset), &to_copy);

		pmm_release(page);

		// if we didn't use the whole space in the page, set the iterator back a bit
		size_t diff_between_available_and_used = page_remaining - to_copy;
		if (diff_between_available_and_used) {
			size_t iterator_offset = iovec_iterator_total_offset(iovec_iterator);
			iovec_iterator_set(iovec_iterator, iterator_offset - diff_between_available_and_used);
		}

		done += to_copy;

		space += to_copy;
		if (stream_status == UHDA_STREAM_STATUS_PAUSED &&
			space >= output->desc.fragmentsize) {
			printf("hda: starting stream\n");

			uhda_stream_play(stream, true);
			stream_status = UHDA_STREAM_STATUS_RUNNING;
		}
	}

	if (ret) {
		return ret;
	}

	*writep = size;
	return 0;
}

static int hda_pause(ossdesc_t *desc, bool waituntilempty, bool reset) {
	hda_output_t *output = desc->private;

	UhdaStream** streams;
	size_t stream_count;
	uhda_get_output_streams(output->ctrl, &streams, &stream_count);
	__assert(stream_count);

	UhdaStream *stream = streams[0];

	UhdaStreamStatus stream_status = uhda_stream_get_status(stream);
	if (waituntilempty && stream_status == UHDA_STREAM_STATUS_RUNNING) {
		while (true) {
			uint32_t remaining;
			uhda_stream_get_remaining(stream, &remaining);
			if (!remaining) {
				break;
			}

			int err = semaphore_wait(&output->sem, 1);
			if (err == SCHED_WAKEUP_REASON_INTERRUPTED) {
				break;
			}
		}
	}

	uhda_stream_play(stream, false);

	if (reset) {
		uhda_stream_shutdown(stream);
		uhda_path_shutdown(output->path);
	}

	return 0;
}

static int hda_get_buffer_avail(ossdesc_t *desc, int *ret) {
	hda_output_t *output = desc->private;

	UhdaStream** streams;
	size_t stream_count;
	uhda_get_output_streams(output->ctrl, &streams, &stream_count);
	__assert(stream_count);

	UhdaStream *stream = streams[0];

	int total = desc->fragmentcount * desc->fragmentsize;

	uint32_t remaining;
	uhda_stream_get_remaining(stream, &remaining);

	*ret = (int) (total - remaining);
	return 0;
}

static int hda_open(ossdesc_t *desc) {
	// todo exclusive lock shared between outputs
	return 0;
}

static int hda_close(ossdesc_t *desc) {
	return 0;
}

static void hda_init_output_group(UhdaController *ctrl, const UhdaOutputGroup *output_group) {
	const UhdaOutput *primary_output = find_headphone_output(output_group);
	if (!primary_output) {
		primary_output = find_speaker_output(output_group);
	}

	if (!primary_output) {
		printf("hda: couldn't find primary output, ignoring output group\n");
		return;
	}

	hda_output_t *output = alloc(sizeof(hda_output_t));
	__assert(output);

	UhdaPath *path;
	UhdaStatus status = uhda_find_path(primary_output, NULL, 0, false, &path);
	__assert(status == UHDA_STATUS_SUCCESS);

	output->ctrl = ctrl;
	output->output = primary_output;
	output->path = path;
	output->sample_rate = 44100;
	output->channels = 2;
	output->fmt = UHDA_FORMAT_PCM16;
	output->bits = 16;
	output->desc = (ossdesc_t) {
		.oformats = AFMT_S16_LE,
		.minrate = 5513,
		.maxrate = 192000,
		.minchannels = 1,
		.maxchannels = 16,
		.nrates = 10,
		.rates = {
			8000,
			11025,
			16000,
			22050,
			44100,
			48000,
			88200,
			96000,
			176400,
			192000
		},
		.private = output,
		.setrate = hda_set_rate,
		.setchannels = hda_set_channels,
		.setfmt = hda_set_fmt,
		.queue = hda_queue,
		.pause = hda_pause,
		.resume = NULL,
		.getbufferavail = hda_get_buffer_avail,
		.open = hda_open,
		.close = hda_close
	};
	SEMAPHORE_INIT(&output->sem, 1);

	oss_register(&output->desc);
}

static void hda_init_codec(UhdaController *ctrl, const UhdaCodec *codec) {
	const UhdaOutputGroup * const*output_groups;
	size_t output_group_count;
	uhda_codec_get_output_groups(codec, &output_groups, &output_group_count);

	const UhdaOutputGroup *primary_group = NULL;
	for (size_t i = 0; i < output_group_count; ++i) {
		if (find_headphone_output(output_groups[i])) {
			hda_init_output_group(ctrl, output_groups[i]);
			primary_group = output_groups[i];
			break;
		}
	}

	for (size_t i = 0; i < output_group_count; ++i) {
		if (primary_group && output_groups[i] == primary_group) {
			continue;
		}

		hda_init_output_group(ctrl, output_groups[i]);
	}
}

static void hda_init_controller(pcienum_t *e) {
	printf("hda: initializing for %04x:%04x\n", e->vendor, e->device);

	UhdaController *ctrl;
	UhdaStatus status = uhda_init(e, &ctrl);
	__assert(status == UHDA_STATUS_SUCCESS);

	const UhdaCodec * const*codecs;
	size_t codec_count;
	uhda_get_codecs(ctrl, &codecs, &codec_count);

	for (size_t i = 0; i < codec_count; ++i) {
		hda_init_codec(ctrl, codecs[i]);
	}
}

void hda_init() {
	int i = 0;
	for (;;) {
		pcienum_t *e = pci_getenum(UHDA_MATCHING_CLASS, UHDA_MATCHING_SUBCLASS, -1, -1, -1, -1, i++);
		if (e == NULL) {
			break;
		}

		hda_init_controller(e);
	}

	i = 0;
	for (;;) {
		pcienum_t *e;

		for (size_t j = 0; j < sizeof(UHDA_PCI_IDS) / sizeof(*UHDA_PCI_IDS); ++j) {
			e = pci_getenum(-1, -1, -1, UHDA_PCI_IDS[j].vendor, UHDA_PCI_IDS[j].device, -1, i++);
			if (e) {
				break;
			}
		}

		if (e == NULL) {
			break;
		}

		hda_init_controller(e);
	}
}
