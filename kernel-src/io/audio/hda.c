#include <kernel/pci.h>
#include <logging.h>
#include <kernel/hda.h>
#include <kernel/alloc.h>
#include <kernel/pmm.h>
#include <kernel/oss.h>
#include <arch/e9.h>
#include <ringbuffer.h>
#include <semaphore.h>

#define CHUNK_SIZE 0x1000

#define GCAP_64OK(cap) ((cap) & 1)
#define GCAP_BSS(cap) (((cap) >> 3) & 0x1f)
#define GCAP_ISS(cap) (((cap) >> 8) & 0xf)
#define GCAP_OSS(cap) (((cap) >> 12) & 0xf)

#define GCTL_CRST(ctl) ((ctl) & 1)
#define GCTL_SETCRST(ctl, rst) ctl = ((ctl) & ~1) | (rst)

#define INTCTL_SETGIE(ctl, value) ctl = ((ctl) & ~(1 << 31)) | ((value) << 31)
#define INTCTL_SETCIE(ctl, value) ctl = ((ctl) & ~(1 << 30)) | ((value) << 30)
#define INTCTL_SETSIE(ctl, num, value) ctl = ((ctl) & ~(1 << (num))) | ((value) << (num))

#define INTSTS_CIS(sts) (((sts) >> 30) & 1)
#define INTSTS_SIS(sts, num) (((sts) >> (num)) & 1)

#define CORBWP_SETWP(corbwp, wp) corbwp = ((corbwp) & ~0xff) | (wp)

#define CORBRP_RP(corbrp) ((corbrp) & 0xff)

#define CORBCTL_RUN(ctl) (((ctl) >> 1) & 1)
#define CORBCTL_SETRUN(ctl, run) ctl = ((ctl) & ~(1 << 1)) | ((run) << 1)

#define CORBSIZE_SZCAP(corbsize) (((corbsize) >> 4) & 0xf)
#define CORBSIZE_SETSIZE(corbsize, size) corbsize = ((corbsize) & ~3) | (size)

#define RIRBWP_WP(rirbwp) ((rirbwp) & 0xff)

#define RINTCNT_SETCNT(rintcnt, cnt) rintcnt = ((rintcnt & ~0xff) | (cnt))

#define RIRBCTL_DMAEN(ctl) (((ctl) >> 1) & 1)
#define RIRBCTL_SETRINTCTL(ctl, value) ctl = ((ctl) & ~1) | (value)
#define RIRBCTL_SETDMAEN(ctl, value) ctl = ((ctl) & ~(1 << 1)) | ((value) << 1)

#define RIRBSTS_INTFL(sts) ((sts) & 1)
#define RIRBSTS_CLEARINTFL(sts) sts = (sts) | 1

#define RIRBSIZE_SZCAP(rirbsize) (((rirbsize) >> 4) & 0xf)
#define RIRBSIZE_SETSIZE(rirbsize, size) rirbsize = ((rirbsize) & ~3) | (size)

#define SDCTL0_RST(ctl) ((ctl) & 1)
#define SDCTL0_RUN(ctl) (((ctl) >> 1) & 1)

#define SDCTL0_SETRST(ctl, value) ctl = ((ctl) & ~1) | (value)
#define SDCTL0_SETRUN(ctl, value) ctl = ((ctl) & ~(1 << 1)) | ((value) << 1)
#define SDCTL0_SETIOCE(ctl, value) ctl = ((ctl) & ~(1 << 2)) | ((value) << 2)
#define SDCTL0_SETFEIE(ctl, value) ctl = ((ctl) & ~(1 << 3)) | ((value) << 3)

#define SDCTL2_SETSTRM(ctl, value) ctl = ((ctl) & ~(0xf << 4)) | ((value) << 4)

#define SDSTS_BCIS(sts) (((sts) >> 2) & 1)
#define SDSTS_CLEARBCIS(sts) sts = (sts) | (1 << 2)

#define SDLVI_SETLVI(sdlvi, lvi) sdlvi = ((sdlvi) & ~0xff) | (lvi)

#define FMT_CHAN(channels) ((channels) - 1)
#define FMT_BITS_8 (0 << 4)
#define FMT_BITS_16 (1 << 4)
#define FMT_BITS_20 (2 << 4)
#define FMT_BITS_24 (3 << 4)
#define FMT_BITS_32 (4 << 4)
#define FMT_DIV(div) (((div) - 1) << 8)
#define FMT_MULT(mult) (((mult) - 1) << 11)
#define FMT_BASE_48KHZ (0 << 14)
#define FMT_BASE_441KHZ (1 << 14)

#define VERB_SET_CONVERTER_FMT 2
#define VERB_SET_AMP_GAIN_MUTE 3
#define VERB_SET_PROC_COEF 4
#define VERB_SET_COEF_INDEX 5
#define VERB_GET_PROC_COEF 0xc
#define VERB_SET_CONNECTION 0x701
#define VERB_SET_POWER_STATE 0x705
#define VERB_SET_CONVERTER_CONTROL 0x706
#define VERB_SET_PIN_CONTROL 0x707
#define VERB_SET_EAPD_ENABLE 0x70c
#define VERB_SET_CONVERTER_CHANNELS 0x72d
#define VERB_GET_PARAM 0xf00
#define VERB_GET_CONN_LIST_ENTRY 0xf02
#define VERB_GET_CONFIG_DEFAULT 0xf1c

#define PARAM_VENDOR_ID 0
#define PARAM_SUBORDINATE_NODE_CNT 4
#define PARAM_FUNC_GROUP_TYPE 5
#define PARAM_AUDIO_WIDGET_CAPS 9
#define PARAM_PIN_CAPS 0xc
#define PARAM_INPUT_AMP_CAPS 0xd
#define PARAM_OUTPUT_AMP_CAPS 0x12
#define PARAM_CONN_LIST_LEN 0xe

#define FUNC_GROUP_TYPE_AUDIO 1

#define WIDGET_TYPE_AUDIO_OUT 0
#define WIDGET_TYPE_AUDIO_IN 1
#define WIDGET_TYPE_AUDIO_MIXER 2
#define WIDGET_TYPE_AUDIO_SELECTOR 3
#define WIDGET_TYPE_PIN_COMPLEX 4

#define AMP_SET_GAIN(gain) (gain)
#define AMP_SET_MUTE (1 << 7)
#define AMP_SET_RIGHT (1 << 12)
#define AMP_SET_LEFT (1 << 13)
#define AMP_SET_IN (1 << 14)
#define AMP_SET_OUT (1 << 15)

#define PIN_CONTROL_SET_IN_ENABLE (1 << 5)
#define PIN_CONTROL_SET_OUT_ENABLE (1 << 6)
#define PIN_CONTROL_SET_HPHN (1 << 7)

#define EAPD_ENABLE_EAPD (1 << 1)

typedef struct {
	uint16_t gcap;
	uint8_t vmin;
	uint8_t vmaj;
	uint16_t outpay;
	uint16_t inpay;
	uint32_t gctl;
	uint16_t wakeen;
	uint16_t wakests;
	uint16_t gsts;
	uint8_t rsvd0[6];
	uint16_t outstrmpay;
	uint16_t instrmpay;
	uint32_t rsvd1;
	uint32_t intctl;
	uint32_t intsts;
	uint64_t rsvd2;
	uint32_t walclk;
	uint32_t rsvd3;
	uint32_t ssync;
	uint32_t rsvd4;
	uint32_t corblbase;
	uint32_t corbubase;
	uint16_t corbwp;
	uint16_t corbrp;
	uint8_t corbctl;
	uint8_t corbsts;
	uint8_t corbsize;
	uint8_t rsvd5;
	uint32_t rirblbase;
	uint32_t rirbubase;
	uint16_t rirbwp;
	uint16_t rintcnt;
	uint8_t rirbctl;
	uint8_t rirbsts;
	uint8_t rirbsize;
	uint8_t rsvd6;
	uint32_t icoi;
	uint32_t icii;
	uint16_t icis;
	uint8_t rsvd7[6];
	uint32_t dpiblbase;
	uint32_t dpibubase;
	uint64_t rsvd8;
} __attribute__((packed)) hdabar0_t;

typedef struct {
	uint8_t ctl0;
	uint8_t ctl1;
	uint8_t ctl2;
	uint8_t sts;
	uint32_t lpib;
	uint32_t cbl;
	uint16_t lvi;
	uint16_t rsvd0;
	uint16_t fifod;
	uint16_t fmt;
	uint32_t rsvd1;
	uint32_t bdpl;
	uint32_t bdpu;
} __attribute__((packed)) hdastreamregs_t;

typedef struct {
	uint32_t data;
} hdaverb_t;

typedef struct {
	uint32_t resp;
	uint32_t extended;
} hdaresponse_t;

typedef struct {
	uint64_t address;
	uint32_t length;
	uint32_t ioc;
} hdabdlentry_t;

typedef struct {
	volatile hdastreamregs_t *regs;
	hdabdlentry_t *bdl;
	size_t currentpos;
	ringbuffer_t buffer;
	spinlock_t bufferlock;
	mutex_t lock;
	semaphore_t buffersem;
} hdastream_t;

typedef struct {
	volatile hdabar0_t *bar0;

	void *corbrirbphys;
	volatile hdaverb_t *corb;
	int corbcount;
	uint32_t corbptr;
	volatile hdaresponse_t *rirb;
	int rirbcount;
	int rirbptr;
	int lastprocessedrirb;

	hdastream_t instreams[15];
	int instreamcount;
	hdastream_t outstreams[15];
	int outstreamcount;
	hdastream_t bistreams[30];
	int bistreamcount;

	isr_t *isr;
	spinlock_t lock;
	semaphore_t entrysem;
	thread_t **threads;
} hdacontroller_t;

typedef struct {
	uint8_t nid;
	uint8_t selectorindex;
} hdawidgetconnection_t;

typedef struct {
	hdawidgetconnection_t *connections;
	int connectioncount;
	uint32_t pincaps;
	uint32_t inampcaps;
	uint32_t outampcaps;
	uint32_t config;
	uint8_t type;
	uint8_t nid;
} hdawidget_t;

typedef struct hdacodec_t hdacodec_t;

typedef struct {
	uint32_t samplerate;
	uint8_t bits;
	uint8_t channels;
} hdahwparams_t;

typedef struct {
	hdacodec_t *codec;
	hdawidget_t **widgets;
	int widgetcount;

	ossdesc_t oss;
	hdahwparams_t activeparams;
	bool paramschanged;
	bool issigned;
} hdapath_t;

typedef struct hdacodec_t {
	hdacontroller_t *ctrl;
	uint8_t addr;
	hdawidget_t *widgets;
	uint8_t *pinnids;
	int pincount;
	uint8_t *outnids;
	int outcount;
	hdapath_t *outpaths;
	int outpathcount;

	spinlock_t lock;
	mutex_t exclusivelock;
	hdapath_t *activeoutpath;
} hdacodec_t;

static hdaverb_t makeverb(uint8_t codecaddr, uint8_t nid, uint32_t payload) {
	__assert((payload >> 20) == 0);
	__assert((codecaddr >> 4) == 0);
	uint32_t data = payload | (nid << 20) | (codecaddr << 28);
	return (hdaverb_t){data};
}

static hdaverb_t makeshortverb(uint8_t codecaddr, uint8_t nid, uint16_t id, uint8_t data) {
	__assert((id >> 12) == 0);
	return makeverb(codecaddr, nid, id << 8 | data);
}

static hdaverb_t makelongverb(uint8_t codecaddr, uint8_t nid, uint8_t id, uint16_t data) {
	__assert((id >> 4) == 0);
	return makeverb(codecaddr, nid, id << 16 | data);
}

static hdaresponse_t hdasendverbandwait(hdacontroller_t *ctrl, hdaverb_t verb) {
	bool intstatus = interrupt_set(false);
	semaphore_wait(&ctrl->entrysem, false);

	spinlock_acquire(&ctrl->lock);

	uint8_t rirbptr = ++ctrl->rirbptr;
	ctrl->corb[++ctrl->corbptr] = verb;

	uint16_t wp = ctrl->bar0->corbwp;
	CORBWP_SETWP(wp, ctrl->corbptr);
	ctrl->bar0->corbwp = wp;

	if (ctrl->corbptr == ctrl->corbcount - 1) {
		// hack to make the pointer wrap around to zero for the next verb
		ctrl->corbptr = UINT32_MAX;
	}
	if (ctrl->rirbptr == ctrl->rirbcount - 1) {
		ctrl->rirbptr = UINT32_MAX;
	}

	sched_prepare_sleep(false);
	ctrl->threads[rirbptr] = current_thread();
	spinlock_release(&ctrl->lock);
	sched_yield();

	hdaresponse_t resp = ctrl->rirb[rirbptr];

	semaphore_signal(&ctrl->entrysem);
	interrupt_set(intstatus);
	return resp;
}

static void hdasetconverterfmt(hdacodec_t *codec, uint8_t nid, uint16_t value) {
	hdaverb_t verb = makelongverb(codec->addr, nid, VERB_SET_CONVERTER_FMT, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetampgain(hdacodec_t *codec, uint8_t nid, uint16_t value) {
	hdaverb_t verb = makelongverb(codec->addr, nid, VERB_SET_AMP_GAIN_MUTE, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetcoef(hdacodec_t *codec, uint8_t nid, uint16_t value) {
	hdaverb_t verb = makelongverb(codec->addr, nid, VERB_SET_PROC_COEF, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetcoefindex(hdacodec_t *codec, uint8_t nid, uint16_t value) {
	hdaverb_t verb = makelongverb(codec->addr, nid, VERB_SET_COEF_INDEX, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static uint16_t hdagetcoef(hdacodec_t *codec, uint8_t nid) {
	hdaverb_t verb = makelongverb(codec->addr, nid, VERB_GET_PROC_COEF, 0);
	return hdasendverbandwait(codec->ctrl, verb).resp & 0xffff;
}

static void hdasetconnection(hdacodec_t *codec, uint8_t nid, uint8_t index) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_SET_CONNECTION, index);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetpowerstate(hdacodec_t *codec, uint8_t nid, uint8_t value) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_SET_POWER_STATE, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetconvertercontrol(hdacodec_t *codec, uint8_t nid, uint8_t stream, uint8_t channel) {
	__assert((stream >> 4) == 0);
	__assert((channel & 0xf) == 0);
	uint8_t data = (stream << 4) | channel;
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_SET_CONVERTER_CONTROL, data);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetpincontrol(hdacodec_t *codec, uint8_t nid, uint8_t value) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_SET_PIN_CONTROL, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdaseteapdenable(hdacodec_t *codec, uint8_t nid, uint8_t value) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_SET_EAPD_ENABLE, value);
	hdasendverbandwait(codec->ctrl, verb);
}

static void hdasetconverterchannels(hdacodec_t *codec, uint8_t nid, uint8_t channels) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_SET_CONVERTER_CHANNELS, channels - 1);
	hdasendverbandwait(codec->ctrl, verb);
}

static uint32_t hdagetparameter(hdacodec_t *codec, uint8_t nid, uint8_t id) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_GET_PARAM, id);
	return hdasendverbandwait(codec->ctrl, verb).resp;
}

static uint32_t hdagetconnlistentry(hdacodec_t *codec, uint8_t nid, uint8_t offset) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_GET_CONN_LIST_ENTRY, offset);
	return hdasendverbandwait(codec->ctrl, verb).resp;
}

static uint32_t hdagetconfigdefault(hdacodec_t *codec, uint8_t nid) {
	hdaverb_t verb = makeshortverb(codec->addr, nid, VERB_GET_CONFIG_DEFAULT, 0);
	return hdasendverbandwait(codec->ctrl, verb).resp;
}

typedef struct {
	hdawidget_t *widget;
	uint8_t connindex;
} pathstackentry_t;

typedef struct {
	pathstackentry_t *data;
	int size;
	int cap;
} pathstack_t;

static void pathstackpush(pathstack_t *stack, pathstackentry_t entry) {
	if (stack->size == stack->cap) {
		int newcap;
		if (stack->cap == 0) {
			newcap = 8;
		}
		else {
			newcap = stack->cap + stack->cap / 2;
		}
		pathstackentry_t *data = realloc(stack->data, newcap * sizeof(pathstackentry_t));
		__assert(data);
		stack->data = data;
		stack->cap = newcap;
	}

	stack->data[stack->size++] = entry;
}

static void hdafindoutputpaths(hdacodec_t *codec) {
	pathstack_t stack = {};

	for (int pinindex = 0; pinindex < codec->pincount; pinindex++) {
		hdawidget_t *pin = &codec->widgets[codec->pinnids[pinindex]];
		// if not output capable skip
		if ((pin->pincaps & (1 << 4)) == 0)
			continue;

		// if no connection skip
		if ((pin->pincaps >> 30) == 1)
			continue;

		pathstackpush(&stack, (pathstackentry_t){
			.widget = pin,
			.connindex = 0
		});

		while (stack.size != 0) {
			pathstackentry_t *entry = &stack.data[stack.size - 1];
			if (entry->connindex == entry->widget->connectioncount) {
				stack.size--;
				continue;
			}

			uint8_t nextnid = entry->widget->connections[entry->connindex++].nid;
			hdawidget_t *nextwidget = &codec->widgets[nextnid];
			__assert(nextwidget);

			if (nextwidget->type == WIDGET_TYPE_AUDIO_OUT) {
				hdapath_t path = {};
				path.codec = codec;
				path.widgets = alloc((stack.size + 1) * sizeof(hdawidget_t *));
				__assert(path.widgets);
				path.widgetcount = stack.size + 1;

				for (int i = 0; i < stack.size; i++) {
					path.widgets[i] = stack.data[i].widget;
				}

				path.widgets[stack.size] = nextwidget;

				hdapath_t *outpaths = realloc(
					codec->outpaths,
					(codec->outpathcount + 1) * sizeof(hdapath_t));
				__assert(outpaths);
				codec->outpaths = outpaths;
				codec->outpaths[codec->outpathcount++] = path;

				// only find one path per pin complex for now
				stack.size = 0;
				break;
			}
			else {
				bool circular = false;
				for (int i = 0; i < stack.size; i++) {
					if (stack.data[i].widget == nextwidget) {
						circular = true;
						break;
					}
				}

				if (circular || stack.size >= 20)
					continue;

				pathstackpush(&stack, (pathstackentry_t){
					.widget = nextwidget,
					.connindex = 0
				});
			}
		}
	}

	if (stack.cap) {
		free(stack.data);
	}

	printf("hda: found %u possible outputs\n", codec->outpathcount);
}

static int hdasetrate(ossdesc_t *desc, int rate);
static int hdasetchannels(ossdesc_t *desc, int channels);
static int hdasetfmt(ossdesc_t *desc, int fmt);
static int hdaqueue(ossdesc_t *desc, iovec_iterator_t *iovec_iterator, size_t size, size_t *writep);
static int hdapause(ossdesc_t *desc, bool waituntilempty, bool reset);
static int hdaresume_unlocked(ossdesc_t *desc);
static int hdaresume(ossdesc_t *desc);
static int hdagetbufferavail(ossdesc_t *desc, int *ret);
static int hdaopen(ossdesc_t *desc);
static int hdaclose(ossdesc_t *desc);

static void hdasetactivepath(hdacodec_t *codec, hdapath_t *path);
static void hdasetactiveparams(hdacodec_t *codec, const hdahwparams_t *params);

static uint16_t hdareadcoef(hdacodec_t *codec, uint8_t nid, uint16_t index) {
	hdasetcoefindex(codec, nid, index);
	return hdagetcoef(codec, nid);
}

static void hdawritecoef(hdacodec_t *codec, uint8_t nid, uint16_t index, uint16_t value) {
	hdasetcoefindex(codec, nid, index);
	hdasetcoef(codec, nid, value);
}

static void hdaenumeratecodec(hdacontroller_t *ctrl, uint8_t codecaddr) {
	printf("hda: codec found at address %d\n", codecaddr);

	hdacodec_t *codec = alloc(sizeof(hdacodec_t));
	codec->ctrl = ctrl;
	codec->addr = codecaddr;
	codec->widgets = alloc(0x100 * sizeof(hdawidget_t));
	__assert(codec->widgets);
	codec->pinnids = NULL;
	codec->pincount = 0;
	codec->outnids = NULL;
	codec->outcount = 0;
	codec->outpaths = NULL;
	codec->outpathcount = 0;
	codec->activeoutpath = NULL;
	SPINLOCK_INIT(codec->lock);
	MUTEX_INIT(&codec->exclusivelock);

	uint32_t vendorresp = hdagetparameter(codec, 0, PARAM_VENDOR_ID);
	uint16_t deviceid = vendorresp & 0xffff;
	uint16_t vendorid = (vendorresp >> 16) & 0xffff;

	if (vendorid == 0x10ec && deviceid == 0x0887) {
		printf("hda: applying realtek pll quirk\n");

		uint8_t pllnid = 0x20;
		uint16_t coefindex = 0xa;
		uint16_t coefbit = 10;

		uint16_t old = hdareadcoef(codec, pllnid, coefindex);
		old &= ~(1 << coefbit);
		hdawritecoef(codec, pllnid, coefindex, old);
	}

	uint32_t funcgroupcntresp = hdagetparameter(codec, 0, PARAM_SUBORDINATE_NODE_CNT);
	uint8_t numfuncgroups = funcgroupcntresp & 0xff;
	uint8_t funcgroupstartnid = (funcgroupcntresp >> 16) & 0xff;
	for (int funcgroupi = 0; funcgroupi < numfuncgroups; funcgroupi++) {
		uint8_t funcgroupnid = funcgroupstartnid + funcgroupi;

		uint32_t funcgrouptyperesp = hdagetparameter(codec, funcgroupnid, PARAM_FUNC_GROUP_TYPE);
		uint8_t funcgrouptype = funcgrouptyperesp & 0xff;
		if (funcgrouptype != FUNC_GROUP_TYPE_AUDIO)
			continue;

		uint32_t widgetcntresp = hdagetparameter(codec, funcgroupnid, PARAM_SUBORDINATE_NODE_CNT);
		uint8_t numwidgets = widgetcntresp & 0xff;
		uint8_t widgetstartnid = (widgetcntresp >> 16) & 0xff;

		for (int widgeti = 0; widgeti < numwidgets; widgeti++) {
			uint8_t nid = widgetstartnid + widgeti;

			uint32_t audiocaps = hdagetparameter(codec, nid, PARAM_AUDIO_WIDGET_CAPS);
			uint32_t pincaps = hdagetparameter(codec, nid, PARAM_PIN_CAPS);
			uint32_t inampcaps = hdagetparameter(codec, nid, PARAM_INPUT_AMP_CAPS);
			uint32_t outampcaps = hdagetparameter(codec, nid, PARAM_OUTPUT_AMP_CAPS);
			uint8_t connlistlen = hdagetparameter(codec, nid, PARAM_CONN_LIST_LEN);
			uint32_t config = hdagetconfigdefault(codec, nid);

			uint8_t type = (audiocaps >> 20) & 0xf;
			bool connlistpresent = audiocaps & (1 << 8);

			hdawidgetconnection_t *connections = NULL;
			int connectioncount = 0;

			if (connlistpresent && connlistlen) {
				// long form connections are not supported
				__assert((connlistlen & (1 << 7)) == 0);

				uint8_t *tmp = alloc(connlistlen);
				__assert(tmp);

				int readoffset = 0;
				while (readoffset < connlistlen) {
					uint32_t value = hdagetconnlistentry(codec, nid, readoffset);
					int to_read = (int)min(connlistlen - readoffset, 4);

					for (int i = 0; i < to_read; i++) {
						uint8_t entry = (value >> (i * 8)) & 0xff;
						tmp[readoffset + i] = entry;

						// entry is a range with the previous entry
						if (entry & (1 << 7)) {
							__assert(readoffset + i > 0);
							__assert((tmp[readoffset + i - 1] & (1 << 7)) == 0);
							__assert((entry & 0x7f) >= tmp[readoffset + i - 1]);

							uint8_t count = (entry & 0x7f) - tmp[readoffset + i - 1];
							connectioncount += count;
						}
						else {
							connectioncount++;
						}
					}

					readoffset += to_read;
				}

				connections = alloc(connectioncount * sizeof(hdawidgetconnection_t));
				__assert(connections);

				int offset = 0;
				for (int i = 0; i < connlistlen; i++) {
					uint8_t entry = tmp[i];

					// entry is a range with the previous entry
					if (entry & (1 << 7)) {
						uint8_t start = tmp[i - 1];
						uint8_t end = entry & 0x7f;

						for (uint8_t j = start + 1; j <= end; j++) {
							hdawidgetconnection_t *connection = &connections[offset++];
							connection->nid = j;
							connections->selectorindex = i - 1;
						}
					}
					else {
						connections[offset++] = (hdawidgetconnection_t){
							.nid = entry,
							.selectorindex = i
						};
					}
				}

				__assert(offset == connectioncount);

				free(tmp);
			}

			if (type == WIDGET_TYPE_AUDIO_OUT) {
				uint8_t *outnids = realloc(codec->outnids, codec->outcount + 1);
				__assert(outnids);
				codec->outnids = outnids;

				codec->outnids[codec->outcount++] = nid;
			}
			else if (type == WIDGET_TYPE_PIN_COMPLEX) {
				uint8_t *pinnids = realloc(codec->pinnids, codec->pincount + 1);
				__assert(pinnids);
				codec->pinnids = pinnids;

				codec->pinnids[codec->pincount++] = nid;
			}

			codec->widgets[nid] = (hdawidget_t){
				.connections = connections,
				.connectioncount = connectioncount,
				.pincaps = pincaps,
				.inampcaps = inampcaps,
				.outampcaps = outampcaps,
				.config = config,
				.type = type,
				.nid = nid
			};
		}
	}

	hdafindoutputpaths(codec);

	for (int i = 0; i < codec->outpathcount; i++) {
		hdapath_t *path = &codec->outpaths[i];

		path->oss = (ossdesc_t){
			.oformats = AFMT_U8 | AFMT_U16_LE | AFMT_S16_LE,
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
			.private = path,
			.setrate = hdasetrate,
			.setchannels = hdasetchannels,
			.setfmt = hdasetfmt,
			.queue = hdaqueue,
			.pause = hdapause,
			.resume = hdaresume,
			.getbufferavail = hdagetbufferavail,
			.open = hdaopen,
			.close = hdaclose
		};
		path->activeparams = (hdahwparams_t){};
		path->paramschanged = true;

		oss_register(&path->oss);

		hdawidget_t *pin = path->widgets[0];

		uint8_t connectiontype = (pin->config >> 16) & 0xf;
		uint8_t defaultdevice = (pin->config >> 20) & 0xf;
		uint8_t location = (pin->config >> 24) & 0x3f;
		uint8_t connectivity = (pin->config >> 30) & 0x3;

		const char *name = "";
		switch (defaultdevice) {
			case 0:
				name = "line out";
				break;
			case 1:
				name = "speaker";
				break;
			case 2:
				name = "headphone out";
				break;
			case 3:
				name = "cd";
				break;
			case 4:
				name = "spdif out";
				break;
			case 5:
				name = "digital other out";
				break;
			case 6:
				name = "modem line side";
				break;
			case 7:
				name = "modem handset side";
				break;
			case 8:
				name = "line in";
				break;
			case 9:
				name = "aux";
				break;
			case 0xa:
				name = "mic in";
				break;
			case 0xb:
				name = "telephony";
				break;
			case 0xc:
				name = "spdif in";
				break;
			case 0xd:
				name = "digital other in";
				break;
			case 0xe:
				name = "reserved";
				break;
			case 0xf:
				name = "other";
				break;
			default:
				name = "unknown";
				break;
		}

		const char *locstr;
		uint8_t geometric = location & 0xf;
		switch (geometric) {
			case 0:
				locstr = "n/a";
				break;
			case 1:
				locstr = "rear";
				break;
			case 2:
				locstr = "front";
				break;
			case 3:
				locstr = "left";
				break;
			case 4:
				locstr = "right";
				break;
			case 5:
				locstr = "top";
				break;
			case 6:
				locstr = "bottom";
				break;
			case 7:
				locstr = "special (eg. rear panel";
				break;
			default:
				locstr = "unknown";
				break;
		}

		printf("hda: oss device dsp%d: %s %s\n", path->oss.id, locstr, name);
	}

	if (codec->outpathcount) {
		for (size_t i = 0; i < codec->outpathcount; i++) {
			hdahwparams_t params = {
				.samplerate = 44100,
				.bits = 16,
				.channels = 2
			};
			codec->outpaths[i].activeparams = params;
			codec->outpaths[i].paramschanged = true;
		}
	}
}

static void hdasetactivepath(hdacodec_t *codec, hdapath_t *path) {
	bool intstatus = interrupt_set(false);
	spinlock_acquire(&codec->lock);

	printf("hda: setting out path\n");
	for (int i = 0; i < path->widgetcount; i++) {
		hdawidget_t *widget = path->widgets[i];

		hdasetpowerstate(codec, widget->nid, 0);

		if (i != path->widgetcount - 1) {
			hdawidget_t *next = path->widgets[i + 1];

			if (widget->connectioncount > 1) {
				int index = 0;
				for (int j = 0; j < widget->connectioncount; j++) {
					if (next->nid == widget->connections[j].nid) {
						index = widget->connections[j].selectorindex;
						break;
					}
				}

				printf("set connection of %d to %d (%d selector)\n", widget->nid, next->nid, index);
				hdasetconnection(codec, widget->nid, index);
			}
		}

		printf("widget %d type %d\n", widget->nid, widget->type);

		if (widget->type == WIDGET_TYPE_PIN_COMPLEX) {
			if ((widget->pincaps) & (1 << 16)) {
				hdaseteapdenable(codec, widget->nid, EAPD_ENABLE_EAPD);
			}

			uint8_t maxgain = widget->outampcaps >> 8 & 0x7f;
			uint16_t ampdata = AMP_SET_OUT |
				AMP_SET_LEFT |
				AMP_SET_RIGHT |
				AMP_SET_GAIN(maxgain);
			hdasetampgain(codec, widget->nid, ampdata);

			uint8_t pindata = PIN_CONTROL_SET_HPHN | PIN_CONTROL_SET_OUT_ENABLE;
			hdasetpincontrol(codec, widget->nid, pindata);
		}
		else if (widget->type == WIDGET_TYPE_AUDIO_MIXER) {
			uint8_t maxgain = widget->outampcaps >> 8 & 0x7f;
			uint16_t ampdata = AMP_SET_OUT |
				AMP_SET_LEFT |
				AMP_SET_RIGHT |
				AMP_SET_GAIN(maxgain);
			hdasetampgain(codec, widget->nid, ampdata);

			maxgain = widget->inampcaps >> 8 & 0x7f;
			ampdata = AMP_SET_IN |
				AMP_SET_LEFT |
				AMP_SET_RIGHT |
				AMP_SET_GAIN(maxgain);
			hdasetampgain(codec, widget->nid, ampdata);
		}
		else if (widget->type == WIDGET_TYPE_AUDIO_OUT) {
			uint8_t maxgain = widget->outampcaps >> 8 & 0x7f;
			uint16_t ampdata = AMP_SET_OUT |
				AMP_SET_LEFT |
				AMP_SET_RIGHT |
				AMP_SET_GAIN(maxgain);
			hdasetampgain(codec, widget->nid, ampdata);
			hdasetconvertercontrol(codec, widget->nid, 1, 0);
		}
		else {
			printf("hda: unsupported widget type %d\n", widget->type);
			uint8_t maxgain = widget->outampcaps >> 8 & 0x7f;
			uint16_t ampdata = AMP_SET_OUT |
				AMP_SET_LEFT |
				AMP_SET_RIGHT |
				AMP_SET_GAIN(maxgain);
			hdasetampgain(codec, widget->nid, ampdata);
		}
	}

	codec->activeoutpath = path;

	spinlock_release(&codec->lock);
	interrupt_set(intstatus);
}

static uint16_t makehdaformat(const hdahwparams_t *params, hdahwparams_t* actual) {
	if (params->channels == 0) {
		actual->channels = 2;
	}
	else if (params->channels > 16) {
		actual->channels = 16;
	}
	else {
		actual->channels = params->channels;
	}

	uint16_t value = 0;
	value |= FMT_CHAN(actual->channels);

	if (params->bits <= 8) {
		actual->bits = 8;
		value |= FMT_BITS_8;
	}
	else if (params->bits <= 16) {
		actual->bits = 16;
		value |= FMT_BITS_16;
	}
	else if (params->bits <= 20) {
		actual->bits = 20;
		value |= FMT_BITS_20;
	}
	else if (params->bits <= 24) {
		actual->bits = 24;
		value |= FMT_BITS_24;
	}
	else {
		actual->bits = 32;
		value |= FMT_BITS_32;
	}

	uint32_t rate = params->samplerate;
	uint32_t actualrate;
	if (rate <= 5513) {
		actualrate = 5513;
		value |= FMT_BASE_441KHZ | FMT_DIV(8);
	}
	if (rate <= 6000) {
		actualrate = 6000;
		value |= FMT_BASE_48KHZ | FMT_DIV(8);
	}
	else if (rate <= 6300) {
		actualrate = 6300;
		value |= FMT_BASE_441KHZ | FMT_DIV(7);
	}
	else if (rate <= 6857) {
		actualrate = 6857;
		value |= FMT_BASE_48KHZ | FMT_DIV(7);
	}
	else if (rate <= 7350) {
		actualrate = 7350;
		value |= FMT_BASE_441KHZ | FMT_DIV(6);
	}
	else if (rate <= 8000) {
		actualrate = 8000;
		value |= FMT_BASE_48KHZ | FMT_DIV(6);
	}
	else if (rate <= 8820) {
		actualrate = 8820;
		value |= FMT_BASE_441KHZ | FMT_DIV(5);
	}
	else if (rate <= 9600) {
		actualrate = 9600;
		value |= FMT_BASE_48KHZ | FMT_DIV(5);
	}
	else if (rate <= 11025) {
		actualrate = 11025;
		value |= FMT_BASE_441KHZ | FMT_DIV(4);
	}
	else if (rate <= 12000) {
		actualrate = 12000;
		value |= FMT_BASE_48KHZ | FMT_DIV(4);
	}
	else if (rate <= 12600) {
		actualrate = 12600;
		value |= FMT_BASE_441KHZ | FMT_DIV(7) | FMT_MULT(2);
	}
	else if (rate <= 13714) {
		actualrate = 13714;
		value |= FMT_BASE_48KHZ | FMT_DIV(7) | FMT_MULT(2);
	}
	else if (rate <= 14700) {
		actualrate = 14700;
		value |= FMT_BASE_441KHZ | FMT_DIV(3);
	}
	else if (rate <= 16000) {
		actualrate = 16000;
		value |= FMT_BASE_48KHZ | FMT_DIV(3);
	}
	else if (rate <= 16538) {
		actualrate = 16538;
		value |= FMT_BASE_441KHZ | FMT_DIV(8) | FMT_MULT(3);
	}
	else if (rate <= 17640) {
		actualrate = 17640;
		value |= FMT_BASE_441KHZ | FMT_DIV(5) | FMT_MULT(2);
	}
	else if (rate <= 18000) {
		actualrate = 18000;
		value |= FMT_BASE_48KHZ | FMT_DIV(8) | FMT_MULT(3);
	}
	else if (rate <= 18900) {
		actualrate = 18900;
		value |= FMT_BASE_441KHZ | FMT_DIV(7) | FMT_MULT(3);
	}
	else if (rate <= 19200) {
		actualrate = 19200;
		value |= FMT_BASE_48KHZ | FMT_DIV(5) | FMT_MULT(2);
	}
	else if (rate <= 20571) {
		actualrate = 20571;
		value |= FMT_BASE_48KHZ | FMT_DIV(7) | FMT_MULT(3);
	}
	else if (rate <= 22050) {
		actualrate = 22050;
		value |= FMT_BASE_441KHZ | FMT_DIV(2);
	}
	else if (rate <= 24000) {
		actualrate = 24000;
		value |= FMT_BASE_48KHZ | FMT_DIV(2);
	}
	else if (rate <= 25200) {
		actualrate = 25200;
		value |= FMT_BASE_441KHZ | FMT_DIV(7) | FMT_MULT(4);
	}
	else if (rate <= 26460) {
		actualrate = 26460;
		value |= FMT_BASE_441KHZ | FMT_DIV(5) | FMT_MULT(3);
	}
	else if (rate <= 27429) {
		actualrate = 27429;
		value |= FMT_BASE_48KHZ | FMT_DIV(7) | FMT_MULT(4);
	}
	else if (rate <= 28800) {
		actualrate = 28800;
		value |= FMT_BASE_48KHZ | FMT_DIV(5) | FMT_MULT(3);
	}
	else if (rate <= 29400) {
		actualrate = 29400;
		value |= FMT_BASE_441KHZ | FMT_DIV(3) | FMT_MULT(2);
	}
	else if (rate <= 32000) {
		actualrate = 32000;
		value |= FMT_BASE_48KHZ | FMT_DIV(3) | FMT_MULT(2);
	}
	else if (rate <= 33075) {
		actualrate = 33075;
		value |= FMT_BASE_441KHZ | FMT_DIV(4) | FMT_MULT(3);
	}
	else if (rate <= 35280) {
		actualrate = 35280;
		value |= FMT_BASE_441KHZ | FMT_DIV(5) | FMT_MULT(4);
	}
	else if (rate <= 36000) {
		actualrate = 36000;
		value |= FMT_BASE_48KHZ | FMT_DIV(4) | FMT_MULT(3);
	}
	else if (rate <= 38400) {
		actualrate = 38400;
		value |= FMT_BASE_48KHZ | FMT_DIV(5) | FMT_MULT(4);
	}
	else if (rate <= 44100) {
		actualrate = 44100;
		value |= FMT_BASE_441KHZ;
	}
	else if (rate <= 48000) {
		actualrate = 48000;
		value |= FMT_BASE_48KHZ;
	}
	else if (rate <= 58800) {
		actualrate = 58800;
		value |= FMT_BASE_441KHZ | FMT_DIV(3) | FMT_MULT(4);
	}
	else if (rate <= 64000) {
		actualrate = 64000;
		value |= FMT_BASE_48KHZ | FMT_DIV(3) | FMT_MULT(4);
	}
	else if (rate <= 66150) {
		actualrate = 66150;
		value |= FMT_BASE_441KHZ | FMT_DIV(2) | FMT_MULT(3);
	}
	else if (rate <= 72000) {
		actualrate = 72000;
		value |= FMT_BASE_48KHZ | FMT_DIV(2) | FMT_MULT(3);
	}
	else if (rate <= 88200) {
		actualrate = 88200;
		value |= FMT_BASE_441KHZ | FMT_MULT(2);
	}
	else if (rate <= 96000) {
		actualrate = 96000;
		value |= FMT_BASE_48KHZ | FMT_MULT(2);
	}
	else if (rate <= 132300) {
		actualrate = 132300;
		value |= FMT_BASE_441KHZ | FMT_MULT(3);
	}
	else if (rate <= 144000) {
		actualrate = 144000;
		value |= FMT_BASE_48KHZ | FMT_MULT(3);
	}
	else if (rate <= 176400) {
		actualrate = 176400;
		value |= FMT_BASE_441KHZ | FMT_MULT(4);
	}
	else {
		actualrate = 192000;
		value |= FMT_BASE_48KHZ | FMT_MULT(4);
	}

	actual->samplerate = actualrate;
	return value;
}

static void hdasetactiveparams(hdacodec_t *codec, const hdahwparams_t *params) {
	bool intstatus = interrupt_set(false);
	spinlock_acquire(&codec->lock);

	hdahwparams_t tmp;
	uint16_t fmt = makehdaformat(params, &tmp);

	hdapath_t *path = codec->activeoutpath;

	hdawidget_t *out = path->widgets[path->widgetcount - 1];
	hdasetconverterchannels(codec, out->nid, params->channels);
	hdasetconverterfmt(codec, out->nid, fmt);

	codec->ctrl->outstreams[0].regs->fmt = fmt;

	spinlock_release(&codec->lock);
	interrupt_set(intstatus);
}

static void hdastreamrun(hdastream_t *strm, bool run) {
	uint8_t ctl0 = strm->regs->ctl0;
	SDCTL0_SETRUN(ctl0, run);
	strm->regs->ctl0 = ctl0;
}

__attribute__((format(printf, 1, 2))) static void hdaprint(const char *fmt, ...) {
	char buf[128];
	va_list ap;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	arch_e9_puts(buf);
}

static void hdastream_irq(hdastream_t *strm) {
	size_t pos = strm->currentpos;

	size_t index = pos / PAGE_SIZE;
	uintptr_t address = strm->bdl[index].address;

	spinlock_acquire(&strm->bufferlock);
	size_t size = RINGBUFFER_DATACOUNT(&strm->buffer);

	if (size < PAGE_SIZE) {
		hdaprint("hdastream_irq: buffer underflow\n");
		memset(MAKE_HHDM(address), 0, PAGE_SIZE);
		semaphore_signal_limit(&strm->buffersem, 1);
	}
	else {
		ringbuffer_read(&strm->buffer, MAKE_HHDM(address), PAGE_SIZE);
		if (size - PAGE_SIZE < RINGBUFFER_SIZE(&strm->buffer) / 2) {
			semaphore_signal(&strm->buffersem);
		}
	}

	spinlock_release(&strm->bufferlock);

	strm->currentpos += PAGE_SIZE;
	if (strm->currentpos == 256 * PAGE_SIZE) {
		strm->currentpos = 0;
	}

	uint8_t sts = strm->regs->sts;
	__assert(SDSTS_BCIS(sts));
	SDSTS_CLEARBCIS(sts);
	strm->regs->sts = sts;
}

static void hda_irq(isr_t *isr, context_t *context) {
	hdacontroller_t *ctrl = isr->priv;

	uint8_t rirbsts = ctrl->bar0->rirbsts;
	if (RIRBSTS_INTFL(rirbsts)) {
		uint16_t wp = RIRBWP_WP(ctrl->bar0->rirbwp);

		int i = ctrl->lastprocessedrirb;
		while (true) {
			__assert(ctrl->threads[i]);
			sched_wakeup(ctrl->threads[i], SCHED_WAKEUP_REASON_NORMAL);

			if (i == wp) {
				break;
			}

			i++;
			if (i == ctrl->rirbcount) {
				i = 0;
			}
		}

		ctrl->lastprocessedrirb = wp;

		RIRBSTS_CLEARINTFL(rirbsts);
		ctrl->bar0->rirbsts = rirbsts;
	}
	else {
		int totalstreams = ctrl->instreamcount +
			ctrl->outstreamcount +
			ctrl->bistreamcount;

		uint32_t sts = ctrl->bar0->intsts;
		for (int i = 0; i < totalstreams; i++) {
			if (INTSTS_SIS(sts, i)) {
				hdastream_t* strm;
				if (i < ctrl->instreamcount) {
					strm = &ctrl->instreams[i];
				}
				else if (i < ctrl->instreamcount + ctrl->outstreamcount) {
					strm = &ctrl->outstreams[i - ctrl->instreamcount];
				}
				else {
					strm = &ctrl->bistreams[i - ctrl->instreamcount - ctrl->outstreamcount];
				}

				hdastream_irq(strm);
			}
		}
	}
}

static int hdasetrate(ossdesc_t *desc, int rate) {
	hdapath_t *path = desc->private;

	if (rate < desc->minrate) {
		rate = desc->minrate;
	}
	else if (rate > desc->maxrate) {
		rate = desc->maxrate;
	}

	if (path->activeparams.samplerate != rate) {
		path->activeparams.samplerate = rate;
		path->paramschanged = true;
	}

	return rate;
}

static int hdasetchannels(ossdesc_t *desc, int channels) {
	hdapath_t *path = desc->private;

	if (channels < desc->minchannels) {
		channels = desc->minrate;
	}
	else if (channels > desc->maxchannels) {
		channels = desc->maxrate;
	}

	if (path->activeparams.channels != channels) {
		path->activeparams.channels = channels;
		path->paramschanged = true;
	}

	return channels;
}

static int hdasetfmt(ossdesc_t *desc, int fmt) {
	hdapath_t *path = desc->private;

	if (fmt == AFMT_U16_LE) {
		if (path->activeparams.bits != 16) {
			path->activeparams.bits = 16;
			path->paramschanged = true;
		}

		path->issigned = false;

		return AFMT_U16_LE;
	}
	else if (fmt == AFMT_S16_LE) {
		if (path->activeparams.bits != 16) {
			path->activeparams.bits = 16;
			path->paramschanged = true;
		}

		path->issigned = true;

		return AFMT_S16_LE;
	}
	else {
		if (path->activeparams.bits != 8) {
			path->activeparams.bits = 8;
			path->paramschanged = true;
		}

		return AFMT_U8;
	}
}

static int hdaqueue(ossdesc_t *desc, iovec_iterator_t *iovec_iterator, size_t size, size_t *writep) {
	hdapath_t *path = desc->private;
	hdastream_t *strm = &path->codec->ctrl->outstreams[0];

	bool irqstate = interrupt_set(false);
	MUTEX_ACQUIRE(&strm->lock, false);

	if (!RINGBUFFER_SIZE(&strm->buffer)) {
		if (!path->oss.fragmentcount) {
			path->oss.fragmentcount = 2;
		}
		if (!path->oss.fragmentsize) {
			size_t bytespersecond = path->activeparams.samplerate * path->activeparams.channels * (path->activeparams.bits / 8);
			size_t bytesperms = bytespersecond / 1000;
			path->oss.fragmentsize = (int)bytesperms * 50;
		}
		hdaprint("hdaqueue: buffer initialized to %d fragments of %d bytes\n", path->oss.fragmentcount, path->oss.fragmentsize);
		ringbuffer_init(&strm->buffer, path->oss.fragmentcount * path->oss.fragmentsize);
	}

	int ret = 0;

	size_t done = 0;
	while (done < size) {
		spinlock_acquire(&strm->bufferlock);

		size_t freespace = RINGBUFFER_FREESPACE(&strm->buffer);
		if (!freespace) {
			spinlock_release(&strm->bufferlock);
			int err = semaphore_wait(&strm->buffersem, true);
			if (err == SCHED_WAKEUP_REASON_INTERRUPTED) {
				ret = EINTR;
				break;
			}

			continue;
		}

		void *page;
		size_t page_offset, page_remaining;
		ret = iovec_iterator_next_page(iovec_iterator, &page_offset, &page_remaining, &page);
		if (ret)
			break;

		__assert(page);

		size_t tocopy = freespace;
		if (tocopy > page_remaining) {
			tocopy = page_remaining;
		}
		ringbuffer_write(&strm->buffer, MAKE_HHDM(page), tocopy);
		spinlock_release(&strm->bufferlock);

		pmm_release(page);

		// if we didnt use the whole space in the page, set the iterator back a bit
		size_t diff_between_available_and_used = page_remaining - tocopy;
		if (diff_between_available_and_used) {
			size_t iterator_offset = iovec_iterator_total_offset(iovec_iterator);
			iovec_iterator_set(iovec_iterator, iterator_offset - diff_between_available_and_used);
		}

		done += tocopy;

		if (!SDCTL0_RUN(strm->regs->ctl0) &&
			RINGBUFFER_SIZE(&strm->buffer) - freespace + tocopy >= path->oss.fragmentsize) {
			hdaprint("hdaqueue: starting stream\n");

			size_t initialcount;
			if (path->oss.fragmentsize < PAGE_SIZE * 2) {
				initialcount = 2;
			}
			else {
				initialcount = path->oss.fragmentsize / PAGE_SIZE;
			}

			spinlock_acquire(&strm->bufferlock);

			for (size_t offset = 0; offset < initialcount; offset++) {
				uintptr_t address = strm->bdl[offset].address;

				ringbuffer_read(&strm->buffer, MAKE_HHDM(address), PAGE_SIZE);
			}

			spinlock_release(&strm->bufferlock);

			strm->currentpos = initialcount * PAGE_SIZE;

			hdaresume_unlocked(desc);
		}
	}

	MUTEX_RELEASE(&strm->lock);
	interrupt_set(irqstate);

	if (ret)
		return ret;

	*writep = size;
	return 0;
}

static int hdapause(ossdesc_t *desc, bool waituntilempty, bool reset) {
	hdapath_t *path = desc->private;
	hdastream_t *strm = &path->codec->ctrl->outstreams[0];

	bool irqstate = interrupt_set(false);
	MUTEX_ACQUIRE(&strm->lock, false);

	if (waituntilempty && SDCTL0_RUN(strm->regs->ctl0)) {
		while (true) {
			spinlock_acquire(&strm->bufferlock);
			size_t avail = RINGBUFFER_DATACOUNT(&strm->buffer);
			spinlock_release(&strm->bufferlock);
			if (avail < PAGE_SIZE) {
				break;
			}
			else {
				int err = semaphore_wait(&strm->buffersem, true);
				if (err == SCHED_WAKEUP_REASON_INTERRUPTED) {
					break;
				}
			}
		}
	}

	hdastreamrun(strm, false);

	if (reset) {
		uint8_t ctl0 = strm->regs->ctl0;
		SDCTL0_SETRST(ctl0, true);
		strm->regs->ctl0 = ctl0;
		while (!SDCTL0_RST(strm->regs->ctl0));

		ctl0 = strm->regs->ctl0;
		SDCTL0_SETRST(ctl0, false);
		strm->regs->ctl0 = ctl0;
		while (SDCTL0_RST(strm->regs->ctl0));

		ctl0 = strm->regs->ctl0;
		SDCTL0_SETIOCE(ctl0, true);
		strm->regs->ctl0 = ctl0;

		uint8_t ctl2 = strm->regs->ctl2;
		SDCTL2_SETSTRM(ctl2, 1);
		strm->regs->ctl2 = ctl2;

		strm->currentpos = 0;
		ringbuffer_remove(&strm->buffer, RINGBUFFER_DATACOUNT(&strm->buffer));
	}

	MUTEX_RELEASE(&strm->lock);
	interrupt_set(irqstate);

	return 0;
}

static int hdaresume_unlocked(ossdesc_t *desc) {
	hdapath_t *path = desc->private;

	if (path->codec->activeoutpath != path) {
		hdasetactivepath(path->codec, path);
	}

	if (path->codec->activeoutpath->paramschanged) {
		path->codec->activeoutpath->paramschanged = false;
		hdaprint(
			"hdaresume: rate set to %u, channels %hhu and bits %hhu (%u bytes per second)\n",
			path->codec->activeoutpath->activeparams.samplerate,
			path->codec->activeoutpath->activeparams.channels,
			path->codec->activeoutpath->activeparams.bits,
			path->codec->activeoutpath->activeparams.samplerate *
			path->codec->activeoutpath->activeparams.channels *
			path->codec->activeoutpath->activeparams.bits / 8);
		hdasetactiveparams(path->codec, &path->codec->activeoutpath->activeparams);
	}

	hdastream_t *strm = &path->codec->ctrl->outstreams[0];
	hdastreamrun(strm, true);
	return 0;
}

static int hdaresume(ossdesc_t *desc) {
	hdapath_t *path = desc->private;
	hdastream_t *strm = &path->codec->ctrl->outstreams[0];

	bool irqstate = interrupt_set(false);
	MUTEX_ACQUIRE(&strm->lock, false);

	int err = hdaresume_unlocked(desc);

	MUTEX_RELEASE(&strm->lock);
	interrupt_set(irqstate);
	return err;
}

static int hdagetbufferavail(ossdesc_t *desc, int *ret) {
	hdapath_t *path = desc->private;
	hdastream_t *strm = &path->codec->ctrl->outstreams[0];

	bool irqstate = spinlock_acquireirqclear(&strm->bufferlock);

	size_t space = RINGBUFFER_FREESPACE(&strm->buffer);

	spinlock_releaseirqrestore(&strm->bufferlock, irqstate);

	*ret = (int)space;
	return 0;
}

static int hdaopen(ossdesc_t *desc) {
	hdapath_t *path = desc->private;
	if (!MUTEX_TRY(&path->codec->exclusivelock)) {
		return EBUSY;
	}
	return 0;
}

static int hdaclose(ossdesc_t *desc) {
	hdapath_t *path = desc->private;
	MUTEX_RELEASE(&path->codec->exclusivelock);
	return 0;
}

static void initcontroller(pcienum_t *e) {
	pcibar_t bar0p = pci_getbar(e, 0);
	volatile hdabar0_t *bar0 = (volatile hdabar0_t *)bar0p.address;
	__assert(bar0);

	pci_setcommand(e, PCI_COMMAND_MMIO, 1);
	pci_setcommand(e, PCI_COMMAND_IO, 0);
	pci_setcommand(e, PCI_COMMAND_IRQDISABLE, 1);
	pci_setcommand(e, PCI_COMMAND_BUSMASTER, 1);

	printf("hda: found controller at %02x:%02x.%x\n", e->bus, e->device, e->function);

	if (GCAP_64OK(bar0->gcap) == 0) {
		printf("hda: controller doesn't support 64-bit\n");
		return;
	}

	hdacontroller_t *controller = alloc(sizeof(hdacontroller_t));
	__assert(controller);

	controller->bar0 = bar0;
	controller->outstreamcount = GCAP_OSS(bar0->gcap);
	controller->instreamcount = GCAP_ISS(bar0->gcap);
	controller->bistreamcount = GCAP_BSS(bar0->gcap);
	controller->corbptr = 0;
	controller->rirbptr = 0;
	controller->lastprocessedrirb = 1;
	SPINLOCK_INIT(controller->lock);

	printf("hda: controller supports %d in streams, %d out streams and %d bi-directional streams\n",
		controller->instreamcount,
		controller->outstreamcount,
		controller->bistreamcount);

	for (int i = 0; i < controller->instreamcount; i++) {
		hdastreamregs_t *regs = (hdastreamregs_t *)((uintptr_t)bar0 + 0x80 + i * 0x20);
		controller->instreams[i].regs = regs;
	}

	for (int i = 0; i < controller->outstreamcount; i++) {
		hdastreamregs_t *regs = (hdastreamregs_t *)(
			(uintptr_t)bar0 + 0x80 + controller->instreamcount * 0x20 + i * 0x20);
		controller->outstreams[i].regs = regs;
	}

	for (int i = 0; i < controller->bistreamcount; i++) {
		hdastreamregs_t *regs = (hdastreamregs_t *)(
			(uintptr_t)bar0 + 0x80 + controller->instreamcount * 0x20 +
			controller->outstreamcount * 0x20 + i * 0x20);
		controller->outstreams[i].regs = regs;
	}

	// check if the controller is running and stop it if it is
	if (GCTL_CRST(bar0->gctl)) {
		uint8_t corbctl = bar0->corbctl;
		CORBCTL_SETRUN(corbctl, false);
		bar0->corbctl = corbctl;
		while (CORBCTL_RUN(bar0->corbctl));

		uint8_t rirbctl = bar0->rirbctl;
		RIRBCTL_SETDMAEN(rirbctl, false);
		bar0->rirbctl = rirbctl;
		while (RIRBCTL_DMAEN(bar0->rirbctl));

		for (int i = 0; i < controller->instreamcount; i++) {
			hdastream_t *stream = &controller->instreams[i];
			uint8_t ctl0 = stream->regs->ctl0;
			SDCTL0_SETRUN(ctl0, false);
			stream->regs->ctl0 = ctl0;
			while (SDCTL0_RUN(stream->regs->ctl0));
		}

		for (int i = 0; i < controller->outstreamcount; i++) {
			hdastream_t *stream = &controller->outstreams[i];
			uint8_t ctl0 = stream->regs->ctl0;
			SDCTL0_SETRUN(ctl0, false);
			stream->regs->ctl0 = ctl0;
			while (SDCTL0_RUN(stream->regs->ctl0));
		}

		for (int i = 0; i < controller->bistreamcount; i++) {
			hdastream_t *stream = &controller->bistreams[i];
			uint8_t ctl0 = stream->regs->ctl0;
			SDCTL0_SETRUN(ctl0, false);
			stream->regs->ctl0 = ctl0;
			while (SDCTL0_RUN(stream->regs->ctl0));
		}

		uint32_t gctl = bar0->gctl;
		GCTL_SETCRST(gctl, false);
		bar0->gctl = gctl;
		while (GCTL_CRST(bar0->gctl));
	}

	uint32_t gctl = bar0->gctl;
	GCTL_SETCRST(gctl, true);
	bar0->gctl = gctl;
	while (!GCTL_CRST(bar0->gctl));

	void *corbrirbphys = pmm_allocpage(PMM_SECTION_DEFAULT);
	__assert(corbrirbphys);
	controller->corbrirbphys = corbrirbphys;

	uint8_t corbsize = bar0->corbsize;
	uint8_t corbsizecap = CORBSIZE_SZCAP(corbsize);
	uint8_t corbsizevalue;
	if (corbsizecap & (1 << 2)) {
		controller->corbcount = 256;
		corbsizevalue = 1 << 1;
	}
	else if (corbsizecap & (1 << 1)) {
		controller->corbcount = 16;
		corbsizevalue = 1;
	}
	else {
		__assert(corbsizecap & 1);
		controller->corbcount = 2;
		corbsizevalue = 0;
	}

	CORBSIZE_SETSIZE(corbsize, corbsizevalue);
	bar0->corbsize = corbsize;
	bar0->corblbase = (uintptr_t)corbrirbphys;
	bar0->corbubase = (uintptr_t)corbrirbphys >> 32;

	uint8_t rirbsize = bar0->rirbsize;
	uint8_t rirbsizecap = RIRBSIZE_SZCAP(rirbsize);
	uint8_t rirbsizevalue;
	if (rirbsizecap & (1 << 2)) {
		controller->rirbcount = 256;
		rirbsizevalue = 1 << 1;
	}
	else if (rirbsizecap & (1 << 1)) {
		controller->rirbcount = 16;
		rirbsizevalue = 1;
	}
	else {
		__assert(rirbsizecap & 1);
		controller->rirbcount = 2;
		rirbsizevalue = 0;
	}

	RIRBSIZE_SETSIZE(rirbsize, rirbsizevalue);
	bar0->rirbsize = rirbsize;
	bar0->rirblbase = (uintptr_t)corbrirbphys + PAGE_SIZE / 2;
	bar0->rirbubase = ((uintptr_t)corbrirbphys + PAGE_SIZE / 2) >> 32;

	controller->corb = MAKE_HHDM(corbrirbphys);
	controller->rirb = MAKE_HHDM((uintptr_t)corbrirbphys + PAGE_SIZE / 2);

	controller->threads = alloc(sizeof(thread_t *) * controller->rirbcount);
	__assert(controller->threads);
	memset(controller->threads, 0, sizeof(thread_t *) * controller->rirbcount);

	SEMAPHORE_INIT(&controller->entrysem, controller->rirbcount);

	uint8_t corbctl = bar0->corbctl;
	CORBCTL_SETRUN(corbctl, true);
	bar0->corbctl = corbctl;

	uint16_t rintcnt = bar0->rintcnt;
	RINTCNT_SETCNT(rintcnt, 1);
	bar0->rintcnt = rintcnt;

	uint8_t rirbctl = bar0->rirbctl;
	RIRBCTL_SETDMAEN(rirbctl, true);
	RIRBCTL_SETRINTCTL(rirbctl, true);
	bar0->rirbctl = rirbctl;

	// wait for codec reset
	sched_sleep_us(1000);

	isr_t *isr = interrupt_allocate(hda_irq, ARCH_EOI, IPL_NET);
	__assert(isr);
	isr->priv = controller;
	controller->isr = isr;

	// enable interrupts
	if (e->msix.exists) {
		size_t intcount = pci_initmsix(e);
		__assert(intcount);

		pci_msixadd(e, 0, INTERRUPT_IDTOVECTOR(isr->id), 1, 0);
		pci_msixsetmask(e, 0);
	} else if (e->msi.exists) {
		size_t intcount = pci_initmsi(e, 1);
		__assert(intcount);
		pci_msisetbase(e, INTERRUPT_IDTOVECTOR(isr->id), 1, 0);
	} else {
		printf("hda: controller doesn't support msi-x or msi\n");
		free(controller);
		pmm_free(corbrirbphys, PAGE_SIZE);
		return;
	}

	int totalstreams = controller->instreamcount +
		controller->outstreamcount +
		controller->bistreamcount;

	uint32_t intctl = bar0->intctl;
	INTCTL_SETGIE(intctl, true);
	INTCTL_SETCIE(intctl, true);
	for (int i = 0; i < totalstreams; i++) {
		INTCTL_SETSIE(intctl, i, true);
	}
	bar0->intctl = intctl;

	volatile hdastreamregs_t *regs = controller->outstreams[0].regs;

	uint8_t ctl0 = regs->ctl0;
	SDCTL0_SETIOCE(ctl0, true);
	regs->ctl0 = ctl0;

	uint8_t ctl2 = regs->ctl2;
	SDCTL2_SETSTRM(ctl2, 1);
	regs->ctl2 = ctl2;

	// check for attached codecs
	uint16_t status = bar0->wakests;
	for (int i = 0; i < 15; i++) {
		if (status & (1 << i)) {
			hdaenumeratecodec(controller, i);
		}
	}

	__assert(controller->outstreamcount);
	hdastream_t *strm = &controller->outstreams[0];

	void *bdlphys = pmm_allocpage(PMM_SECTION_DEFAULT);
	__assert(bdlphys);

	strm->regs->bdpl = (uintptr_t)bdlphys;
	strm->regs->bdpu = (uintptr_t)bdlphys >> 32;

	strm->bdl = MAKE_HHDM(bdlphys);
	strm->currentpos = 0;
	strm->buffer.size = 0;
	strm->buffer.write = 0;
	strm->buffer.read = 0;
	SPINLOCK_INIT(strm->bufferlock);
	MUTEX_INIT(&strm->lock);
	SEMAPHORE_INIT(&strm->buffersem, 0);

	strm->regs->cbl = 256 * PAGE_SIZE;

	uint16_t lvi = strm->regs->lvi;
	SDLVI_SETLVI(lvi, 255);
	strm->regs->lvi = lvi;

	for (int i = 0; i < 256; i++) {
		void *addr = pmm_allocpage(PMM_SECTION_DEFAULT);
		__assert(addr);

		strm->bdl[i] = (hdabdlentry_t){
			.address = (uintptr_t)addr,
			.length = PAGE_SIZE,
			.ioc = 1
		};
	}
}

static pcienum_t *hda_get_dev(int i) {
	pcienum_t *e = pci_getenum(4, 3, -1, -1, -1, -1, i);
	if (e)
		return e;

	e = pci_getenum(-1, -1, -1, 0x8086, 0xa0c8, -1, i);
	if (e)
		return e;

	return NULL;
}

void hda_init() {
	int i = 0;
	for (;;) {
		pcienum_t *e = hda_get_dev(i++);
		if (e == NULL)
			break;
		initcontroller(e);
	}
}
