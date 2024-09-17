#include <kernel/oss.h>
#include <kernel/devfs.h>
#include <kernel/usercopy.h>
#include <kernel/alloc.h>
#include <logging.h>

static mutex_t tablemutex;
static int currentid = 0;
static hashtable_t devtable;

static ossdesc_t *getdesc(int id) {
	void *ret;
	MUTEX_ACQUIRE(&tablemutex, false);
	int err = hashtable_get(&devtable, &ret, &id, sizeof(id));
	MUTEX_RELEASE(&tablemutex);
	return err ? NULL : ret;
}

#define SNDCTL_DSP_GETFMTS 0x8004500b
#define SNDCTL_DSP_GETCAPS 0x8004500f
#define SNDCTL_DSP_GETOSPACE 0x8010500c
#define SNDCTL_DSP_SPEED 0xc0045002
#define SNDCTL_DSP_SETFMT 0xc0045005
#define SNDCTL_DSP_CHANNELS 0xc0045006
#define SNDCTL_DSP_SETFRAGMENT 0xc004500a
#define SNDCTL_ENGINEINFO 0xc49c580c

#define PCM_CAP_DUPLEX 0x100
#define PCM_CAP_OUTPUT 0x20000

typedef char oss_longname_t[64];
typedef char oss_label_t[16];
typedef char oss_devnode_t[32];
typedef char oss_devname_t[64];
typedef char oss_cmd_t[64];
typedef char oss_id_t[16];
typedef char oss_handle_t[32];

typedef struct {
	int dev;
	oss_devname_t name;
	int busy;
	int pid;
	int caps;
	int iformats;
	int oformats;
	int magic;
	oss_cmd_t cmd;
	int card_number;
	int port_number;
	int mixer_dev;
	int legacy_device;
	int enabled;
	int flags;
	int min_rate;
	int max_rate;
	int min_channels;
	int max_channels;
	int binding;
	int rate_source;
	oss_handle_t handle;
	unsigned int nrates;
	unsigned int rates[20];
	oss_longname_t song_name;
	oss_label_t label;
	int latency;
	oss_devnode_t devnode;
	int next_play_engine;
	int next_rec_engine;
	int filler[184];
} oss_audioinfo_t;

typedef struct {
	int fragments;
	int fragstotal;
	int fragsize;
	int bytes;
} audio_buf_info_t;

static int dsp_open(int minor, vnode_t **vnode, int flags) {
	ossdesc_t *desc = getdesc(minor);
	if (desc == NULL)
		return ENODEV;

	return desc->open(desc);
}

static int dsp_close(int minor, int flags) {
	ossdesc_t *desc = getdesc(minor);
	if (desc == NULL)
		return ENODEV;

	desc->pause(desc, true, true);
	desc->close(desc);

	return 0;
}

static int dsp_write(int minor, iovec_iterator_t *iovec_iterator, size_t size, uintmax_t offset, int flags, size_t *writec) {
	ossdesc_t *desc = getdesc(minor);
	if (desc == NULL)
		return ENODEV;

	return desc->queue(desc, iovec_iterator, size, writec);
}

static int dsp_ioctl(int minor, unsigned long request, void *arg, int *result, cred_t *cred) {
	ossdesc_t *desc = getdesc(minor);
	if (desc == NULL)
		return ENODEV;

	int ret = 0;

	switch (request) {
		case SNDCTL_DSP_GETFMTS:
		{
			int fmts = desc->oformats;
			ret = usercopy_touser(arg, &fmts, sizeof(int));
			break;
		}
		case SNDCTL_DSP_GETCAPS:
		{
			int caps = PCM_CAP_DUPLEX | PCM_CAP_OUTPUT;
			ret = usercopy_touser(arg, &caps, sizeof(int));
			break;
		}
		case SNDCTL_DSP_GETOSPACE:
		{
			audio_buf_info_t info = {};
			ret = desc->getbufferavail(desc, &info.bytes);
			if (ret != 0) {
				break;
			}

			ret = usercopy_touser(arg, &info, sizeof(info));
			break;
		}
		case SNDCTL_DSP_SPEED:
		{
			int rate;
			ret = usercopy_fromuser(&rate, arg, sizeof(int));
			if (ret != 0) {
				break;
			}

			rate = desc->setrate(desc, rate);
			ret = usercopy_touser(arg, &rate, sizeof(int));
			break;
		}
		case SNDCTL_DSP_SETFMT:
		{
			int fmt;
			ret = usercopy_fromuser(&fmt, arg, sizeof(int));
			if (ret != 0) {
				break;
			}

			fmt = desc->setfmt(desc, fmt);
			ret = usercopy_touser(arg, &fmt, sizeof(int));
			break;
		}
		case SNDCTL_DSP_CHANNELS:
		{
			int count;
			ret = usercopy_fromuser(&count, arg, sizeof(int));
			if (ret != 0) {
				break;
			}

			count = desc->setchannels(desc, count);
			ret = usercopy_touser(arg, &count, sizeof(int));
			break;
		}
		case SNDCTL_DSP_SETFRAGMENT:
		{
			unsigned int frag;
			ret = usercopy_fromuser(&frag, arg, sizeof(unsigned int));
			if (ret != 0) {
				break;
			}

			int fragsize = 1 << (frag & 0xffff);
			int maxfrags = (int)(frag >> 16);

			desc->fragmentsize = fragsize;
			desc->fragmentcount = maxfrags;

			printf("oss: %d fragments of size %d\n", maxfrags, fragsize);

			break;
		}
		case SNDCTL_ENGINEINFO:
		{
			oss_audioinfo_t *info = alloc(sizeof(oss_audioinfo_t));
			if (!info) {
				ret = ENOMEM;
				break;
			}

			info->dev = desc->id;
			snprintf(info->name, sizeof(info->name), "default");
			info->busy = 0;
			info->pid = 0;
			info->caps = PCM_CAP_OUTPUT;
			info->iformats = 0;
			info->oformats = desc->oformats;
			info->magic = 0;
			info->cmd[0] = 0;
			info->card_number = 0;
			info->port_number = 0;
			info->mixer_dev = 0;
			info->legacy_device = 0;
			info->enabled = 1;
			info->flags = 0;
			info->min_rate = desc->minrate;
			info->max_rate = desc->maxrate;
			info->min_channels = desc->minchannels;
			info->max_channels = desc->maxchannels;
			info->binding = 0;
			info->rate_source = 0;
			info->handle[0] = 0;
			info->nrates = desc->nrates;
			memcpy(info->rates, desc->rates, desc->nrates * sizeof(unsigned int));
			info->song_name[0] = 0;
			info->label[0] = 0;
			info->latency = -1;
			info->devnode[0] = 0;
			info->next_play_engine = 0;
			info->next_rec_engine = 0;

			ret = usercopy_touser(arg, info, sizeof(oss_audioinfo_t));
			free(info);
			break;
		}
		default:
			ret = EINVAL;
			break;
	}

	return ret;
}

static devops_t dsp_devops = {
	.open = dsp_open,
	.close = dsp_close,
	.write = dsp_write,
	.ioctl = dsp_ioctl
};

void oss_register(ossdesc_t *desc) {
	MUTEX_ACQUIRE(&tablemutex, false);
	int id = currentid++;
	int ret = hashtable_set(&devtable, desc, &id, sizeof(id), true);
	MUTEX_RELEASE(&tablemutex);

	desc->id = id;

	__assert(ret == 0);

	char name[32];
	snprintf(name, 32, "dsp%i", id);

	__assert(devfs_register(&dsp_devops, name, V_TYPE_CHDEV, DEV_MAJOR_OSS, id, 0660, NULL) == 0);

	vattr_t attr = {0};
	attr.mode = 0755;
	ret = devfs_createsymlink(name, "dsp", &attr);
	__assert(ret == 0 || ret == EEXIST);
}

void oss_init() {
	hashtable_init(&devtable, 100);
	MUTEX_INIT(&tablemutex);
}
