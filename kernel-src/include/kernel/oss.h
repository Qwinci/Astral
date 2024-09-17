#ifndef _OSS_H
#define _OSS_H

#include <stdbool.h>
#include <stddef.h>
#include <kernel/iovec.h>

#define AFMT_U8 0x8
#define AFMT_S16_LE 0x10
#define AFMT_U16_LE 0x80

typedef struct ossdesc_t {
	int id;
	int fragmentsize;
	int fragmentcount;

	int oformats;
	int minrate;
	int maxrate;
	int minchannels;
	int maxchannels;
	unsigned int nrates;
	unsigned int rates[20];

	void *private;

	// returns the actual rate
	int (*setrate)(struct ossdesc_t *desc, int rate);
	// returns the actual count
	int (*setchannels)(struct ossdesc_t *desc, int channels);
	// returns the actual format
	int (*setfmt)(struct ossdesc_t *desc, int fmt);

	// synchronously queue data into the playback buffer and if this is the first write
	// starts playback if enough data has been written
	int (*queue)(struct ossdesc_t *desc, iovec_iterator_t *iovec_iterator, size_t size, size_t *writec);
	// pause the stream optionally waiting for the buffer to be empty and/or resetting the stream afterwards
	int (*pause)(struct ossdesc_t *desc, bool waituntilempty, bool reset);
	// resume the stream after pausing or open
	int (*resume)(struct ossdesc_t *desc);
	// get the available buffer space
	int (*getbufferavail)(struct ossdesc_t *desc, int *ret);

	// open this path (exclusively)
	int (*open)(struct ossdesc_t *desc);
	// close this path
	int (*close)(struct ossdesc_t *desc);
} ossdesc_t;

void oss_register(ossdesc_t *desc);
void oss_init();

#endif
