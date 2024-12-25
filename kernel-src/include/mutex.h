#ifndef _MUTEX_H
#define _MUTEX_H

#include <semaphore.h>

typedef semaphore_t mutex_t;

#define MUTEX_INIT(m) \
	SEMAPHORE_INIT(m, 1);

#define MUTEX_ACQUIRE(m) \
	semaphore_wait(m, false)

#define MUTEX_ACQUIRE_TIMED(m, t) \
	semaphore_timedwait(m, t, false)

#define MUTEX_RELEASE(m) \
	semaphore_signal(m)

#define MUTEX_TRY(m) \
	semaphore_test(m)

#define MUTEX_DEFINE(x) SEMAPHORE_DEFINE(x, 1)

#endif
