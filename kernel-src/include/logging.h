#ifndef _LOGGING_H
#define _LOGGING_H

#include <printf.h>
#include <panic.h>
#include <mutex.h>
#include <arch/cpu.h>
#include <util.h>

void _putchar(char c);
void logging_sethook(void (*fun)(char));
void logging_init();
extern mutex_t printf_mutex;

#ifdef printf
#undef printf
#endif

#define printf(...) { \
	if (likely(current_thread())) {\
		MUTEX_ACQUIRE(&printf_mutex); \
	} else { \
		while (MUTEX_TRY(&printf_mutex) == false) CPU_PAUSE(); \
	} \
	printf_(__VA_ARGS__); \
	MUTEX_RELEASE(&printf_mutex); \
}

#define __assert(x) \
	if (unlikely(!(x))){ \
		printf("\e[91m%s:%s:%d: %s failed\n\e[0m", __FILE__, __func__, __LINE__, #x); \
		_panic(NULL, NULL); \
	};


#endif
