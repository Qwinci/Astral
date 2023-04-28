#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <arch/context.h>
#include <kernel/vmm.h>

#define SCHED_THREAD_FLAGS_QUEUED 1
#define SCHED_THREAD_FLAGS_RUNNING 2

typedef int pid_t;
typedef int tid_t;
typedef int gid_t;
typedef int uid_t;
typedef unsigned int mode_t;

struct proc_t;

typedef struct thread_t {
	struct thread_t *next;
	struct thread_t *prev;
	struct proc_t *proc;
	struct cpu_t *cpu;
	context_t context;
	extracontext_t extracontext;
	void *kernelstack;
	void *kernelstacktop;
	size_t kernelstacksize;
	vmmcontext_t *vmmctx;
	tid_t tid;
	int flags;
	long priority;
} thread_t;

typedef struct proc_t {
	spinlock_t lock;
	struct proc_t *sibling;
	struct proc_t *parent;
	struct proc_t *child;
	pid_t pid;
	gid_t gid;
	uid_t uid;
	thread_t **threads;
	size_t threadtablesize;
	size_t runningthreadcount;
	mode_t umask;
	int flags;
} proc_t;

#include <arch/cpu.h>

void sched_init();
void sched_runinit();
void sched_threadexit();
void sched_queue(thread_t *thread);
void sched_stopcurrentthread();
void sched_yield();
proc_t *sched_newproc();
thread_t *sched_newthread(void *ip, size_t kstacksize, int priority, proc_t *proc, void *ustack);

#endif
