#include <kernel/syscalls.h>
#include <arch/signal.h>
#include <arch/cpu.h>
#include <logging.h>

#ifdef __x86_64__
#include <arch/msr.h>
#endif

// this expects CTX_SP to be pointing to the signal frame
__attribute__((noreturn)) void syscall_sigreturn(context_t *context) {
	sigframe_t sigframe;
	int error = usercopy_fromuser(&sigframe, (void *)CTX_SP(context), sizeof(sigframe_t));
	if (error || ARCH_CONTEXT_ISUSER(&sigframe.context) == false) {
		printf("syscall_sigreturn: bad return stack or bad return information\n");
		proc_terminate(SIGSEGV);
	}

	interrupt_set(false);
	signal_altstack(current_thread(), &sigframe.oldstack, NULL);
	signal_changemask(current_thread(), SIG_SETMASK, &sigframe.oldmask, NULL);

	arch_sigframe_get_context_from_mcontext(&sigframe);

	__assert(ARCH_CONTEXT_ISUSER(&sigframe.context));

	#ifdef __x86_64__
	// XXX is this behavior correct? ucontext doesnt seem to save it on linux and so it will not be saved
	sigframe.extracontext.gsbase = rdmsr(MSR_KERNELGSBASE);
	sigframe.extracontext.fsbase = rdmsr(MSR_FSBASE);
	#endif
	memcpy(&current_thread()->extracontext, &sigframe.extracontext, sizeof(extracontext_t));
	ARCH_CONTEXT_THREADLOAD(current_thread(), context);

	interrupt_set(true);
	arch_context_switch(&sigframe.context);

	// if we were to return normally, CTX_RET and CTX_ERRNO would be corrupted,
	// so we will manually call arch_context_switch, which will jump to the right userland code
	// (and also check for pending signals again)
	__builtin_unreachable();
}
