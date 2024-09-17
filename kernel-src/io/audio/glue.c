#include <uhda/kernel_api.h>
#include <kernel/pci.h>
#include <kernel/alloc.h>
#include <kernel/pmm.h>
#include <logging.h>
#include <spinlock.h>

UhdaStatus uhda_kernel_pci_read(void *pci_device, uint8_t offset, uint8_t size, uint32_t *res) {
	pcienum_t *dev = pci_device;

	switch (size) {
		case 1: {
			*res = pci_read8(dev->bus, dev->device, dev->function, offset);
			break;
		}
		case 2: {
			*res = pci_read16(dev->bus, dev->device, dev->function, offset);
			break;
		}
		case 4: {
			*res = pci_read32(dev->bus, dev->device, dev->function, offset);
			break;
		}
		default:
			return UHDA_STATUS_UNSUPPORTED;
	}

	return UHDA_STATUS_SUCCESS;
}

UhdaStatus uhda_kernel_pci_write(void *pci_device, uint8_t offset, uint8_t size, uint32_t value) {
	pcienum_t *dev = pci_device;

	switch (size) {
		case 1: {
			pci_write8(dev->bus, dev->device, dev->function, offset, value);
			break;
		}
		case 2: {
			pci_write16(dev->bus, dev->device, dev->function, offset, value);
			break;
		}
		case 4: {
			pci_write32(dev->bus, dev->device, dev->function, offset, value);
			break;
		}
		default:
			return UHDA_STATUS_UNSUPPORTED;
	}

	return UHDA_STATUS_SUCCESS;
}

typedef struct {
	UhdaIrqHandlerFn fn;
	void *arg;
} uhda_handler_t;

static void hda_irq(isr_t *isr, context_t *context) {
	uhda_handler_t *handler = isr->priv;
	handler->fn(handler->arg);
}

UhdaStatus uhda_kernel_pci_allocate_irq(
	void *pci_device,
	UhdaIrqHint hint,
	UhdaIrqHandlerFn fn,
	void *arg,
	void **opaque_irq) {
	pcienum_t *dev = pci_device;

	isr_t *isr = interrupt_allocate(hda_irq, ARCH_EOI, IPL_MAX);
	__assert(isr);

	uhda_handler_t *handler = alloc(sizeof(uhda_handler_t));
	__assert(handler);
	handler->fn = fn;
	handler->arg = arg;

	isr->priv = handler;

	// todo don't ignore hint
	if (dev->msix.exists) {
		size_t int_count = pci_initmsix(dev);
		__assert(int_count);

		pci_msixsetmask(dev, 1);
		pci_msixadd(dev, 0, INTERRUPT_IDTOVECTOR(isr->id), 1, 0);
	} else if (dev->msi.exists) {
		size_t int_count = pci_initmsi(dev, 1);
		__assert(int_count);

		pci_msisetbase(dev, INTERRUPT_IDTOVECTOR(isr->id), 1, 0);
	} else {
		printf("hda: controller doesn't support msi-x or msi\n");
		return UHDA_STATUS_UNSUPPORTED;
	}

	*opaque_irq = isr;

	return UHDA_STATUS_SUCCESS;
}

void uhda_kernel_pci_deallocate_irq(void *pci_device, void *opaque_irq) {
	isr_t *isr = opaque_irq;

	// todo free irq
	free(isr->priv);
}

void uhda_kernel_pci_enable_irq(void *pci_device, void *opaque_irq, bool enable) {
	pcienum_t *dev = pci_device;

	if (dev->msix.exists) {
		if (enable) {
			pci_msixsetmask(dev, 0);
		} else {
			pci_msixsetmask(dev, 1);
		}
	}
}

UhdaStatus uhda_kernel_pci_map_bar(void *pci_device, uint32_t bar, void **virt) {
	pcienum_t *dev = pci_device;

	pcibar_t info = pci_getbar(dev, (int)bar);
	*virt = (void *) info.address;
	return UHDA_STATUS_SUCCESS;
}

void uhda_kernel_pci_unmap_bar(void *pci_device, uint32_t bar, void *virt) {}

void* uhda_kernel_malloc(size_t size) {
	return alloc(size);
}

void uhda_kernel_free(void* ptr, size_t size) {
	if (!ptr) {
		return;
	}

	free(ptr);
}

void uhda_kernel_delay(uint32_t microseconds) {
	timespec_t start = timekeeper_timefromboot();
	while (timespec_diffus(start, timekeeper_timefromboot()) < microseconds) CPU_PAUSE();
}

void uhda_kernel_log(const char* str) {
	printf("hda: %s\n", str);
}

UhdaStatus uhda_kernel_allocate_physical(size_t size, uintptr_t* res) {
	void *phys = pmm_alloc(size / 0x1000, PMM_SECTION_DEFAULT);

	if (!phys) {
		return UHDA_STATUS_NO_MEMORY;
	}

	*res = (uintptr_t)phys;
	return UHDA_STATUS_SUCCESS;
}

void uhda_kernel_deallocate_physical(uintptr_t phys, size_t size) {
	pmm_free((void *)phys, size / 0x1000);
}

UhdaStatus uhda_kernel_map(uintptr_t phys, size_t size, void** virt) {
	void *mapping = vmm_map(
		NULL,
		size,
		VMM_FLAGS_PHYSICAL,
		ARCH_MMU_FLAGS_WRITE |
		ARCH_MMU_FLAGS_READ |
		ARCH_MMU_FLAGS_NOEXEC |
		ARCH_MMU_FLAGS_UC,
		(void *)phys);
	if (!mapping) {
		return UHDA_STATUS_NO_MEMORY;
	}

	*virt = mapping;
	return UHDA_STATUS_SUCCESS;
}

void uhda_kernel_unmap(void* virt, size_t size) {
	vmm_unmap(virt, size, 0);
}

UhdaStatus uhda_kernel_create_spinlock(void **spinlock) {
	spinlock_t *lock = alloc(sizeof(spinlock_t));

	if (lock == NULL) {
		return UHDA_STATUS_NO_MEMORY;
	}

	SPINLOCK_INIT(*lock);
	*spinlock = lock;
	return UHDA_STATUS_SUCCESS;
}

void uhda_kernel_free_spinlock(void *spinlock) {
	free(spinlock);
}

UhdaIrqState uhda_kernel_lock_spinlock(void *spinlock) {
	return spinlock_acquireirqclear(spinlock);
}

void uhda_kernel_unlock_spinlock(void* spinlock, UhdaIrqState irq_state) {
	spinlock_releaseirqrestore(spinlock, irq_state);
}
