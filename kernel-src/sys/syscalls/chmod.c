#include <kernel/syscalls.h>
#include <kernel/alloc.h>
#include <kernel/file.h>
#include <kernel/vfs.h>
#include <arch/cpu.h>
#include <kernel/auth.h>

// node expected to be locked
static int dochmod(vnode_t *node, mode_t mode) {
	vattr_t attr;
	cred_t *cred = &current_thread()->proc->cred;

	int e = auth_filesystem_check(cred, AUTH_ACTIONS_FILESYSTEM_SETATTR, node, NULL);
	if (e)
		return e;

	e = VOP_GETATTR(node, &attr, cred);
	if (e)
		return e;

	attr.mode = mode;

	return VOP_SETATTR(node, &attr, V_ATTR_MODE, cred);
}

syscallret_t syscall_fchmodat(context_t *, int dirfd, char *upath, mode_t mode, int flags) {
	syscallret_t ret = {
		.ret = -1
	};

	size_t pathlen;
	ret.errno = usercopy_strlen(upath, &pathlen);
	if (ret.errno)
		return ret;

	char *path = alloc(pathlen + 1);
	if (path == NULL) {
		ret.errno = ENOMEM;
		return ret;
	}

	ret.errno = usercopy_fromuser(path, upath, pathlen);
	if (ret.errno) {
		free(path);
		return ret;
	}

	file_t *file = NULL;
	vnode_t *dirnode = NULL;
	vnode_t *node = NULL;
	ret.errno = dirfd_enter(path, dirfd, &file, &dirnode);
	if (ret.errno)
		goto cleanup;

	ret.errno = vfs_lookup(&node, dirnode, path, NULL, (flags & AT_SYMLINK_NOFOLLOW) ? VFS_LOOKUP_NOLINK : 0);
	if (ret.errno)
		goto cleanup;

	ret.errno = dochmod(node, mode);
	ret.ret = ret.errno ? -1 : 0;

	// locked by vfs_lookup
	VOP_UNLOCK(node);

	cleanup:

	if (node)
		VOP_RELEASE(node);

	if (dirnode)
		dirfd_leave(dirnode, file);

	free(path);

	return ret;
}

syscallret_t syscall_fchmod(context_t *, int fd, mode_t mode) {
	syscallret_t ret = {
		.ret = -1
	};

	file_t *file = fd_get(fd);
	if (file == NULL) {
		ret.errno = EBADF;
		return ret;
	}

	VOP_LOCK(file->vnode);
	ret.errno = dochmod(file->vnode, mode);
	ret.ret = ret.errno ? -1 : 0;
	VOP_UNLOCK(file->vnode);

	fd_release(file);

	return ret;
}
