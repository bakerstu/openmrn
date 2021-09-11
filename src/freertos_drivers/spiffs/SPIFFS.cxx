/** @copyright
 * Copyright (c) 2018, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file SPIFFS.cxx
 * This file implements the generic SPIFFS driver.
 *
 * @author Stuart W. Baker
 * @date 1 January 2018
 */

#include "SPIFFS.hxx"

#include <fcntl.h>

#include "spiffs.h"
#include "spiffs_nucleus.h"

#ifndef _FDIRECT
#define _FDIRECT 0x80000
#endif

#ifndef O_DIRECT
#define O_DIRECT _FDIRECT
#endif

void SPIFFS::extern_lock(struct spiffs_t *fs)
{
    static_cast<SPIFFS *>(fs->user_data)->lock_.lock();
}

void SPIFFS::extern_unlock(struct spiffs_t *fs)
{
    static_cast<SPIFFS *>(fs->user_data)->lock_.unlock();
}

extern "C"
{
/// global error number for the last SPIFFS error
int globalLastSPIFFSError;

/// Provide mutex lock.
/// @param fs reference to the file system instance
void extern_spiffs_lock(struct spiffs_t *fs)
{
    SPIFFS::extern_lock(fs);
}

/// Provide mutex unlock.
/// @param fs reference to the file system instance
void extern_spiffs_unlock(struct spiffs_t *fs)
{
    SPIFFS::extern_unlock(fs);
}
} // extern "C"

// static
int SPIFFS::flash_read(
    struct spiffs_t *fs, unsigned addr, unsigned size, uint8_t *dst)
{
    return static_cast<SPIFFS *>(fs->user_data)->flash_read(addr, size, dst);
}

// static
int SPIFFS::flash_write(
    struct spiffs_t *fs, unsigned addr, unsigned size, uint8_t *src)
{
    return static_cast<SPIFFS *>(fs->user_data)->flash_write(addr, size, src);
}

// static
int SPIFFS::flash_erase(struct spiffs_t *fs, unsigned addr, unsigned size)
{
    return static_cast<SPIFFS *>(fs->user_data)->flash_erase(addr, size);
}

//
// SPIFFS::SPIFFS()
//
SPIFFS::SPIFFS(size_t physical_address, size_t size_on_disk,
               size_t erase_block_size, size_t logical_block_size,
               size_t logical_page_size, size_t max_num_open_descriptors,
               size_t cache_pages,
               std::function<void()> post_format_hook)
    : FileSystem()
    , postFormatHook_(std::move(post_format_hook))
    , lock_()
    , workBuffer_(new uint8_t[logical_page_size * 2])
    , fdSpaceSize_(max_num_open_descriptors * sizeof(spiffs_fd))
    , fdSpace_(new uint8_t[fdSpaceSize_])
    , cacheSize_(sizeof(spiffs_cache) +
                 cache_pages * (sizeof(spiffs_cache_page) + logical_page_size))
    , cache_(new uint8_t[cacheSize_])
    , formatted_(false)
    , anyDirty_(false)
{
    fs_ = new spiffs;
    memset(fs_, 0, sizeof(spiffs));
    fs_->user_data = this;
    spiffs_config tmp{  //
        .hal_read_f       = flash_read,
        .hal_write_f      = flash_write,
        .hal_erase_f      = flash_erase,
        .phys_size        = size_on_disk,
        .phys_addr        = physical_address,
        .phys_erase_block = erase_block_size,
        .log_block_size   = logical_block_size,
        .log_page_size    = logical_page_size};
    memcpy(&fs_->cfg, &tmp, sizeof(fs_->cfg));
}

//
// Destructor
//
SPIFFS::~SPIFFS()
{
    // Performing unmount in the destructor of the base class is not
    // possible, because the virtual functions for reading and writing the
    // flash cannot be called anymore.
    HASSERT(SPIFFS_mounted(fs_) == 0);
    delete[] fdSpace_;
    delete[] workBuffer_;
    delete fs_;
}

//
// SPIFFS::unmount
//
void SPIFFS::unmount()
{
    if (name)
    {
        SPIFFS_unmount(fs_);
        name = nullptr;
    }
}

///
/// Open directory metadata structure
///
struct SPIFFS::OpenDir
{
    spiffs_DIR dir_;       ///< directory object
    struct dirent dirent_; ///< directory entry
};

//
// SPIFFS::format()
//
void SPIFFS::format()
{
    HASSERT(SPIFFS_mounted(fs_) == 0);

    // formatting requires at least one mounting, so mount then unmount
    if (do_mount() == 0)
    {
        SPIFFS_unmount(fs_);
    }

    HASSERT(SPIFFS_format(fs_) == 0);
    formatted_ = true;
}

//
// SPIFFS::do_mount()
//
int SPIFFS::do_mount()
{
    spiffs_config tmp;
    memcpy(&tmp, &fs_->cfg, sizeof(tmp));
    return SPIFFS_mount(fs_, &tmp, workBuffer_, fdSpace_,
                        fdSpaceSize_, cache_, cacheSize_, nullptr);
}

//
// SPIFFS::open()
//
int SPIFFS::open(File *file, const char *path, int flags, int mode)
{
    spiffs_flags ffs_flags = 0;
    if (flags & O_APPEND)
    {
        ffs_flags |= SPIFFS_O_APPEND;
    }
    if (flags & O_TRUNC)
    {
        ffs_flags |= SPIFFS_O_TRUNC;
    }
    if (flags & O_CREAT)
    {
        ffs_flags |= SPIFFS_O_CREAT;
    }
    if ((flags & O_ACCMODE) == O_RDONLY)
    {
        ffs_flags |= SPIFFS_O_RDONLY;
    }
    if ((flags & O_ACCMODE) == O_WRONLY)
    {
        ffs_flags |= SPIFFS_O_WRONLY;
    }
    if ((flags & O_ACCMODE) == O_RDWR)
    {
        ffs_flags |= SPIFFS_O_RDWR;
    }
    if (flags & O_DIRECT)
    {
        ffs_flags |= SPIFFS_O_DIRECT;
    }
    if (flags & O_EXCL)
    {
        ffs_flags |= SPIFFS_O_EXCL;
    }

    spiffs_file fd = ::SPIFFS_open(fs_, path, ffs_flags, 0);

    if (fd < 0)
    {
        return -errno_translate(fd);
    }
    else
    {
        // no error occured
        file->privInt = fd;
        return 0;
    }
}

//
// SPIFFS::close()
//
int SPIFFS::close(File *file)
{
    spiffs_file fd = file->privInt;

    file->dirty = false;
    int result = SPIFFS_close(fs_, fd);

    if (result != SPIFFS_OK)
    {
        return -errno_translate(result);
    }
    
    return 0;
}

//
// SPIFFS::unlink()
//
int SPIFFS::unlink(const char *path)
{
    int result = SPIFFS_remove(fs_, path);

    if (result < 0)
    {
        return -errno_translate(result);
    }

    return 0;
}

//
// SPIFFS::read()
//
ssize_t SPIFFS::read(File *file, void *buf, size_t count)
{
    spiffs_file fd = file->privInt;

    ssize_t result = SPIFFS_read(fs_, fd, buf, count);

    if (result < 0)
    {
        return -errno_translate(result);
    }

    return result;
}

//
// SPIFFS::write()
//
ssize_t SPIFFS::write(File *file, const void *buf, size_t count)
{
    spiffs_file fd = file->privInt;

    ssize_t result = SPIFFS_write(fs_, fd, (void *)buf, count);

    if (result < 0)
    {
        return -errno_translate(result);
    }

    {
        AtomicHolder h(this);
        file->dirty = true;
        anyDirty_ = true;
    }
    
    return result;
}

//
// SPIFFS::lseek()
//
off_t SPIFFS::lseek(File* file, off_t offset, int whence)
{
    spiffs_file fd = file->privInt;
    int spiffs_whence;

    switch (whence)
    {
        default:
            return (off_t)-EINVAL;
        case SEEK_SET:
            spiffs_whence = SPIFFS_SEEK_SET;
            break;
        case SEEK_CUR:
            spiffs_whence = SPIFFS_SEEK_CUR;
            break;
        case SEEK_END:
            spiffs_whence = SPIFFS_SEEK_END;
            break;
    }

    off_t result = SPIFFS_lseek(fs_, fd, offset, spiffs_whence);

    if (result < 0)
    {
        return -errno_translate(result);
    }

    return result;
}

/// Common post processing for SPIFFS::stat() and SPIFFS::fstat().
/// @param stat structure to fill status info into
/// @param ff_stat SPIFFS structure to gather info from
static void stat_post_process(struct stat *stat, spiffs_stat *ffs_stat)
{
    memset(stat, 0, sizeof(*stat));
    stat->st_ino = ffs_stat->obj_id;
    stat->st_size = ffs_stat->size;
    switch (ffs_stat->type)
    {
        default:
            break;
        case SPIFFS_TYPE_FILE:
            stat->st_mode = S_IFREG;
            break;
        case SPIFFS_TYPE_DIR:
            stat->st_mode = S_IFDIR | S_IXUSR | S_IXGRP | S_IXOTH;
            break;
        case SPIFFS_TYPE_HARD_LINK:
            stat->st_mode = S_IFLNK | S_IXUSR | S_IXGRP | S_IXOTH;
            break;
        case SPIFFS_TYPE_SOFT_LINK:
            stat->st_mode = S_IFLNK | S_IXUSR | S_IXGRP | S_IXOTH;
            break;
    }

    stat->st_mode |= S_IRUSR | S_IRGRP | S_IROTH |
                     S_IWUSR | S_IWGRP | S_IWOTH;

}

//
// SPIFFS::fstat()
//
int SPIFFS::fstat(File* file, struct stat *stat)
{
    spiffs_file fd = file->privInt;
    spiffs_stat ffs_stat;

    ssize_t result = SPIFFS_fstat(fs_, fd, &ffs_stat);

    if (result < 0)
    {
        return -errno_translate(result);
    }

    stat_post_process(stat, &ffs_stat);

    return 0;
}

//
// SPIFFS::stat()
//
int SPIFFS::stat(const char *path, struct stat *stat)
{
    spiffs_stat ffs_stat;

    if (!strcmp(path, "") || !strcmp(path, "/"))
    {
        // this is the root directory, which cannot be stat
        ffs_stat.type = SPIFFS_TYPE_DIR;
        ffs_stat.obj_id = 0;
        ffs_stat.size = 0;
    }
    else
    {
        ssize_t result = SPIFFS_stat(fs_, path, &ffs_stat);

        if (result < 0)
        {
            return -errno_translate(result);
        }
    }

    stat_post_process(stat, &ffs_stat);

    return 0;
}

//
// SPIFFS::fsync()
//
int SPIFFS::fsync(File *file)
{
    spiffs_file fd = file->privInt;
    file->dirty = false;
    int result = SPIFFS_fflush(fs_, fd);
    if (result < 0)
    {
        file->dirty = true;
        return -errno_translate(result);
    }
    return 0;
}

//
// SPIFFS::closedir()
//
int SPIFFS::closedir(File *file)
{
    OpenDir *dir = static_cast<OpenDir*>(file->priv);
    int result = SPIFFS_closedir(&dir->dir_);

    if (result == 0)
    {
        free(dir);
        return 0;
    }
    else
    {
        return -errno_translate(result);
    }
}

//
// SPIFFS::opendir()
//
File *SPIFFS::opendir(File *file, const char *name)
{
    // special malloc is to reserve space for the full path name
    OpenDir *dir = static_cast<OpenDir*>(malloc(sizeof(OpenDir) +
                                                SPIFFS_OBJ_NAME_LEN +
                                                strlen(this->name) + 1));

    extern_lock(fs_);
    spiffs_DIR *result = SPIFFS_opendir(fs_, name, &dir->dir_);
    if (!result)
    {
        free(dir);
        errno = errno_translate(fs_->err_code);
    }
    else
    {
        // no error occured
        file->priv = dir;
        file->dir = true;
    }
    extern_unlock(fs_);

    return file;
}

//
// SPIFFS::readdir()
//
struct dirent *SPIFFS::readdir(File *file)
{
    OpenDir *dir = static_cast<OpenDir*>(file->priv);
    spiffs_dirent dirent;

    spiffs_dirent *result = SPIFFS_readdir(&dir->dir_, &dirent);

    if (!result)
    {
        return nullptr;
    }
    else
    {
        dir->dirent_.d_ino = dirent.obj_id;
        strcpy(dir->dirent_.d_name, this->name);
        strcat(dir->dirent_.d_name, "/");
        strcat(dir->dirent_.d_name, (char*)dirent.name);
        return &dir->dirent_;
    }
}

//
// SPIFFS::errno_translate()
//
int SPIFFS::errno_translate(int spiffs_error)
{
    globalLastSPIFFSError = spiffs_error;

    switch (spiffs_error)
    {
        default:
            // unknown error
            HASSERT(0);
            break;
        case SPIFFS_OK:
            return 0;
        case SPIFFS_ERR_NOT_MOUNTED:
            // should never get here
            HASSERT(0);
            break;
        case SPIFFS_ERR_FULL:
            return ENOSPC;
        case SPIFFS_ERR_NOT_FOUND:
            return ENOENT;
        case SPIFFS_ERR_END_OF_OBJECT:
            return EOVERFLOW;
        case SPIFFS_ERR_DELETED:
            return EFAULT;
        case SPIFFS_ERR_NOT_FINALIZED:
            return EBUSY;
        case SPIFFS_ERR_NOT_INDEX:
            return EINVAL;
        case SPIFFS_ERR_OUT_OF_FILE_DESCS:
            return EMFILE;
        case SPIFFS_ERR_FILE_CLOSED:
            return EBADF;
        case SPIFFS_ERR_FILE_DELETED:
            return EFAULT;
        case SPIFFS_ERR_BAD_DESCRIPTOR:
            return EBADF;
        case SPIFFS_ERR_IS_INDEX:
            break;
        case SPIFFS_ERR_IS_FREE:
            return EBUSY;
        case SPIFFS_ERR_INDEX_SPAN_MISMATCH:
            break;
        case SPIFFS_ERR_DATA_SPAN_MISMATCH:
            break;
        case SPIFFS_ERR_INDEX_REF_FREE:
            break;
        case SPIFFS_ERR_INDEX_REF_LU:
            break;
        case SPIFFS_ERR_INDEX_REF_INVALID:
            break;
        case SPIFFS_ERR_INDEX_FREE:
            break;
        case SPIFFS_ERR_INDEX_LU:
            break;
        case SPIFFS_ERR_INDEX_INVALID:
            return EINVAL;
        case SPIFFS_ERR_NOT_WRITABLE:
            return EACCES;
        case SPIFFS_ERR_NOT_READABLE:
            return EACCES;
        case SPIFFS_ERR_CONFLICTING_NAME:
            break;
        case SPIFFS_ERR_NOT_CONFIGURED:
            break;

        case SPIFFS_ERR_NOT_A_FS:
            break;
        case SPIFFS_ERR_MOUNTED:
            break;
        case SPIFFS_ERR_ERASE_FAIL:
            break;
        case SPIFFS_ERR_MAGIC_NOT_POSSIBLE:
            break;

        case SPIFFS_ERR_NO_DELETED_BLOCKS:
            break;

        case SPIFFS_ERR_FILE_EXISTS:
            return EEXIST;

        case SPIFFS_ERR_NOT_A_FILE:
            return ENOENT;
        case SPIFFS_ERR_RO_NOT_IMPL:
            break;
        case SPIFFS_ERR_RO_ABORTED_OPERATION:
            return EAGAIN;
        case SPIFFS_ERR_PROBE_TOO_FEW_BLOCKS:
            return ENOSPC;
        case SPIFFS_ERR_PROBE_NOT_A_FS:
            break;
        case SPIFFS_ERR_NAME_TOO_LONG:
            return ENAMETOOLONG;

        case SPIFFS_ERR_IX_MAP_UNMAPPED:
            break;
        case SPIFFS_ERR_IX_MAP_MAPPED:
            break;
        case SPIFFS_ERR_IX_MAP_BAD_RANGE:
            break;
        case SPIFFS_ERR_SEEK_BOUNDS:
            return EINVAL;

        case SPIFFS_ERR_INTERNAL:
            break;

        case SPIFFS_ERR_TEST:
            break;
    }

    return EINVAL;
}
