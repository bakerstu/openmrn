/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file FileSystem.cxx
 * This file imlements Device level methods of the device file system.
 *
 * @author Stuart W. Baker
 * @date 31 January 2015
 */

#include "Devtab.hxx"

#include <fcntl.h>
#include <unistd.h>

FileSystem *FileSystem::first = NULL;

/** Constructor.
 */
FileSystem::FileSystem()
    : FileIO(nullptr)
{
    mutex.lock();
    next = first;
    first = this;
    prev = NULL;
    if (next)
    {
        next->prev = this;
    }
    mutex.unlock();
}

/** Destructor.
 */
FileSystem::~FileSystem()
{
    mutex.lock();
    if (first == this)
    {
        first = next;
    }
    else
    {
        prev->next = next;
    }
    if (next)
    {
        next->prev = prev;
    }
    mutex.unlock();
}

/** Locate the file system for a given path.
 * @param path full path to file/directory
 * @return reference to file system on success, else nullptr
 */
FileSystem *FileSystem::fs_lookup(const char *path)
{
    for (FileSystem *fs = first; fs != NULL; fs = fs->next)
    {
        if (fs->name == nullptr)        
        {
            /* mount path has no name, probably not mounted yet */
            continue;
        }
        size_t fs_name_len = strlen(fs->name);
        if (fs_name_len > strlen(path))
        {
            /* path less than mount path */
            continue;
        }
        if (strncmp(fs->name, path, fs_name_len))
        {
            /* no basename match */
            continue;
        }
        if (path[fs_name_len] != '/' && strcmp(fs->name, path))
        {
            /* not a directory break or exact name match*/
            continue;
        }
        return fs;
    }

    return nullptr;
}

/** Open a file or device.
 * @param reent thread save reentrant structure
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode, ignored in this implementation
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::open(struct _reent *reent, const char *path, int flags, int mode)
{
    mutex.lock();
    int fd = fd_alloc();
    mutex.unlock();
    if (fd < 0)
    {
        errno = EMFILE;
        return -1;
    }
    files[fd].flags = flags;

    FileSystem *fs = fs_lookup(path);

    if (!fs)
    {
        // No device found.
        fd_free(fd);
        errno = ENODEV;
        return -1;
    }

    files[fd].dev = fs;
    files[fd].device = false;
    const char *subpath = path + strlen(fs->name) + 1;
    int result = files[fd].dev->open(&files[fd], subpath, flags, mode);
    if (result < 0)
    {
        fd_free(fd);
        errno = -result;
        return -1;
    }
    return fd;
}

/** Close a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to close
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::close(struct _reent *reent, int fd)
{
    File* f = file_lookup(fd);
    if (!f) 
    {
        /* errno should already be set appropriately */
        return -1;
    }
    if (fd >=0 && fd <= 2)
    {
        // stdin, stdout, and stderr never get closed
        return 0;
    }
    int result = f->dev->close(f);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    fd_free(fd);
    return 0;
}

/** Remove a file.
 * @param reent thread safe reentrant structure
 * @param path file name
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::unlink(struct _reent *reent, const char *path)
{
    FileSystem *fs = fs_lookup(path);
    if (!fs)
    {
        errno = ENOENT;
        return -1;
    }

    const char *subpath = path + strlen(fs->name) + 1;
    int result = fs->unlink(subpath);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return 0;
}

/** Get the status information of a file or device.
 * @param reent thread save reentrant structure
 * @param path file or device name
 * @param stat structure to fill status info into
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::stat(struct _reent *reent, const char *path, struct stat *stat)
{
    FileSystem *fs = fs_lookup(path);
    if (!fs)
    {
        errno = ENOENT;
        return -1;
    }

    const char *subpath = path + strlen(fs->name) + 1;
    int result = fs->stat(subpath, stat);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return 0;
}

/** Get the status information of a file or device.
 * @param file file reference for this device
 * @param stat structure to fill status info into
 * @return 0 upon successor or negative error number upon error.
 */
int FileSystem::fstat(File* file, struct stat *stat)
{
    memset(stat, 0, sizeof(*stat));
    return 0;
}

/** Synchronize (flush) a file to disk.
 * @param fd file descriptor to sync
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::fsync(int fd)
{
    File *file = file_lookup(fd);
    HASSERT(!file->device);

    if (!file)
    {
        errno = EBADF;
        return -1;
    }

    int result = static_cast<FileSystem*>(file->dev)->fsync(file);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;
}

/*
 * FileSystem::closedir()
 */
int FileSystem::closedir(DIR *dirp)
{
    File *file = (File*)dirp;
    FileSystem *fs = static_cast<FileSystem*>(file->dev);

    if (!file->dir)
    {
        errno = EBADF;
        return -1;
    }
    int result = fs->closedir(file);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;
}

/*
 * FileSystem::opendir()
 */
DIR *FileSystem::opendir(const char *name)
{
    mutex.lock();
    int fd = fd_alloc();
    mutex.unlock();
    if (fd < 0)
    {
        errno = EMFILE;
        return NULL;
    }
    files[fd].flags = 0;
    files[fd].dir = true;

    FileSystem *fs = fs_lookup(name);
    if (fs)
    {
        files[fd].dev = fs;
        files[fd].device = false;
        const char *subpath = name + strlen(fs->name) + 1;
        DIR *result = (DIR*)fs->opendir(&files[fd], subpath);
        if (result)
        {
            return result;
        }
    }
    // No device found.
    fd_free(fd);
    errno = ENOENT;
    return NULL;
}

/*
 * FileSystem::readdir()
 */
struct dirent *FileSystem::readdir(DIR *dirp)
{
    File *file = (File*)dirp;
    FileSystem *fs = static_cast<FileSystem*>(file->dev);

    HASSERT(file->dir);

    return fs->readdir(file);
}


#ifdef __cplusplus
extern "C" {
#endif

/*
 * closedir()
 */
int closedir(DIR *dirp)
{
    return FileSystem::closedir(dirp);
}

/*
 * opendir()
 */
DIR *opendir(const char *name)
{
    return FileSystem::opendir(name);
}

/*
 * readdir()
 */
struct dirent *readdir(DIR *dirp)
{
    return FileSystem::readdir(dirp);
}

#ifdef __cplusplus
}
#endif

