/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file TempFile.hxx
 *
 * Helper classes for creating temporary files for testing and mock
 * implementations.
 *
 * @author Balazs Racz
 * @date 22 Mar 2015
 */

#ifndef _OS_TEMPFILE_HXX_
#define _OS_TEMPFILE_HXX_

#include <string>

#include <fcntl.h>
#include "utils/logging.h"

/** This class creates a temporary directory for the test, and removes it when
 * the test is done.  The caller is responsible for creating and removing the
 * files in this directory. 
 *
 * The temporary directory will be under the current working directory (usually
 * wherever the test is running). */
class TempDir {
public:
  TempDir() {
#ifdef __linux__
    dirName_ = "/tmp/openmrntmpdirXXXXXX";
#else
    dirName_ = "./openmrntmpdirXXXXXX";
#endif
    dirName_.c_str();
    HASSERT(mkdtemp(&dirName_[0]));
  }

  ~TempDir() {
    if (rmdir(dirName_.c_str()) != 0) {
      LOG(WARNING, "Error deleting temporary directory %s: %s",
          dirName_.c_str(), strerror(errno));
    }
  }

  static TempDir* instance() {
    static TempDir me;
    return &me;
  }

  const string& name() const {
    return dirName_;
  }

private:
  string dirName_;
};

/** This class creates a temporary file for the test, and removes it when the
 * test is done. */
class TempFile {
public:
  TempFile(const TempDir& dir, const string& basename) {
    fileName_ = dir.name() + "/" + basename + ".XXXXXX";
    fileName_.c_str();
    fd_ = mkstemp((char*)fileName_.c_str());
  }

  ~TempFile() {
    ::close(fd_);
    ::unlink(fileName_.c_str());
  }

  const string& name() const {
    return fileName_;
  }

  int fd()
  {
      return fd_;
  }

  /// writes a single byte to the temporary file.
  void write(const uint8_t byte) {
    string s;
    s.push_back(byte);
    write(s);
  }

  /// writes the given data to the temporary file.
  void write(const string& s) {
    size_t ofs = 0;
    while (ofs < s.size()) {
      int ret = ::write(fd_, s.data() + ofs, s.size() - ofs);
      HASSERT(ret >= 0);
      ofs += ret;
    }
    fsync(fd_);
  }

private:
  string fileName_;
  int fd_;
};

#endif
