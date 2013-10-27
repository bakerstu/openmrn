#ifndef _EXECUTOR_NOTIFIABLE_HXX_
#define _EXECUTOR_NOTIFIABLE_HXX_

#include "os/OS.hxx"

//! An object that can schedule itself on an executor to run.
class Notifiable {
public:
  virtual void Notify() = 0;
};

// A Notifiable for synchronously waiting for a notification.
// TODO(balazs.racz): We should make a syncnotifiable not need a semaphore
// of itself, but rather use a thread-local semaphore.
class SyncNotifiable : public Notifiable {
public:
  SyncNotifiable() : sem_(0) {}

  virtual void Notify() {
    sem_.post();
  }

  void WaitForNotification() {
    sem_.wait();
  }

private:
  DISALLOW_COPY_AND_ASSIGN(SyncNotifiable);
  OSSem sem_;
};

class EmptyNotifiable : public Notifiable {
public:
  virtual void Notify() {}

  static Notifiable* DefaultInstance();
};

#endif  // _EXECUTOR_NOTIFIABLE_HXX_
