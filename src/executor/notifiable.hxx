#ifndef _EXECUTOR_NOTIFIABLE_HXX_
#define _EXECUTOR_NOTIFIABLE_HXX_

#include "os/OS.hxx"
#include "executor/lock.hxx"

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

  virtual void Notify() { sem_.post(); }

  void WaitForNotification() { sem_.wait(); }

 private:
  DISALLOW_COPY_AND_ASSIGN(SyncNotifiable);
  OSSem sem_;
};

class EmptyNotifiable : public Notifiable {
 public:
  virtual void Notify() {}

  static Notifiable* DefaultInstance();
};

// A BarrierNotifiable allows to create a number of child Notifiable and wait
// for all of them to finish. When the last one is finished, the parent done
// callback is called.
class BarrierNotifiable : public Notifiable, private Lockable {
 public:
  BarrierNotifiable(Notifiable* done) : count_(1), done_(done) {}
  ~BarrierNotifiable();

  // Call this for each child task.
  Notifiable* NewChild();
  // When there are no more child tasks to add, call MaybeDone. Then once all
  // previously added child tasks are done, the parent callback will be
  // called. If you haven't added any children, this will call the parent
  // callback inline.
  void MaybeDone() { Notify(); }
  virtual void Notify();

 private:
  unsigned count_;
  Notifiable* done_;
};

inline BarrierNotifiable* NewBarrierNotifiable(Notifiable* done) {
  return new BarrierNotifiable(done);
}

#endif  // _EXECUTOR_NOTIFIABLE_HXX_
