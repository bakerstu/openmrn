#include "executor/notifiable.hxx"

#include "utils/macros.h"

static EmptyNotifiable default_empty_notifiable;

Notifiable* EmptyNotifiable::DefaultInstance() {
  return &default_empty_notifiable;
}

static CrashNotifiable default_crash_notifiable;

Notifiable* CrashNotifiable::DefaultInstance() {
  return &default_crash_notifiable;
}

void CrashNotifiable::notify() { DIE("Called CrashNotifiable."); }

BarrierNotifiable* BarrierNotifiable::new_child() {
  LockHolder h(this);
  count_++;
  return this;
}

void BarrierNotifiable::notify() {
  unsigned new_value;
  {
    LockHolder h(this);
    new_value = --count_;
  }
  if (!new_value) {
    HASSERT(done_);
    done_->notify();
  }
}

BarrierNotifiable::~BarrierNotifiable() { HASSERT(!count_); }

BarrierNotifiable* BarrierNotifiable::reset(Notifiable* done) {
  LockHolder h(this);
  HASSERT(!count_);
  count_ = 1;
  done_ = done;
  return this;
}
