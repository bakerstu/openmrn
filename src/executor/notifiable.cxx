#include "executor/notifiable.hxx"

#include "utils/macros.h"

static EmptyNotifiable default_empty_notifiable;

Notifiable* EmptyNotifiable::DefaultInstance() {
  return &default_empty_notifiable;
}

Notifiable* BarrierNotifiable::NewChild() {
  LockHolder h(this);
  count_++;
  return this;
}

void BarrierNotifiable::Notify() {
  unsigned new_value;
  {
    LockHolder h(this);
    new_value = --count_;
  }
  if (!new_value) {
    HASSERT(done_);
    done_->Notify();
  }
}

BarrierNotifiable::~BarrierNotifiable() {
  HASSERT(!count_);
}
