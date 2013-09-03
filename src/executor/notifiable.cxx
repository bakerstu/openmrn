#include "executor/notifiable.hxx"

static EmptyNotifiable default_empty_notifiable;

Notifiable* EmptyNotifiable::DefaultInstance() {
  return &default_empty_notifiable;
}
