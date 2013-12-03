#ifndef _EXECUTOR_NOTIFIABLE_HXX_
#define _EXECUTOR_NOTIFIABLE_HXX_

#include "os/OS.hxx"
#include "executor/lock.hxx"

//! An object that can schedule itself on an executor to run.
class Notifiable
{
public:
    virtual void Notify() = 0;
};

// A Notifiable for synchronously waiting for a notification.
// TODO(balazs.racz): We should make a syncnotifiable not need a semaphore
// of itself, but rather use a thread-local semaphore.
class SyncNotifiable : public Notifiable
{
public:
    SyncNotifiable() : sem_(0)
    {
    }

    virtual void Notify()
    {
        sem_.post();
    }

    void WaitForNotification()
    {
        sem_.wait();
    }

private:
    DISALLOW_COPY_AND_ASSIGN(SyncNotifiable);
    OSSem sem_;
};

class EmptyNotifiable : public Notifiable
{
public:
    virtual void Notify()
    {
    }

    static Notifiable* DefaultInstance();
};

// This notifiable will crash whenever called.
class CrashNotifiable : public Notifiable
{
public:
    virtual void Notify();

    static Notifiable* DefaultInstance();
};

/** A class for reliably detecting whether a Notification has happened yet or
 * not.
 *
 * ProxyNotifiable can give a Notifiable callback, proxy to a parent Notifiable
 * any calls that may come in, and tell through {@link HasBeenNotified} whether
 * such a callback has happened yet or not.
 *
 */

class ProxyNotifiable : private Notifiable
{
public:
    ProxyNotifiable() : parent_(nullptr)
    {
    }
    //! Creates a new callback. When this callback is called, the parent
    //! notifiable is called and HasBeenNotified() will return true after that
    //! point. This function must not be called again until the returned
    //callback
    //! is invoked.
    //
    //! @returns a Notifiable to be used as a done callback for some
    //asynchronous
    //! processing.
    Notifiable* NewCallback(Notifiable* parent)
    {
        HASSERT(!parent_);
        parent_ = parent;
        return this;
    }
    //! @Returns true if the Notifiable returned by NewCallback has already been
    //! called.
    bool HasBeenNotified()
    {
        return !parent_;
    };

private:
    //! Implementation of the private Notifiable interface.
    virtual void Notify()
    {
        Notifiable* p = parent_;
        HASSERT(p);
        parent_ = nullptr;
        p->Notify();
    }

    Notifiable* parent_;
};

// A BarrierNotifiable allows to create a number of child Notifiable and wait
// for all of them to finish. When the last one is finished, the parent done
// callback is called.
class BarrierNotifiable : public Notifiable, private Lockable
{
public:
    BarrierNotifiable() : count_(0), done_(nullptr)
    {
    }
    BarrierNotifiable(Notifiable* done) : count_(1), done_(done)
    {
    }
    void Reset(Notifiable* done);
    ~BarrierNotifiable();

    // Call this for each child task.
    Notifiable* NewChild();
    // When there are no more child tasks to add, call MaybeDone. Then once all
    // previously added child tasks are done, the parent callback will be
    // called. If you haven't added any children, this will call the parent
    // callback inline.
    void MaybeDone()
    {
        Notify();
    }
    virtual void Notify();

private:
    unsigned count_;
    Notifiable* done_;
};

inline BarrierNotifiable* NewBarrierNotifiable(Notifiable* done)
{
    return new BarrierNotifiable(done);
}

#endif // _EXECUTOR_NOTIFIABLE_HXX_
