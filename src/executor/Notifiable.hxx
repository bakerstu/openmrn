#ifndef _EXECUTOR_NOTIFIABLE_HXX_
#define _EXECUTOR_NOTIFIABLE_HXX_

#include "os/OS.hxx"
#include "utils/Atomic.hxx"
#include "utils/Queue.hxx"

/// An object that can schedule itself on an executor to run.
class Notifiable
{
public:
    virtual void notify() = 0;
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

    virtual void notify()
    {
        sem_.post();
    }

    /* Blocks the current thread until the notification is delivered. */
    void wait_for_notification()
    {
        sem_.wait();
    }

private:
    OSSem sem_;

    DISALLOW_COPY_AND_ASSIGN(SyncNotifiable);
};

class EmptyNotifiable : public Notifiable
{
public:
    virtual void notify()
    {
    }

    static Notifiable* DefaultInstance();
};

// This notifiable will crash whenever called.
class CrashNotifiable : public Notifiable
{
public:
    virtual void notify();

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
    /// Creates a new callback. When this callback is called, the parent
    /// notifiable is called and HasBeenNotified() will return true after that
    /// point. This function must not be called again until the returned
    // callback
    /// is invoked.
    //
    /// @returns a Notifiable to be used as a done callback for some
    // asynchronous
    /// processing.
    Notifiable* NewCallback(Notifiable* parent)
    {
        HASSERT(!parent_);
        parent_ = parent;
        return this;
    }
    /// @Returns true if the Notifiable returned by NewCallback has already been
    /// called.
    bool HasBeenNotified()
    {
        return !parent_;
    };

private:
    /// Implementation of the private Notifiable interface.
    virtual void notify()
    {
        Notifiable* p = parent_;
        HASSERT(p);
        parent_ = nullptr;
        p->notify();
    }

    Notifiable* parent_;
};

// A BarrierNotifiable allows to create a number of child Notifiable and wait
// for all of them to finish. When the last one is finished, the parent done
// callback is called.
class BarrierNotifiable : public Notifiable, private Lockable
{
public:
    /** Constructs a barrier notifiable that is done. Users should call reset()
     * later. */
    BarrierNotifiable() : count_(0), done_(nullptr)
    {
    }
    BarrierNotifiable(Notifiable* done) : count_(1), done_(done)
    {
    }

    // Resets the barrier. Returns &*this. Asserts that is_done().
    BarrierNotifiable* reset(Notifiable* done);

    ~BarrierNotifiable();

    // Call this for each child task.
    BarrierNotifiable* new_child();
    // When there are no more child tasks to add, call MaybeDone. Then once all
    // previously added child tasks are done, the parent callback will be
    // called. If you haven't added any children, this will call the parent
    // callback inline.
    void maybe_done()
    {
        notify();
    }
    virtual void notify();

    // Returns true if the barrier condition is true, i.e., the owner has clled
    // MaybeDone and all children have called Done.
    bool is_done()
    {
        return !count_;
    }

private:
    unsigned count_;
    Notifiable* done_;
};

inline BarrierNotifiable* NewBarrierNotifiable(Notifiable* done)
{
    return new BarrierNotifiable(done);
}

/** This class sends a notification in its destructor. Use as RAII class:
 *
 * bool DoFoo(Notifiable* done)
 * {
 *    AutoNotify n(done);
 *    // ... doo stuuufff ...
 *    if (something_wrong) return false;
 *    // do more stuff
 *    return true;
 * }
 *
 * The notification will be called on all return statements. */
class AutoNotify
{
public:
    AutoNotify(Notifiable* n) : n_(n)
    {
    }

    ~AutoNotify()
    {
        if (n_)
        {
            n_->notify();
        }
    }

    /* Transfers the ownership of the notification; it will NOT be called in
     * the destructor. The caller is now responsible for calling it.
     * @returns the notification pointer stored in the constructor. */
    Notifiable* Transfer()
    {
        Notifiable* r = n_;
        n_ = nullptr;
        return r;
    }

private:
    Notifiable* n_;
};

#define AutoNotify(l) int error_omitted_autonotify_holder_variable[-1]


#endif // _EXECUTOR_NOTIFIABLE_HXX_
