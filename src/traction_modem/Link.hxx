/** @copyright
 * Copyright (c) 2025, Stuart Baker
 * All rights reserved
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
 * @file Link.hxx
 *
 * Implements link management and acts as a proxy to TxFlow and RxFlow.
 *
 * @author Stuart Baker
 * @date 26 May 2025
 */

#ifndef _TRACTION_MODEM_LINK_HXX_
#define _TRACTION_MODEM_LINK_HXX_

#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"

namespace traction_modem
{

/// Interface for link start and up/down status.
class LinkStatusInterface
{
protected:
    /// Called when the link is started.
    virtual void on_link_start()
    {
    }

    /// Called when link transitions to "up" state.
    virtual void on_link_up()
    {
    }

    /// Called when link transitions to "down" state.
    virtual void on_link_down()
    {
    }

    /// Protected access for Link object.
    friend class Link;
};

/// Link object that holds references to the TX and RX interfaces and tracks
/// link state.
class Link : private Atomic
{
public:
    /// Constructor
    /// @param tx_iface reference to the transmit interface
    /// @param rx_iface reference to the receive interface
    Link(TxInterface *tx_iface, RxInterface *rx_iface)
        : txIface_(tx_iface)
        , rxIface_(rx_iface)
        , state_(State::STOP)
    {
    }

    /// Check on the link status
    /// @return true if link is up, else link is down.
    bool is_link_up()
    {
        return state_ == State::UP;
    }

    /// Bind an interface to the TX and RX interfaces to start transmitting and
    /// receiving on.
    /// @param fd interface to transmit and receive messages on
    void start(int fd)
    {
        {
            AtomicHolder h(this);
            if (state_ != State::STOP)
            {
                // Already started.
                LOG_ERROR("Link already started.");
                return;
            }
            state_ = State::DOWN;
        }        
        txIface_->start(fd);
        rxIface_->start(fd);
        for (auto it = linkCbacks_.begin(); it != linkCbacks_.end(); ++it)
        {
            (*it)->on_link_start();
        }
    }

    /// Register for updates on link status.
    /// @param interface object to register
    void register_link_status(LinkStatusInterface *interface)
    {
        linkCbacks_.push_back(interface);
    }

    /// Register for updates on link status.
    /// @param interface object to register
    void unregister_link_status(LinkStatusInterface *interface)
    {
        for (auto it = linkCbacks_.begin(); it != linkCbacks_.end(); ++it)
        {
            if (interface == *it)
            {
                linkCbacks_.erase(it);
                break;
            }
        }
    }

    /// Get a reference to the link's transmit interface.
    TxInterface *get_tx_iface()
    {
        return txIface_;
    }

    /// Get a reference to the links receive interface.
    RxInterface *get_rx_iface()
    {
        return rxIface_;
    }

private:
    /// Link state.
    enum class State
    {
        STOP, ///< link not yet started
        DOWN, ///< link down
        UP, ///< link up
    };

    /// Called when link transitions to "up" state.
    void on_link_up()
    {
        state_ = State::UP;
        for (auto it = linkCbacks_.begin(); it != linkCbacks_.end(); ++it)
        {
            (*it)->on_link_up();
        }
        LOG(VERBOSE, "Link: on_link_up()");
    }

    /// Called when link transitions to "down" state.
    void on_link_down()
    {
        state_ = State::DOWN;
        for (auto it = linkCbacks_.begin(); it != linkCbacks_.end(); ++it)
        {
            (*it)->on_link_down();
        }
        LOG(VERBOSE, "Link: on_link_down()");
    }

    TxInterface *txIface_; ///< reference to the transmit interface interface
    RxInterface *rxIface_; ///< reference to the receive interface
    State state_; ///< link state

    /// List of interfaces that have registered for link updates.
    std::vector<LinkStatusInterface*> linkCbacks_;

    // Private access for LinkManager.
    friend class LinkManager;
};

/// Object that can manage the link, including negotiation of baud rate.
/// @todo Need a way to adjust the baud rate of the UART interface.
class LinkManager : public LinkStatusInterface
                  , public PacketFlowInterface
                  , public StateFlowBase
{
public:
    /// Constructor.
    /// @param service service that the flow is bound to
    /// @param link the link object that the manager is bound to
    /// @param use_default_baud the default baud rate of 250 Kbps will be used
    ///        and the baud rate negotiation will be skipped.
    LinkManager(Service *service, Link *link, bool use_default_baud = true)
        : StateFlowBase(service)
        , link_(link)
        , timer_(this)
        , pingTimer_(service->executor(), this)
        , baudSupport_(Defs::BAUD_NONE)
        , useDefaultBaud_(use_default_baud)
    {
        link_->register_link_status(this);
    }

#if defined(GTEST)
    /// Disable default baud setting to override constructor argument.
    void TEST_disable_default_baud()
    {
        useDefaultBaud_ = false;
    }

    /// Request a shutdown for concluding tests.
    void TEST_shutdown()
    {
        TEST_shutdown_ = true;
    }
private:
    /// Flag used in order to initiate the conclusion of tests.
    bool TEST_shutdown_{false};
#endif
private:
    /// Alias for short response timeout used during link establishment.
    static constexpr long long RESP_TIMEOUT = Defs::RESP_TIMEOUT_SHORT;

    /// Called when the link is started.
    void on_link_start() override
    {
        start_flow(STATE(ping));
    }

    /// Send a link establishment ping. Look for a pong response.
    /// @return nest state is pong(), wait for timeout or early trigger.
    Action ping()
    {
#if defined (GTEST)
        if (TEST_shutdown_)
        {
            return exit();
        }
#endif
        LOG(VERBOSE, "LinkManager::ping()");
        link_->get_rx_iface()->register_handler(this, Defs::RESP_PING);
        baudSupport_ = Defs::BAUD_NONE;
        Defs::Payload p = Defs::get_ping_payload();
        link_->get_tx_iface()->send_packet(p);
        return sleep_and_call(
            &timer_, RESP_TIMEOUT, STATE(baud_rate_query));
    }

    /// Either pong received or timed out. If pong received, send a baud
    /// rate query, wait for a baud rate query response.
    /// @return next state ping() on timeout, else link_is_up() if using the
    ///         default baud rate or baud_rate_request() if it should be
    ///         negotiated, wait for timeout or early trigger.
    Action baud_rate_query()
    {
        LOG(VERBOSE, "LinkManager::baud_rate_query()");
        link_->get_rx_iface()->unregister_handler(this, Defs::RESP_PING);
        if (!timer_.is_triggered())
        {
            // Timed out, waiting for response try again.
            return call_immediately(STATE(ping));
        }
        if (useDefaultBaud_)
        {
            // Do not negotiate the baud rate, stick with the default.
            return call_immediately(STATE(link_is_up));
        }

        link_->get_rx_iface()->register_handler(
            this, Defs::RESP_BAUD_RATE_QUERY);

        // Query for the supported baud rates.
        Defs::Payload p = Defs::get_baud_rate_query_payload();
        link_->get_tx_iface()->send_packet(p);
        return sleep_and_call(
            &timer_, RESP_TIMEOUT, STATE(baud_rate_request));
    }

    /// Either baud rate response received or timed out. If baud rate
    /// response received, either accept the current baud or transition
    /// directly to link up.
    /// @return next state ping() on timeout, else next state is
    ///         link_is_up(), Wait on timeout.
    Action baud_rate_request()
    {
        LOG(VERBOSE, "LinkManager::baud_rate_request()");
        link_->get_rx_iface()->unregister_handler(
            this, Defs::RESP_BAUD_RATE_QUERY);
        if (!timer_.is_triggered())
        {
            // Timed out, waiting for response try again.
            return call_immediately(STATE(ping));
        }

        HASSERT((baudSupport_ & Defs::BAUD_250K_MASK) != 0);
        /// @todo Need to decide if we should change the baud rate here
        ///       and restart the ping/pong process.

        // For now, we don't try to change the baud.
        return call_immediately(STATE(link_is_up));
    }

    /// Set the link state to "up".
    /// @return next state link_is_down() on timeout
    Action link_is_up()
    {
        LOG(VERBOSE, "LinkManager::link_is_up()");
        link_->on_link_up();
        link_->get_rx_iface()->register_handler(this, Defs::RESP_PING);
        pingTimer_.start(Defs::PING_TIMEOUT);
        return sleep_and_call(
            &timer_, Defs::RESP_TIMEOUT, STATE(link_is_down));
    }

    /// Link timed out. Transition directly to link down.
    /// @return next state ping()
    Action link_is_down()
    {
        LOG(VERBOSE, "LinkManager::link_is_down()");
        pingTimer_.ensure_triggered();
        link_->get_rx_iface()->unregister_handler(this, Defs::RESP_PING);
        link_->on_link_down();
        return call_immediately(STATE(ping));
    }

    /// Receive for link establishment responses.
    /// @param buf incoming message
    /// @param prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override
    {
        auto b = get_buffer_deleter(buf);
        switch(buf->data()->command())
        {
            case Defs::RESP_PING:
                if (link_->is_link_up())
                {
                    // Note: link_is_down() is technically the "next" state, if
                    //       the link fails (timer_ expires without pong).
                    HASSERT(is_state(STATE(link_is_down)));
                    // Everything is good, don't trigger an early timer_ wakeup.
                    timer_.restart();
                    return; 
                }
                break;
            case Defs::RESP_BAUD_RATE_QUERY:
            {
                Defs::BaudRateQueryResponse *brqr =
                    (Defs::BaudRateQueryResponse*)buf->data()->payload.data();
                baudSupport_ = be16toh(brqr->rates_);
                break;
            }
            case Defs::RESP_MEM_W:
                /// @todo There is a special case where we want to intercept
                ///       these packets during a baud rate integrity test.
                break;
        } 
        timer_.ensure_triggered();
    }

    /// Helper timer for sending pings during extended idle times.
    class PingTimer : public Timer
    {
    public:
        /// Constructor.
        /// @param e Executor to run on
        /// @param parent parent object
        PingTimer(ExecutorBase *e, LinkManager *parent)
            : Timer(e->active_timers())
            , parent_(parent)
        {
        }

        /// Timer expired.
        /// @return RESTART after sending a ping, NONE if triggered early.
        long long timeout() override
        {
            if (is_triggered())
            {
                // The link went down.
                return NONE;
            }
            Defs::Payload p = Defs::get_ping_payload();
            parent_->link_->get_tx_iface()->send_packet(p);
            return RESTART;
        }

    private:
        LinkManager *parent_; ///< parent object
    };

    Link *link_; ///< parent object
    StateFlowTimer timer_; ///< timeout helper
    PingTimer pingTimer_; ///< timer that periodically pings
    uint16_t baudSupport_; ///< mask of supported baud rates
    /// True in order to skip the baud rate negotiation and use the default
    /// 250 Kbps baud rate.
    bool useDefaultBaud_;
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_LINK_HXX_