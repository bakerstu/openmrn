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
#define _TRACTION_MODEM_LINK_HXX

#include "traction_modem/RxFlow.hxx"
#include "traction_modem/TxFlow.hxx"

namespace traction_modem
{

/// Interface for link up/down status.
class LinkInterface
{
protected:
    /// Called when link transitions to "up" state.
    virtual void link_up() = 0;

    /// Called when link transitions to "down" state.
    virtual void link_down() = 0;

    // Protected access for Link object.
    friend class Link;
};

/// Link management object that will act as a proxy for TxFlow and RxFlow.
/// @todo Need a way to adjust the baud rate of the UART interface.
class Link : public TxInterface
           , public RxInterface
           , public PacketFlowInterface
           , public LinkInterface
{
public:
    /// Constructor
    /// @param tx_flow reference to the transmit flow
    /// @param rx_flow reference to the receive flow
    /// @param service service that the flow is bound to
    Link(Service *service, TxInterface *tx_flow, RxInterface *rx_flow)
        : linkEstablishment_(service, this)
        , txFlow_(tx_flow)
        , rxFlow_(rx_flow)
        , pingTimer_(service->executor(), this)
        , state_(State::STOP)
        , dispatcher_(service)
    {
        rxFlow_->register_fallback_handler(this);
    }

    /// Check on the link status
    /// @return true if link is up, else link is down.
    bool is_link_up()
    {
        return state_ == State::UP;
    }

    /// Bind an interface to the flow to start transmitting to.
    /// @param fd interface to transmit the messages on
    void start(int fd) override
    {
        if (state_ != State::STOP)
        {
            // Already started.
            return;
        }
        txFlow_->start(fd);
        rxFlow_->start(fd);
        linkEstablishment_.start();
    }

    /// Equates to a packet send. If the link is not up, the packet will be
    /// discarded.
    /// @param p payload to send
    void send_packet(Defs::Payload p) override
    {
        if (state_ == State::UP)
        {
            pingTimer_.restart();
            txFlow_->send_packet(p);
        }
    }

    /// Register for updates on link status.
    void register_link_status(LinkInterface *interface)
    {
        linkInterfaces_.push_back(interface);
    }

    /// Register a message handler.
    /// @param interface interface to dispatch the messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    void register_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK) override
    {
        dispatcher_.register_handler(interface, id, mask);
    }

    /// Unregister a message handler. Must be currently registered by a previous
    /// call to register handler.
    /// @param interface interface previously registered to dispatch the
    ///        messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    void unregister_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK) override
    {
        dispatcher_.unregister_handler(interface, id, mask);
    }

    /// Unregister all message handlers.
    /// @param interface interface previously registered to dispatch the
    ///        messages to
    void unregister_handler_all(PacketFlowInterface *interface) override
    {
        dispatcher_.unregister_handler_all(interface);
    }

    /// Sets one handler to receive all messages that no other handler has
    /// matched. May be called only once in the lifetime of a dispatcher
    /// object.
    /// @param handler is the handler pointer for the fallback handler.
    void register_fallback_handler(PacketFlowInterface *interface) override
    {
        DIE("Fallback handler not allowed.");
    }

private:
    /// State flow that establishes and monitors the link.
    class LinkEstablishment : public PacketFlowInterface, public StateFlowBase
    {
    public:
        /// Constructor.
        /// @param service service that the flow is bound to
        /// @param parent parent object
        LinkEstablishment(Service *service, Link *parent)
            : StateFlowBase(service)
            , parent_(parent)
            , timer_(this)
            , baudSupport_(Defs::BAUD_NONE)
        {
        }

        /// Start the establishing and monitoring the link.
        void start()
        {
            start_flow(STATE(ping));
        }

        /// Reset the link timeout. Should only be called when the link is up.
        void reset_link_timeout()
        {
            HASSERT(is_state(STATE(link_timeout)));
            HASSERT(parent_->is_link_up());
            timer_.restart();
        }

    private:
        /// Alias for short response timeout used during link establishment.
        static constexpr long long RESP_TIMEOUT = Defs::RESP_TIMEOUT_SHORT;

        /// Send a link establishment ping. Look for a pong response.
        /// @return nest state is pong(), wait for timeout or early trigger.
        Action ping()
        {
            parent_->rxFlow_->register_handler(this, Defs::RESP_PING);
            baudSupport_ = Defs::BAUD_NONE;
            Defs::Payload p = Defs::get_ping_payload();
            parent_->txFlow_->send_packet(p);
            return sleep_and_call(&timer_, RESP_TIMEOUT, STATE(pong));
        }

        /// Either pong received or timed out. If pong received, send a baud
        /// rate query, wait for a baud rate query response.
        /// @return next state ping() on timeout, else next state is
        ///         baud_rate_response(), wait for timeout or early trigger.
        Action pong()
        {
            parent_->rxFlow_->unregister_handler(this, Defs::RESP_PING);
            if (!timer_.is_triggered())
            {
                // Timed out, waiting for response try again.
                return call_immediately(STATE(ping));
            }

            parent_->rxFlow_->register_handler(
                this, Defs::RESP_BAUD_RATE_QUERY);

            // Query for the supported baud rates.
            Defs::Payload p = Defs::get_baud_rate_query_payload();
            parent_->txFlow_->send_packet(p);
            return sleep_and_call(
                &timer_, RESP_TIMEOUT, STATE(baud_rate_response));
        }

        /// Either baud rate response received or timed out. If baud rate
        /// response received, either accept the current baud or transition
        /// directly to link up.
        /// @return next state ping() on timeout, else next state is
        ///         link_timeout(), Wait on timeout.
        Action baud_rate_response()
        {
            parent_->rxFlow_->unregister_handler(
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
            parent_->link_up();

            return sleep_and_call(
                &timer_, Defs::RESP_TIMEOUT, STATE(link_timeout));
        }

        /// Link timed out. Transition directly to link down.
        /// @return next state ping()
        Action link_timeout()
        {
            parent_->link_down();
            return call_immediately(STATE(ping));
        }

        /// Receive for link establishment responses.
        /// @buf incoming message
        /// @prio message priority
        void send(Buffer<Message> *buf, unsigned prio) override
        {
            auto b = get_buffer_deleter(buf);
            switch(be16toh(buf->data()->command()))
            {
                case Defs::RESP_PING:
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

        Link *parent_; ///< parent object
        StateFlowTimer timer_; ///< timeout helper
        uint16_t baudSupport_; ///< mask of supported baud rates
    };

    /// Helper timer for sending pings during extended idle times.
    class PingTimer : public Timer
    {
    public:
        /// Constructor.
        /// @param e Executor to run on
        /// @param parent_ parent object
        PingTimer(ExecutorBase *e, Link *parent_)
            : Timer(e->active_timers())
        {
        }

        /// Timer expired.
        /// @return RESTART after sending a ping, NONE if triggered early.
        long long timeout() override
        {
            if (!is_triggered())
            {
                // The link went down.
                return NONE;
            }
            Defs::Payload p = Defs::get_ping_payload();
            parent_->send_packet(p);
            return RESTART;
        }

    private:
        Link *parent_; ///< parent object
    };

    /// Link state.
    enum class State
    {
        STOP, ///< link not yet started
        DOWN, ///< link down
        UP, ///< link up
    };

    /// Receive incoming message
    /// @buf incoming message
    /// @prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override
    {
        if (state_ == State::UP)
        {
            // Transfer the message. Ping responses (pong) can also come
            // through here.
            linkEstablishment_.reset_link_timeout();
            dispatcher_.send(buf, prio);
        }
        else
        {
            // Since the link is not "up" throw the message on the floor.
            buf->unref();
        }
    }

    /// Called when link transitions to "up" state.
    void link_up() override
    {
        pingTimer_.start(Defs::PING_TIMEOUT);
        state_ = State::UP;
        for (auto it = linkInterfaces_.begin();
            it != linkInterfaces_.end(); ++it)
        {
            (*it)->link_up();
        }
    }

    /// Called when link transitions to "down" state.
    void link_down() override
    {
        state_ = State::DOWN;
        pingTimer_.ensure_triggered();
        for (auto it = linkInterfaces_.begin();
            it != linkInterfaces_.end(); ++it)
        {
            (*it)->link_down();
        }
    }

    /// State flow instance that establishes and maintains the link.
    LinkEstablishment linkEstablishment_;

    TxInterface *txFlow_; ///< reference to the receive flow
    RxInterface *rxFlow_; ///< reference to the receive flow
    PingTimer pingTimer_; ///< timer that periodically pings when link is idle
    State state_; ///< link status.

    /// Handles incoming messages from the RX Flow.
    DispatchFlow<Buffer<Message>, 2> dispatcher_;

    /// List of interfaces that have registered for link updates.
    std::vector<LinkInterface*> linkInterfaces_;
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_LINK_HXX