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
        : txFlow_(tx_flow)
        , rxFlow_(rx_flow)
        , restartTimer_(service->executor(), this)
        , txTimer_(service->executor(), this)
        , rxTimer_(service->executor(), this)
        , state_(State::STOP)
        , dispatcher_(service)
    {
        rxFlow_->register_fallback_handler(this);
    }

    /// Check on the link status
    /// @return true if link is up, else link is down.
    bool is_link_up()
    {
        return state_ == STATE::UP;
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
        reset_link();
    }

    /// Equates to a packet send. If the link is not up, the packet will be
    /// discarded.
    /// @param p payload to send
    void send_packet(Defs::Payload p) override
    {
        if (state_ == State::UP)
        {
            txTimer_.restart();
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
    /// Helper timer for restarting the link.
    class RestartTimer : public Timer
    {
    public:
        /// Constructor.
        /// @param e Executor to run on
        /// @param parent_ parent object
        RestartTimer(ExecutorBase *e, Link *parent_)
            : Timer(e->active_timers())
        {
        }

        /// Timer expired.
        /// @returns the new timer period, or one of the above special values.
        long long timeout() override
        {
            parent_->reset_link();
            return NONE;
        }

    private:
        Link *parent_; ///< parent object
    };

    /// Helper timer for periodic transmit.
    class TxTimer : public Timer
    {
    public:
        /// Constructor.
        /// @param e Executor to run on
        /// @param parent_ parent object
        TxTimer(ExecutorBase *e, Link *parent_)
            : Timer(e->active_timers())
        {
        }

        /// Timer expired.
        /// @returns the new timer period, or one of the above special values.
        long long timeout() override
        {
            if (!is_triggered())
            {
                parent_->send_ping();
                return RESTART;
            }
            return NONE
        }

    private:
        Link *parent_; ///< parent object
    };

    /// Helper timer for link receive timeout.
    class RxTimer : public Timer
    {
    public:
        /// Constructor.
        /// @param e Executor to run on
        /// @param parent_ parent object
        TxTimer(ExecutorBase *e, Link *parent_)
            : Timer(e->active_timers())
        {
        }

        /// Timer expired.
        /// @returns the new timer period, or one of the above special values.
        long long timeout() override
        {
            if (!is_triggered())
            {
                parent_->link_down();
            }
            return NONE;
        }

    private:
        Link *parent_; ///< parent object
    };

    enum class State
    {
        STOP, ///< link not yet started
        BAUD, ///< link baud still being negotiated
        BAUD_TEST, ///< link baud chosen, testing for integrity
        UP, ///< ink up
    };

    /// Receive for output commands.
    /// @buf incoming message
    /// @prio message priority
    void send(Buffer<Message> *buf, unsigned prio) override
    {
        // Certain commands/responses are intercepted for link management.
        switch(be16toh(buf->data()->command()))
        {
            case Defs::RESP_PING:
                if (state_ == State::BAUD)
                {
                    // Still in the baud rate negotiation stage.
                    Defs::Payload p = Defs::get_baud_rate_query_payload();
                    txFlow_->send_packet(p);
                }
                else if (state_ == State::UP)
                {
                    // Link is running normally, reset the RX timeout.
                    rxTimer_.restart();
                }
                else
                {
                    // Should never get here.
                    LOG(WARNING, "Unexpected ping response.");
                }
                break;
            case Defs::RESP_BAUD_RATE_QUERY:
            {
                Defs::BaudRateQueryResponse *brqr =
                    (Defs::BaudRateQueryResponse*)buf->data()->payload.data();
                /// @todo For now only supporting 250Kbps.
                HASSERT((be16toh(brqr->rates_) & Defs::BAUD_250K_MASK) != 0);
                /// @todo Need to decide if we should change the baud rate here
                ///       and restart the ping/pong process.
                link_up();
                break;
            }
            case Defs::RESP_MEM_W:
                /// @todo There is a special case where we want to intercept
                ///       these packets during a baud rate integrity test.
            default:
                // Not a link management message.
                if (state_ == State::UP)
                {
                    // Reset the RX timeout.
                    rxTimer_.restart();
                    // Transfer the message.
                    dispatcher_.send(buf, prio);
                    return;
                }
                // Throw the message on the floor since the link is down.
                break;
        }
        buf->unref();
    }

    /// Send a ping packet.
    void send_ping()
    {
        Defs::Payload p = Defs::get_ping_payload();
        txFlow_->send_packet(p);
    }

    /// Restart link.
    void restart_link()
    {
        state_ = State::BAUD;
        restartTimer_.start(SEC_TO_NSEC(3));
    }

    /// Reset the link negotiation.
    void reset_link()
    {
        state_ = State::BAUD;
        send_ping();
    }

    /// Called when link transitions to "up" state.
    void link_up() override
    {
        txTimer_.start(SEC_TO_NSEC(2));
        rxTimer_.start(SEC_TO_NSEC(3));
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
        state_ = State::BAUD;
        txTimer_.ensure_triggered();
        rxTimer_.ensure_triggered();
        for (auto it = linkInterfaces_.begin();
            it != linkInterfaces_.end(); ++it)
        {
            (*it)->link_down();
        }
        // Restart the link.
        restartTimer_.start();
    }

    TxInterface *txFlow_; ///< reference to the receive flow
    RxInterface *rxFlow_; ///< reference to the receive flow
    RestartTimer restartTimer_; ///< timer that manages restarting the link
    TxTimer txTimer_; ///< timer that ensures periodic message transmission
    RxTimer rxTimer_; ///< timer that ensures periodic message reception
    State state_; ///< link status.

    /// Handles incoming messages from the RX Flow.
    DispatchFlow<Buffer<Message>, 2> dispatcher_;

    /// List of interfaces that have registered for link updates.
    std::vector<LinkInterface*> linkInterfaces_;
};


} // namespace traction_modem

#endif // _TRACTION_MODEM_LINK_HXX