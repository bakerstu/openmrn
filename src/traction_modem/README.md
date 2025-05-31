# Traction Modem
Traction modem is a protocol that can be used to exchange typical OpenLCB train information and control over a simple serial interface, typically a UART. Communication is largely peer-to-peer, but link management is performed by the modem side of the interface.

1. [Link Management](#link_management)
2. [Timing](#timing)
3. [Architecture](#architecture)

## Link Management <a id="link_management"></a>
The following command types are used for Link management:

| Command                  | Initiator |
|:-------------------------|:---------:|
| Ping                     | Modem     |
| Pong                     | Train     |
| Baud Rate Query          | Modem     |
| Baud Rate Query Response | Train     |
| Baud Rate Request        | Modem     |
| Write                    | Modem     |
| Write Response           | Train     |
| Nop                      | Train     |

To establish a link, the modem performs the following minimum procedure:

```mermaid
sequenceDiagram
    Modem ->> Train: Ping
    Train ->> Modem: Pong
```

It is assumed, by default, that the baud rate will be 250Kbps. The modem may optionally query for additional baud rates that the train may support.

```mermaid
sequenceDiagram
    Modem ->> Train: Baud Rate Query
    Train ->> Modem: Baud Rate Response (250kbps, 1Mbps, 3Mbps, etc.)
```

At this point, a new baud rate may optionally be chosen. The sequence includes 5 large dummy writes to the Null address space. These must be responded to without error before the link is established.


```mermaid
sequenceDiagram
    Modem ->> Train: Baud Rate Request (3Mbps)
    Train ->> Modem: Baud Rate Query Response
    Note over Modem, Train: Wait 200 milliseconds
    Modem ->> Train: Ping
    Train ->> Modem: Pong
    loop 5 times
        Modem ->> Train: Write (Null Address Space, 256 dummy bytes)
        Train ->> Modem: Write Response (Success, no error, size = 256)
    end
```

The following state machine for link management is implemented in Link.hxx by the LinkManager object on the modem side:

```mermaid
stateDiagram
    direction TB

    Ping: Ping
    BaudRateQuery: Baud Rate Query
    PingBaudRateChange: Baud Rate Request
    Write: Write
    LinkUp: Link Up<br>(start 2 second ping)
    LinkDown: Link Down<br>(stop 2 second ping)

    state PongState <<choice>>
    state DefaultBaudState <<choice>>
    state BaudRateQueryResponseState <<choice>>
    state PongBaudRateChangeState <<choice>>
    state WriteResponseState <<choice>>
    state WriteResponseRepeatState <<choice>>
    state LinkTimeoutState <<choice>>

    [*]                        --> Ping

    Ping                       --> PongState

    PongState                  --> Ping: if (1 second timeout)
    PongState                  --> DefaultBaudState: if (pong received)

    DefaultBaudState           --> LinkUp: if (use default baud)
    DefaultBaudState           --> BaudRateQuery: if (not default baud)

    BaudRateQuery              --> BaudRateQueryResponseState

    BaudRateQueryResponseState --> Ping: if (1 second timeout)
    BaudRateQueryResponseState --> LinkUp: if (use current baud)
    BaudRateQueryResponseState --> PingBaudRateChange: if (change baud)

    PingBaudRateChange         --> PongBaudRateChangeState

    PongBaudRateChangeState    --> Ping: if (1 second timeout)
    PongBaudRateChangeState    --> Write: if (pong received)

    Write                      --> WriteResponseState

    WriteResponseState         --> Ping: if (1 second timeout)
    WriteResponseState         --> Ping: if (error returned)
    WriteResponseState         --> WriteResponseRepeatState: if (success)

    WriteResponseRepeatState   --> Write: if (count < 5)
    WriteResponseRepeatState   --> LinkUp: if (count == 5)

    LinkUp                     --> LinkTimeoutState

    LinkTimeoutState           --> LinkDown: if (3 second pong timeout)
    LinkTimeoutState           --> LinkUp: if (no pong timeout)

    LinkDown                   --> Ping
```
## Timing <a id="timing"></a>
In general, the following link timings should be adhered to:

| Parameter                     | Value | Units |
|:------------------------------|:-----:|:-----:|
| Modem RX Timeout              | 3     | s     |
| Modem Ping Timeout            | 2     | s     |
| Response Timeout              | 750   | ms    |
| Link Establishment RX Timeout | 1     | s     |

### Modem RX Timeout
The modem must receive at least one ping response (pong) every 3 seconds. Periodic pings facilitate this. A ping from the modem should solicit a pong response from the train in order to reset the modem RX timeout.

### Modem Idle Ping Timeout
While the link is in the "up" state, the modem shall send a ping every 2 seconds.

### Response Timeout
For any message that requires a response, the peer shall begin responding within 750 milliseconds.

### Link Establishment Timeout
In order to help facilitate faster link establishment, the RX timeout is shortened in the link establishment phase before the link is "up".

## Architecture <a id="architecture"></a>
These architecture diagrams are highly simplified versions of the underlying implementation.

### Generic Message Handling
The following diagram shows the high level class hierarchy for generic message transmission and reception. The Link object handles the [Link Management](#link_management) through an internal state machine.

```mermaid
classDiagram
    RxInterface                      <|-- RxFlow
    TxInterface                      <|-- TxFlow
    LinkStatusInterface              <|-- LinkManager
    PacketFlowInterface~MessageType~ <|-- LinkManager
    StateFlowBase                    <|-- LinkManager

    RxInterface         : +register_handler(PacketFlowInterface *, ...) void override
    TxInterface         : +send_packet(Payload) virtual void = 0
    Link                : -TxInterface *txFlow_
    Link                : -RxInterface *rxFlow_
    Link                : -State state_
    Link                : -vector~LinkStatusInterface*~ linkIfaces_
    Link                : +Link(TxInterface*, RxInterface*)
    Link                : +is_link_down() bool
    Link                : +start(int fd) void
    Link                : +register_link_status(LinkStatusInterface*) void
    Link                : +unregister_link_status(LinkStatusInterface*) void
    Link                : +get_tx_iface() TxInterface
    Link                : +get_rx_iface() RxInterface
    Link                : -link_down() void
    Link                : -link_up() void
    PacketFlowInterface : +send(MessageType*, unsigned) virtual void = 0
    RxFlow              : -DispatchFlow~Buffer~Message~~ dispatcher_
    TxFlow              : +send_packet(Payload) void override
    LinkStatusInterface : #friend class Link
    LinkStatusInterface : #link_start() virtual void
    LinkStatusInterface : #link_up() virtual void
    LinkStatusInterface : #link_down() virtual void
    LinkManager         : -Link *link_
    LinkManager         : -PingTimer *pingTimer_
    LinkManager         : +LinkManager(Service *, Link*, bool use_default_baud)
    LinkManager         : -link_start() void override
    LinkManager         : -send(MessageType*, unsigned) virtual void = 0
```

### Specialized Message Handling
The MemorySpace object(s) are provided as an example.

```mermaid
classDiagram
    PacketFlowInterface <|-- MemorySpace
    LinkStatusInterface <|-- MemorySpace
    MemorySpace         <|-- CvSpace

    LinkStatusInterface : #friend class Link
    LinkStatusInterface : #link_start() virtual void
    LinkStatusInterface : #link_up() virtual void
    LinkStatusInterface : #link_down() virtual void
    PacketFlowInterface : #send(MessageType*, unsigned) virtual void = 0
    MemorySpace         : #Link *link_
    MemorySpace         : +write() size_t
    MemorySpace         : +read() size_t
    MemorySpace         : #MemorySpace(Service*, Link*)
    MemorySpace         : -send(MessageType*, unsigned) void override
    MemorySpace         : -link_up() void override
```

The following provides and example call sequence for writing a memory space. Link starts down, then later comes up and is interspersed with background pings.

```mermaid
sequenceDiagram
    actor User
    participant MemorySpace
    participant LinkObject as Link
    participant LinkManager
    participant RxFlow
    participant TxFlow

    User         ->> MemorySpace : write()
    activate MemorySpace
    MemorySpace  ->> LinkObject  : is_link_up()
    activate LinkObject
    LinkObject   ->> MemorySpace : false
    deactivate LinkObject
    MemorySpace  ->> User        : 0, ERROR_AGAIN
    deactivate MemorySpace

    User         ->> LinkObject  : start()
    LinkObject   ->> LinkManager : link_start()
    LinkManager -->> TxFlow      : ping
    RxFlow      -->> LinkManager : pong
    Note over LinkManager, RxFlow: Link Up<br>(start background ping/pong)
    LinkManager  ->> LinkObject  : link_up()
    LinkObject   ->> MemorySpace : link_up()

    MemorySpace  ->> TxFlow      : link_.get_tx_Iface()->send_packet()
    activate TxFlow
    activate RxFlow
    RxFlow       ->> MemorySpace : dispatcher_.send()
    deactivate TxFlow
    deactivate RxFlow
    MemorySpace  ->> User        : notify()
    
    LinkManager -->> TxFlow      : ping
    RxFlow      -->> LinkManager : pong

    User         ->> MemorySpace : write()
    activate MemorySpace
    MemorySpace  ->> LinkObject  : is_link_up()
    activate LinkObject
    LinkObject   ->> MemorySpace : true
    deactivate LinkObject
    MemorySpace  ->> User        : bytes written, ERROR_SUCCESS
    deactivate MemorySpace

    LinkManager -->> TxFlow      : ping
    RxFlow      -->> LinkManager : pong
```
