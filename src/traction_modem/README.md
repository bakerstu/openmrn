# Traction Modem
Traction modem is a protocol that can be used to exchange typical OpenLCB train information and control over a simple serial interface, typically a UART. Communication is largely peer-to-peer, but link management is peformed by the modem side of the interface.

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

The following state machine for link management is implemented in Link.hxx on the modem side:

```mermaid
stateDiagram
    direction TB

    Ping: Ping (Link Down)
    BaudRateQuery: Baud Rate Query
    PingBaudRateChange: Baud Rate Request
    Write: Write
    LinkUp: Link Up
    LinkDown: Link Down

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

    LinkTimeoutState           --> LinkDown: if (3 second timeout)
    LinkTimeoutState           --> LinkUp: if (no timeout)

    LinkDown                   --> Ping
```
## Timing <a id="timing"></a>
In general, the following link timings should be adhered to:

| Parameter                     | Value | Units |
|:------------------------------|:-----:|:-----:|
| Modem RX Timeout              | 3     | s     |
| Modem Idle Ping Timeout       | 2     | s     |
| Response Timeout              | 750   | ms    |
| Link Establishment RX Timeout | 1     | s     |

### Modem RX Timeout
The modem must receive at least one message every 3 seconds. Periodic pings help facilitate this. A ping from the modem should solicit a pong response from the train in order to reset the modem RX timeout. Any received message, without framing error, shall reset the modem RX timeout.

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
    RxInterface                      <|-- Link
    LinkInterface                    <|-- Link
    PacketFlowInterface~MessageType~ <|-- Link
    TxInterface                      <|-- Link
    TxInterface                      <|-- TxFlow

    RxInterface         : +register_handler(PacketFlowInterface *, ...) void override
    TxInterface         : +send_packet(Payload) virtual void = 0
    LinkInterface       : +link_up() virtual void = 0
    LinkInterface       : +link_down() virtual void = 0
    PacketFlowInterface : +send(MessageType*, unsigned) virtual void = 0
    RxFlow              : -DispatchFlow~Buffer~Message~~ dispatcher_
    TxFlow              : +send_packet(Payload) void override
    Link                : -TxFlow *txFlow_
    Link                : -RxFlow *rxFlow_
    Link                : -DispatchFlow~Buffer~Message~~ dispatcher_
    Link                : vector~LinkInterface*~ linkInterfaces_
    Link                : +Link(Service*, TxInterface*, RxInterface*, bool)
    Link                : +send_packet(payload) void override
    Link                : -send(MessageType*, unsigned) void override
    Link                : -link_down() void override
    Link                : -link_up() void override
```

### Specialized Message Handling
The MemorySpace object(s) are provided as an example.

```mermaid
classDiagram
    PacketFlowInterface <|-- MemorySpace
    LinkInterface       <|-- MemorySapce

    LinkInterface       : +link_up() virtual void = 0
    LinkInterface       : +link_down() virtual void = 0
    PacketFlowInterface : +send(MessageType*, unsigned) virtual void = 0
    MemorySpace         : Link *link_
    MemorySpace         : +MemorySpace(Service*, Link*)
    MemorySpace         : +write() size_t
    MemorySpace         : +read() size_t
    MemorySpace         : -send(MessageType*, unsigned) void override
    MemorySpace         : -link_up() void override
    MemorySpace         : -link_down() void override
```

The following provides and example call sequence for writing and reading a memory space interspersed with background pings.

```mermaid
sequenceDiagram
    actor User
    participant MemorySpace
    participant LinkObject as Link
    participant TxFlow
    participant RxFlow

    LinkObject  -->> TxFlow      : ping
    RxFlow      -->> LinkObject  : pong
    LinkObject  -->> TxFlow      : ping
    RxFlow      -->> LinkObject  : pong

    User         ->> MemorySpace : write()
    activate MemorySpace
    MemorySpace  ->> LinkObject  : link_->send_packet()
    LinkObject   ->> TxFlow      : txFlow_->send_packet()
    RxFlow       ->> LinkObject  : dispatcher_.send()
    LinkObject   ->> MemorySpace : dispatcher_.send()
    MemorySpace  ->> User : 
    deactivate MemorySpace
    
    LinkObject  -->> TxFlow      : ping
    RxFlow      -->> LinkObject  : pong
    LinkObject  -->> TxFlow      : ping
    RxFlow      -->> LinkObject  : pong

    User         ->> MemorySpace : read()
    activate MemorySpace
    MemorySpace  ->> LinkObject  : link_->send_packet()
    LinkObject   ->> TxFlow      : txFlow_->send_packet()
    RxFlow       ->> LinkObject  : dispatcher_.send()
    LinkObject   ->> MemorySpace : dispatcher_.send()
    MemorySpace  ->> User        : 
    deactivate MemorySpace

    LinkObject  -->> TxFlow      : ping
    RxFlow      -->> LinkObject  : pong
    LinkObject  -->> TxFlow      : ping
    RxFlow      -->> LinkObject  : pong
```
