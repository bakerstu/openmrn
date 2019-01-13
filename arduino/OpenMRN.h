#ifndef _ARDUINO_OPENMRN_H_
#define _ARDUINO_OPENMRN_H_

#include "openlcb/SimpleStack.hxx"
#include "utils/Uninitialized.hxx"
#include "utils/GridConnectHub.hxx"
#include "freertos_drivers/common/Can.hxx"



/// Bridge class that connects an Arduino API style stream to the OpenMRN
/// core stack.
class StreamBridge : public Executable {
public:
    StreamBridge(::Stream *port, CanHubFlow* can_hub)
        : service_(can_hub->service()), port_(port) {
        GCAdapterBase::CreateGridConnectAdapter(&txtHub_, can_hub, false);
        txtHub_.register_port(&writePort_);
    }
    
    /// Called by the loop.
    void run() override {
        loop_for_write();
        loop_for_read();
    }

private:
    /// Access to the stack's executor.
    Service* service_;
    
    void loop_for_write() {
        if (!writeBuffer_) return;
        size_t len = port_->availableForWrite();
        if (!len) return;
        size_t to_write = writeBuffer_->data()->size() - writeOfs_;
        if (len > to_write) len = to_write;
        port_->write(writeBuffer_->data()->data() + writeOfs_, len);
        writeOfs_ += len;
        if (writeOfs_ >= writeBuffer_->data()->size()) {
            writeBuffer_ = nullptr;
            // wakes up state flow to release buffer and take next from the
            // queue.
            writePort_.notify();
        }
    }

    void loop_for_read() {
        if (port_->available() <= 0) return;
        auto *b = txtHub_.alloc();
        port_->read(b->data());
        b->data()->skipMember_ = &writePort_;
        txtHub_.send(b);
    }

    friend class WritePort;
    class WritePort: public HubPort {
    public:
        WritePort(StreamBridge* parent, Service* service)
            : HubPort(service), parent_(parent) {
        }

        Action entry() override {
            parent_->writeBuffer_ = message();
            parent_->writeOfs_ = 0;
            return wait_and_call(STATE(write_done));
        }

        Action write_done() {
            return release_and_exit();
        }

    private:
        StreamBridge* parent_;
    } writePort_{this, service_};

    
    /// Arduino device instance.
    ::Stream* port_;
    /// Buffer we are writing the output from right now.
    Buffer<HubData>* writeBuffer_{nullptr};
    /// Offset in the output string of the next byte to write.
    size_t writeOfs_;
    /// Hub for the textual data.
    HubFlow txtHub_{service_};
};

class OpenMRN : private Executable {
public:
    OpenMRN() {}
    /// Use this constructor if stack() needs to be accessed during the time of
    /// the static construction. Then use init() with no arguments.
    OpenMRN(openlcb::NodeID node_id) {
        init(node_id);
    }
    
    /// Call this function once if the empty constructor was used.
    void init(openlcb::NodeID node_id) {
        stack_.emplace(node_id);
        //stack_->start_stack(false);
    }

    /// Call this function once if the constructor with Node ID was used.
    void init() {
        //stack_->start_stack(false);
    }
    
    /// @return pointer to the stack. Do not call before init().
    openlcb::SimpleCanStack* stack() {
        return stack_.operator->();
    }

    /// Call this function from the loop() function of the Arduino sketch.
    void loop() {
        for (auto* e: loopMembers_) {
            e->run();
        }
    }

    void add_gridconnect_port(::Stream* port) {
      loopMembers_.push_back(
          new StreamBridge<Stream>(port, stack()->can_hub()));
    }
/*
    void add_can_port(Can* port) {
        loopMembers_.push_back(new CanBridge(port, stack()->can_hub()));
    }
*/
private:
    void run() override {
        stack_->executor()->loop_once();
    }
    
    /// Storage space for the OpenLCB stack. Will be constructed in init().
    uninitialized<openlcb::SimpleCanStack> stack_;
    /// List of objects we need to call in each loop iteration.
    vector<Executable*> loopMembers_{{this}};
};


#endif // _ARDUINO_OPENMRN_H_
