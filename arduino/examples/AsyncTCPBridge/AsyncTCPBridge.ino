#include <Arduino.h>
#include <OpenMRN.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <vector>

const char* ssid     = "yourssid";
const char* password = "yourpasswd";

static constexpr uint64_t NODE_ID = UINT64_C(0x050101011423);
OpenMRN openmrn(NODE_ID);

AsyncServer server(12021);

class AsyncClientStream : public ::Stream {
public:
    AsyncClientStream(AsyncClient *client) :_client(client) {
        _client->setNoDelay(true);
        _client->onData([](void *arg, AsyncClient *client, void *data, size_t len) {
          static_cast<AsyncClientStream *>(arg)->feed(data, len);
        }, this);
        _buffer.reserve(256);
    }
    void feed(void *data, size_t len) {
      std::copy(static_cast<uint8_t *>(data), static_cast<uint8_t *>(data) + len, back_inserter(_buffer));
    }
    size_t availableForWrite() {
      return _client->space();
    }
    virtual size_t write(uint8_t data) {
        return write(&data, 1);
    }
    size_t write(const char *buffer, size_t size) {
      return _client->add(buffer, size);
    }
    virtual size_t write(const uint8_t *buffer, size_t size) {
        return _client->add((const char *)buffer, size);
    }
    virtual int available() {
        return _buffer.size();
    }
    virtual int read() {
        int data = *_buffer.begin();
        _buffer.erase(_buffer.begin());
        return data;
    }
    size_t read(HubData *buffer) {
      size_t bytesRead = 0;
      while(available()) {
        buffer->push_back((char)read());
        bytesRead++;
      }
      return bytesRead;
    }
    virtual int peek() {
        return *_buffer.begin();
    }
    virtual void flush() {
        _buffer.clear();
    }
private:
    AsyncClient *_client;
    std::vector<uint8_t> _buffer;
};

void setup() {
    Serial.begin(115200L);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.onClient([](void *arg, AsyncClient *client) {
        openmrn.add_gridconnect_port(new AsyncClientStream(client));
    }, NULL);

    server.begin();

    openmrn.init();
}

void loop() {
    openmrn.loop();
}