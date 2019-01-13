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

class AsyncClientStream : ::Stream {
public:
    AsyncClientStream(AsyncClient *client) :_client(client) {
        _client->setNoDelay(true);
        _client->onData(std::bind(&AsyncClientStream::dataReader, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), NULL);
        _client->onDisconnect(std::bind(&AsyncClientStream::disconnectHandler, this, std::placeholders::_1, std::placeholders::_2), NULL);
        _client->onError(std::bind(&AsyncClientStream::errorHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), NULL);
        _client->onTimeout(std::bind(&AsyncClientStream::timeoutHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), NULL);
        _buffer.reserve(256);
    }
    virtual size_t write(uint8_t data) {
        _client->write(data);
    }
    virtual size_t write(const uint8_t *buffer, size_t size) {
        _client->write(buffer, size);
    }
    virtual int available() {
        return _buffer.size();
    }
    virtual int read() {
        return _buffer.pop_front();
    }
    virtual int peek() {
        return _buffer.front();
    }
    virtual void flush() {
        _buffer.clear();
    }
private:
    void dataReader(void *arg, AsyncClient *client, void *data, size_t len) {
        std::copy(data, data + len, back_inserter(_buffer));
    }
    void disconnectHandler(void *arg, AsyncClient *client) {
        Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
    }
    void errorHandler(void *arg, AsyncClient *client, int8_t error) {
        Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
    }
    void timeoutHandler(void *arg, AsyncClient *client, uint32_t time) {
        Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
    }
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

    server.onClient([]((void *arg, AsyncClient *client)) {
        openmrn.add_gridconnect_port(new AsyncClientStream(client));
    });

    server.begin();

    openmrn.init();
}

void loop() {
    openmrn.loop();
    WiFiClient client = server.available();   // listen for incoming clients
    if(client) {

    }
}