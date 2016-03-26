#include "utils/test_main.hxx"

// We have to avoid pulling in freertos stuff. We redefine the base class to
// avoid dependency on hand-written fileio stuff.
#define _FREERTOS_DRIVERS_COMMON_EEPROM_HXX_

class EEPROM
{
public:
    EEPROM(const char *name, size_t file_size)
        : fileSize(file_size)
    {
    }

    virtual void write(unsigned int index, const void *buf, size_t len) = 0;
    virtual void read(unsigned int index, void *buf, size_t len) = 0;

    size_t file_size()
    {
        return fileSize;
    }

private:
    size_t fileSize;
};

// Terrible hack to test internals of the eeprom emulation.
#define private public
#define protected public

#include "freertos_drivers/common/EEPROMEmulation.hxx"
#include "freertos_drivers/common/EEPROMEmulation.cxx"

static const char FILENAME[] = "/tmp/eeprom";

#define EELEN 32768

// We need to jump through some hoops to define a linker symbol
// "__eeprom_start" in a place that is not actually constant.
namespace foo {
extern "C" {
uint8_t __eeprom_start[EELEN];
uint8_t __eeprom_end;
}
}

#define EEBLOCKSIZE 4

const size_t EEPROMEmulation::SECTOR_SIZE = (4 * 1024);
const size_t EEPROMEmulation::BLOCK_SIZE = (EEBLOCKSIZE);
const size_t EEPROMEmulation::BYTES_PER_BLOCK = (EEBLOCKSIZE / 2);
static constexpr unsigned blocks_per_sector = EEPROMEmulation::SECTOR_SIZE / EEPROMEmulation::BLOCK_SIZE;

class MyEEPROM : public EEPROMEmulation
{
public:
    MyEEPROM(size_t file_size, bool clear = true)
        : EEPROMEmulation(FILENAME, file_size)
    {
        HASSERT(EELEN == &__eeprom_end - &__eeprom_start);
        if (clear) {
            memset(foo::__eeprom_start, 0xFF, EELEN);
        }
        mount();

        LOG(INFO, "sector count %d", sector_count());
        LOG(INFO, "slot count %d", slot_count());
        LOG(INFO, "active index %d", activeIndex);
    }

    unsigned avail() {
        return available;
    }

private:
    void flash_erase(void *address) override {
        ASSERT_LE((void*)&foo::__eeprom_start[0], address);
        ASSERT_GT((void*)&foo::__eeprom_start[EELEN], address);
        ASSERT_EQ(0, (((uint8_t*)address) - foo::__eeprom_start)  % EEPROMEmulation::SECTOR_SIZE);
        memset(address, 0xff, EEPROMEmulation::SECTOR_SIZE);
    }

    void flash_program(uint32_t *data, void *address, uint32_t count) override {
        ASSERT_LE((void*)&__eeprom_start, address);
        ASSERT_GE((void*)(&foo::__eeprom_start[EELEN]), address);
        memcpy(address, data, count);
    }

    int address_to_sector(const void *address) override {
        uint8_t* a = (uint8_t*)address;
        return (a - &foo::__eeprom_start[0]) / SECTOR_SIZE;
    }

    uint32_t *sector_to_address(const int sector) override {
        return (uint32_t*) &(foo::__eeprom_start[sector * SECTOR_SIZE]);
    }
};

TEST(EepromStaticTest, assertions) {
    volatile size_t p1 = (volatile size_t)&__eeprom_start;
    volatile size_t p2 = (volatile size_t)&foo::__eeprom_start[0];

    volatile size_t e1 = (volatile size_t)&__eeprom_end;

    ASSERT_EQ(p1, p2);
    ASSERT_EQ(p1 + EELEN, e1);
    ASSERT_EQ(0, p1 % 4); // alignment
}

class EepromTest : public ::testing::Test {
protected:
    void create(bool clear = true) {
        e.reset(new MyEEPROM(eeprom_size, clear));
    }

    void write_to(unsigned ofs, const string& payload) {
        ee()->write(ofs, payload.data(), payload.size());
    }

    EEPROM* ee() {
        return static_cast<EEPROM*>(e.operator->());
    }

    /** Returns the data payload in a given block. */
    string block_data(unsigned block_number) {
        uint32_t* address = (uint32_t*)&foo::__eeprom_start[block_number * EEBLOCKSIZE];
        uint8_t data[EEBLOCKSIZE / 2];
        for (int i = 0; i < EEBLOCKSIZE / 4; ++i) {
            data[(i * 2) + 0] = (address[i] >> 0) & 0xFF;
            data[(i * 2) + 1] = (address[i] >> 8) & 0xFF;
        }
        return string((char*)data, EEBLOCKSIZE / 2);
    }

    void overflow_block() {
        unsigned avail = e->avail();
        for (int i = 0; i < 27000; ++i) {
            char d[1] = {static_cast<char>(i & 0xff)};
            write_to(27, string(d, 1));
            if (e->avail() > avail) return;
            avail = e->avail();
        }
    }

    /** Returns the data payload in a given block. */
    uint32_t block_address(unsigned block_number) {
        uint32_t* address = (uint32_t*)&foo::__eeprom_start[block_number * EEBLOCKSIZE];
        return ((*address) >> 16) * (EEBLOCKSIZE / 2);
    }

#define EXPECT_AT(ofs, PAYLOAD) { string p(PAYLOAD); string ret(p.size(), 0); ee()->read(ofs, &ret[0], p.size()); EXPECT_EQ(p, ret); }

#define EXPECT_SLOT(block_number, address, payload) { EXPECT_EQ(address, block_address(block_number)); EXPECT_EQ(string(payload), block_data(block_number)); }

    static constexpr unsigned eeprom_size = 1000;
    std::unique_ptr<MyEEPROM> e;
};


TEST_F(EepromTest, create) {
    create();
    EXPECT_EQ(0, e->activeIndex);
    EXPECT_EQ(8, e->sector_count());
    EXPECT_EQ(0, e->address_to_sector(&__eeprom_start));
    EXPECT_EQ((uint32_t*)&__eeprom_start, e->sector(0));
    EXPECT_EQ((uint32_t*)&foo::__eeprom_start[4*1024], e->sector(1));
}

TEST_F(EepromTest, readwrite) {
    create();

    write_to(13, "abcd");
    EXPECT_SLOT(3, 12, "\xFF""a");
    EXPECT_SLOT(4, 14, "bc");
    EXPECT_SLOT(5, 16, "d\xFF");
    EXPECT_SLOT(6, 2*0xFFFF, "\xFF\xFF");

    EXPECT_AT(13, "abcd");
    EXPECT_AT(14, "bc");
    EXPECT_AT(15, "cd");

    write_to(12, "up");
    EXPECT_AT(12, "upb");
    EXPECT_AT(12, "upbcd");
    write_to(12, "kq");
    EXPECT_AT(12, "kqbcd");
    EXPECT_AT(12, "kqbcd\xFF");

    // Reboot MCU
    create(false);
    EXPECT_AT(12, "kqbcd\xFF");
}

TEST_F(EepromTest, smalloverflow) {
    create();
    write_to(13, "abcd");
    EXPECT_AT(13, "abcd");
    EXPECT_SLOT(3, 12, "\xFF""a");
    EXPECT_SLOT(4, 14, "bc");
    EXPECT_SLOT(5, 16, "d\xFF");
    overflow_block();
    EXPECT_EQ(e->sector(1), e->active());
    EXPECT_SLOT(3, 12, "\xFF""a");
    EXPECT_SLOT(4, 14, "bc");
    EXPECT_SLOT(5, 16, "d\xFF");
    EXPECT_SLOT(blocks_per_sector + 3, 12, "\xFF""a");
    EXPECT_SLOT(blocks_per_sector + 4, 14, "bc");
    EXPECT_SLOT(blocks_per_sector + 5, 16, "d\xFF");

    EXPECT_AT(13, "abcd");
    overflow_block();
    EXPECT_AT(13, "abcd");
    overflow_block();
    EXPECT_AT(13, "abcd");
}

TEST_F(EepromTest, many_overflow) {
    create();
    write_to(13, "abcd");
    EXPECT_AT(13, "abcd");
    // A lot of writes will surely cause the data to be overflowed to a new
    // sector
    for (int i = 0; i < 20; ++i) {
        overflow_block();
    }
    EXPECT_AT(13, "abcd");
}
