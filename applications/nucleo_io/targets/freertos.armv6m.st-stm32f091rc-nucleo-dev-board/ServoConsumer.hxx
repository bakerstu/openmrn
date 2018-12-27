#ifndef _APPLICATIONS_IO_BOARD_TARGET_SERVOCONSUMER_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_SERVOCONSUMER_HXX_

#include <memory>
#include "config.hxx"
#include "os/MmapGpio.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"

const unsigned servo_ticks_0 = configCPU_CLOCK_HZ * 1 / 1000;   // 48k
const unsigned servo_ticks_180 = configCPU_CLOCK_HZ * 2 / 1000; // 96k

// Basically a specialized ConfiguredConsumer.
// Can't subclass ConfiguredConsumer here because ServoConsumerConfig
// isn't a subclass of ConsumerConfig,
class ServoConsumer : public DefaultConfigUpdateListener
{
public:
    ServoConsumer(
        openlcb::Node *node, const openlcb::ServoConsumerConfig &cfg, PWM *pwm)
        : DefaultConfigUpdateListener()
        , pwm_(pwm) // save for apply_config, where we actually use it.
        , pwmGpo_(nullptr) // not initialized until apply_config
        , gpioImpl_(node, 0, 0, DummyPinWithRead())
        , consumer_(&gpioImpl_) // don't connect consumer to PWM yet
        , cfg_(cfg)
    {
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        const openlcb::EventId cfg_event_min = cfg_.event_rotate_min().read(fd);
        const openlcb::EventId cfg_event_max = cfg_.event_rotate_max().read(fd);
        const uint8_t cfg_servo_min_pct = cfg_.servo_min_percent().read(fd);
        const uint8_t cfg_servo_max_pct = cfg_.servo_max_percent().read(fd);

        const unsigned servo_range_ticks = servo_ticks_180 - servo_ticks_0;

        const unsigned cfg_srv_ticks_min =
            servo_ticks_0 + (servo_range_ticks * (cfg_servo_min_pct / 100.0));
        const unsigned cfg_srv_ticks_max =
            servo_ticks_0 + (servo_range_ticks * (cfg_servo_max_pct / 100.0));

        // Defaults to CLR at startup.
        const bool was_set = pwmGpo_ && (pwmGpo_->read() == Gpio::SET);

        if (!pwmGpo_ || //
            cfg_event_min != gpioImpl_.event_off() ||
            cfg_event_max != gpioImpl_.event_on() ||
            cfg_srv_ticks_min != pwmGpo_->get_off_counts() ||
            cfg_srv_ticks_max != pwmGpo_->get_on_counts())
        {
            auto saved_node = gpioImpl_.node();

            consumer_.~BitEventConsumer();
            gpioImpl_.~GPIOBit();

            pwmGpo_.reset(new PWMGPO(pwm_,
                /*on_counts=*/cfg_srv_ticks_min,
                /*off_counts=*/cfg_srv_ticks_max));
            pwmGpo_->write(was_set ? Gpio::SET : Gpio::CLR);

            new (&gpioImpl_)
                openlcb::GPIOBit(saved_node, cfg_event_min, cfg_event_max, pwmGpo_.get());
            new (&consumer_) openlcb::BitEventConsumer(&gpioImpl_);

            return REINIT_NEEDED;
        }

        return UPDATED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.servo_min_percent);
        CDI_FACTORY_RESET(cfg_.servo_max_percent);
    }

private:
    // not owned; lives forever
    PWM *pwm_; // timer channel

    // all the rest are owned and must be reset on config change.
    // pwmGpo_ heap-allocated because it's nullptr until first config.
    std::unique_ptr<PWMGPO> pwmGpo_; // has PWM* and on/off counts

    openlcb::GPIOBit gpioImpl_;                // has on/off events, Node*, and Gpio*
    openlcb::BitEventConsumer consumer_; // has GPIOBit*
    const openlcb::ServoConsumerConfig cfg_;
};

#endif // _APPLICATIONS_IO_BOARD_TARGET_SERVOCONSUMER_HXX_
