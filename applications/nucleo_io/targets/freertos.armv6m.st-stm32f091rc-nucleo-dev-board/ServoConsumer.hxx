#ifndef _APPLICATIONS_IO_BOARD_TARGET_SERVOCONSUMER_HXX_
#define _APPLICATIONS_IO_BOARD_TARGET_SERVOCONSUMER_HXX_

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
        , pwm_gpo_(nullptr) // not initialized until apply_config
        , gpio_impl_(node, 0, 0, DummyPinWithRead())
        , consumer_(&gpio_impl_) // don't connect consumer to PWM yet
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
        const bool was_set = pwm_gpo_ && (pwm_gpo_->read() == Gpio::SET);

        if (!pwm_gpo_ || //
            cfg_event_min != gpio_impl_.event_off() ||
            cfg_event_max != gpio_impl_.event_on() ||
            cfg_srv_ticks_min != pwm_gpo_->get_off_counts() ||
            cfg_srv_ticks_max != pwm_gpo_->get_on_counts())
        {
            auto saved_node = gpio_impl_.node();

            consumer_.~BitEventConsumer();
            gpio_impl_.~GPIOBit();
            if (pwm_gpo_)
            {
                delete pwm_gpo_;
            }

            pwm_gpo_ = new PWMGPO(pwm_,
                /*on_counts=*/cfg_srv_ticks_min,
                /*off_counts=*/cfg_srv_ticks_max);
            pwm_gpo_->write(was_set ? Gpio::SET : Gpio::CLR);

            new (&gpio_impl_)
                openlcb::GPIOBit(saved_node, cfg_event_min, cfg_event_max, pwm_gpo_);
            new (&consumer_) openlcb::BitEventConsumer(&gpio_impl_);

            return REINIT_NEEDED;
        }

        return UPDATED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        cfg_.servo_min_percent().write(fd, 0);
        cfg_.servo_max_percent().write(fd, 100);
    }

private:
    // not owned; lives forever
    PWM *pwm_; // timer channel

    // all the rest are owned and must be reset on config change.
    // pwm_gpo_ heap-allocated because it's nullptr until first config.
    PWMGPO *pwm_gpo_; // has PWM* and on/off counts

    openlcb::GPIOBit gpio_impl_;                // has on/off events, Node*, and Gpio*
    openlcb::BitEventConsumer consumer_; // has GPIOBit*
    const openlcb::ServoConsumerConfig cfg_;
};

#endif // _APPLICATIONS_IO_BOARD_TARGET_SERVOCONSUMER_HXX_
