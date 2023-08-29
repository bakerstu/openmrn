/** @copyright
 * Copyright (c) 2018 Balazs Racz
 * All rights reserved.
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
 * @file Stm32PWM.hxx
 * This file implements a PWM driver for the STM32 timers.
 *
 * @author Balazs Racz
 * @date 1 May 2018
 */

#ifndef _FREEERTOS_DRIVERS_ST_STM32PWM_HXX_
#define _FREEERTOS_DRIVERS_ST_STM32PWM_HXX_

#include "PWM.hxx"
#include "utils/Uninitialized.hxx"

#include "stm32f_hal_conf.hxx"

/// Set of 4 PWM channels that belong to a single timer resource in the STM32
/// microcontrollers. Note that the different channels are tied to be using the
/// same period, but the duty cycle can be independently set.
class Stm32PWMGroup
{
public:
    /// Constructor.
    ///
    /// @param base defines which timer resource to use.
    /// @param prescaler is 0 to 65535 (plus 1 will be the prescaling factor)
    /// @param period_counts is the timer period in clock cycles. Must be less
    /// than 65536 * prescaler.
    Stm32PWMGroup(TIM_TypeDef *base, uint16_t prescaler, uint32_t period_counts)
    {
        memset(&timHandle_, 0, sizeof(timHandle_));
        timHandle_.Instance = base;
        timHandle_.Init.Prescaler = prescaler;
        timHandle_.Init.ClockDivision = 0;
        timHandle_.Init.CounterMode = TIM_COUNTERMODE_UP;
        timHandle_.Init.RepetitionCounter = 0;
        timHandle_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        set_period(period_counts); // will call hal init.
        channels_[0].emplace(this, TIM_CHANNEL_1);
        channels_[1].emplace(this, TIM_CHANNEL_2);
        channels_[2].emplace(this, TIM_CHANNEL_3);
        channels_[3].emplace(this, TIM_CHANNEL_4);
    }

    /// @return one PWM channel.
    /// @param id is the channel number, 1..4
    PWM *get_channel(unsigned id)
    {
        HASSERT(id > 0 && id < 5);
        return &*channels_[id - 1];
    }

    /// @return one PWM channel.
    /// @param parent the PWMGroup object. Does not need to be initialized yet.
    /// @param id is the channel number, 1..4
    static constexpr PWM *get_channel(Stm32PWMGroup *parent, unsigned id)
    {
        return uninitialized<Channel>::cast_data(&parent->channels_[id - 1]);
    }

private:
    class Channel : public PWM
    {
    public:
        Channel(Stm32PWMGroup *parent, uint32_t channel)
            : parent_(parent)
            , channel_(channel)
        {
        }

        void set_period(uint32_t counts) override
        {
            parent_->set_period(counts);
        }
        uint32_t get_period() override
        {
            return parent_->timHandle_.Init.Prescaler *
                parent_->timHandle_.Init.Period;
        }
        void set_duty(uint32_t counts) override
        {
            lastDuty_ = counts;
            counts /= parent_->timHandle_.Init.Prescaler;
            HASSERT(counts <= 65535);
            TIM_OC_InitTypeDef config;
            config.OCMode = TIM_OCMODE_PWM1;
            config.OCPolarity = TIM_OCPOLARITY_HIGH;
            config.OCFastMode = TIM_OCFAST_DISABLE;
            config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
            config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
            config.OCIdleState = TIM_OCIDLESTATE_RESET;
            config.Pulse = counts;

            HASSERT(HAL_TIM_PWM_ConfigChannel(
                        &parent_->timHandle_, &config, channel_) == HAL_OK);
            HASSERT(
                HAL_TIM_PWM_Start(&parent_->timHandle_, channel_) == HAL_OK);
        }
        uint32_t get_duty() override
        {
            return lastDuty_;
            return 0;
        }
        uint32_t get_period_max() override
        {
            return 65535 * parent_->timHandle_.Init.Prescaler;
        }
        uint32_t get_period_min() override
        {
            return 2 * parent_->timHandle_.Init.Prescaler;
        }

    private:
        Stm32PWMGroup *parent_;
        uint32_t channel_;
        /// Last set duty cycle
        uint32_t lastDuty_{0};
    };

    friend class Channel;

    void set_period(uint32_t counts)
    {
        counts /= timHandle_.Init.Prescaler;
        timHandle_.Init.Period = counts;
        HASSERT(HAL_TIM_PWM_Init(&timHandle_) == HAL_OK);
    }

    /// HAL handle structure. Note this is extremely wasteful of RAM.
    TIM_HandleTypeDef timHandle_;

    uninitialized<Channel> channels_[4];
};

#endif // _FREEERTOS_DRIVERS_ST_STM32PWM_HXX_
