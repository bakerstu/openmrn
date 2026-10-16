/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file main.cxx
 *
 * Main file for the io board application on the Tiva Launchpad board.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/MemoryConfig.hxx"

#include "driverlib/sysctl.h"

#include "freertos_drivers/common/MCP23017GPIO.hxx"
#include "freertos_drivers/common/PCA9685PWM.hxx"
#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/PersistentGPIO.hxx"

#include "utils/format_utils.hxx"

#include "config.hxx"
#include "hardware.hxx"

// These preprocessor symbols are used to select which physical connections
// will be enabled in the main(). See @ref appl_main below.
#define SNIFF_ON_SERIAL
//#define SNIFF_ON_USB
#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011901ULL;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
openlcb::ConfigDef cfg(0);
// Defines weak constants used by the stack to tell it which device contains
// the volatile configuration information. This device name appears in
// HwInit.cxx that creates the device drivers.
extern const char *const openlcb::CONFIG_FILENAME = "/dev/eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 1500, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

// Defines the GPIO ports used for the producers and the consumers.


/** interrupt_enable */
void ief()
{
  GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5);
}

/** interrupt_disable */
void idf()
{
  GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_5);
}

MCP23017<1> the_port(ief,idf, MSEC_TO_NSEC(10));
MCP23017GPIO the_light(&the_port,0);
MCP23017GPIO the_button(&the_port,1);

#define PWM
#ifdef PWM
PCA9685PWM pca9685pwm;
PCA9685PWMBit pwmBit0(&pca9685pwm, 0);
PCA9685PWMBit pwmBit1(&pca9685pwm, 1);
PCA9685PWMBit pwmBit2(&pca9685pwm, 2);
PCA9685PWMBit pwmBit3(&pca9685pwm, 3);
PCA9685PWMBit pwmBit4(&pca9685pwm, 4);
PCA9685PWMBit pwmBit5(&pca9685pwm, 5);
PCA9685PWMBit pwmBit6(&pca9685pwm, 6);
PCA9685PWMBit pwmBit7(&pca9685pwm, 7);
PCA9685PWMBit pwmBit8(&pca9685pwm, 8);
PCA9685PWMBit pwmBit9(&pca9685pwm, 9);
PCA9685PWMBit pwmBit10(&pca9685pwm, 10);
PCA9685PWMBit pwmBit11(&pca9685pwm, 11);
PCA9685PWMBit pwmBit12(&pca9685pwm, 12);
PCA9685PWMBit pwmBit13(&pca9685pwm, 13);
PCA9685PWMBit pwmBit14(&pca9685pwm, 14);
PCA9685PWMBit pwmBit15(&pca9685pwm, 15);
PWMGPO servo0(&pwmBit0, 0xB4, 0x12C);
PWMGPO servo1(&pwmBit1, 0xB4, 0x12C);
PWMGPO servo2(&pwmBit2, 0xB4, 0x12C);
PWMGPO servo3(&pwmBit3, 0xB4, 0x12C);
PWMGPO servo4(&pwmBit4, 0xB4, 0x12C);
PWMGPO servo5(&pwmBit5, 0xB4, 0x12C);
PWMGPO servo6(&pwmBit6, 0xB4, 0x12C);
PWMGPO servo7(&pwmBit7, 0xB4, 0x12C);
PWMGPO servo8(&pwmBit8, 0xB4, 0x12C);
PWMGPO servo9(&pwmBit9, 0xB4, 0x12C);
PWMGPO servo10(&pwmBit10, 0xB4, 0x12C);
PWMGPO servo11(&pwmBit11, 0xB4, 0x12C);
PWMGPO servo12(&pwmBit12, 0xB4, 0x12C);
PWMGPO servo13(&pwmBit13, 0xB4, 0x12C);
PWMGPO servo14(&pwmBit14, 0xB4, 0x12C);
PWMGPO servo15(&pwmBit15, 0xB4, 0x12C);
#endif
// These wrappers will save the output pin state to EEPROM.
constexpr PersistentGpio PinRed(LED_RED_Pin::instance(), 0);
constexpr PersistentGpio PinGreen(LED_GREEN_Pin::instance(), 1);
constexpr PersistentGpio PinBlue(LED_BLUE_Pin::instance(), 2);

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const kOutputGpio[] = {&PinRed,
                                             &PinGreen,
                                             &PinBlue,
                                             &the_light};

#ifdef PWM
constexpr const Gpio *const servoOutputGpio[] =
{
    &servo0,  &servo1,  &servo2,  &servo3,
    &servo4,  &servo5,  &servo6,  &servo7,
    &servo8,  &servo9,  &servo10, &servo11,
    &servo12, &servo13, &servo14, &servo15,
};
#endif
// Instantiates the actual producer and consumer objects for the given GPIO
// pins from above. The MultiConfiguredConsumer class takes care of most of the
// complicated setup and operation requirements. We need to give it the virtual
// node pointer, the hardware pin definition and the configuration from the CDI
// definition. The virtual node pointer comes from the stack object. The
// configuration structure comes from the CDI definition object, segment 'seg',
// in which there is a repeated group 'consumers'. The GPIO pins get assigned
// to the repetitions in the group in order.
openlcb::MultiConfiguredConsumer consumers(
    stack.node(), kOutputGpio, ARRAYSIZE(kOutputGpio), cfg.seg().consumers());
#ifdef PWM
openlcb::MultiConfiguredConsumer servos(stack.node(), servoOutputGpio,
                        ARRAYSIZE(servoOutputGpio), cfg.seg().servos());
#endif
// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_sw1(
    stack.node(), cfg.seg().producers().entry<0>(), SW1_Pin());
openlcb::ConfiguredProducer producer_sw2(
    stack.node(), cfg.seg().producers().entry<1>(), SW2_Pin());
openlcb::ConfiguredProducer producer_sw3(
    stack.node(), cfg.seg().producers().entry<2>(), (const Gpio*)&the_button);

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(
			  stack.node(), {producer_sw1.polling(), producer_sw2.polling(), producer_sw3.polling()});


class Configuration : private DefaultConfigUpdateListener {
private:

  /// Notifies the component that there is new configuration available for
  /// loading.
  ///
  /// The call is made on the main executor, so the call must not
  /// block. Reading the given EEPROM device should be fine. Asynchronous
  /// operations may be implemented by using a special return status RETRY:
  /// the runner will call the same method on the same component once more
  /// after the done callback is invoked. This allows implementing state
  /// machines.
  ///
  /// @param fd is the file descriptor for the EEPROM file. The current
  /// offset in this file is unspecified, callees must do lseek.
  /// @param initial_load is true if this is the first load upon starting the
  /// binary.
  /// @param done must be notified when the call and its dependent actions
  /// are complete. No other configuration component will be called until the
  /// done callback is invoked.
  ///
  /// @return any necessary action. If returns UPDATED, then assumes that the
  /// configuration change was applied. If returns RETRY, then the same call
  /// will be made again after the notifiable is called. If return
  /// REINIT_NEEDED or REBOOT_NEEDED then at the end of the configuration
  /// update process the node will be reinitialized or rebooted accordingly.
  UpdateAction apply_configuration(int fd, bool initial_load,
				   BarrierNotifiable *done) {
    done->notify();
    if (initial_load)
      return UPDATED;
    
    return REBOOT_NEEDED;
  } // apply_configuration
  
  /// Clears configuration file and resets the configuration settings to
  /// factory value.
  ///
  /// @param fd is the file descriptor for the EEPROM file. The current
  /// offset in this file is unspecified, callees must do lseek.
  void factory_reset(int fd)
  {
    for( int i=0; i<NUM_INPUTS;++i) {
      //char tmp[10];
      //string str;
      //unsigned_integer_to_buffer(i, tmp);
      //cfg.seg().producers().entry(i).description().write(fd,tmp);
      cfg.seg().producers().entry(i).debounce().write(fd,3);
      cfg.seg().producers().entry(i).event_on().write(fd,(NODE_ID << 16) + 8+(i*2) );
      cfg.seg().producers().entry(i).event_off().write(fd,(NODE_ID << 16) + 9+(i*2) );						    
    } // for
    for( int i=0; i<NUM_OUTPUTS;++i) {
      cfg.seg().consumers().entry(i).event_on().write(fd, (NODE_ID << 16) + 0+(i*2) );
      cfg.seg().consumers().entry(i).event_off().write(fd, (NODE_ID << 16) + 1+(i*2) );
    } // for

#ifdef PWM
    for (int i = 0; i < NUM_SERVOS; ++i)
    {
        cfg.seg().servos().entry(i).event_on().write(fd, (NODE_ID << 16) + 32 + (i*2));
        cfg.seg().servos().entry(i).event_off().write(fd, (NODE_ID << 16) + 33 + (i*2));
    }
#endif
  } // factory_reset
} my_updater;


extern "C"
{
  void reboot()
  {
    SysCtlReset();
  } // reboot  
} // extern


/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.check_version_and_factory_reset(
        cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, false);

    // Restores pin states from EEPROM.
    PinRed.restore();
    PinGreen.restore();
    PinBlue.restore();

    the_port.init("/dev/i2c1",0x21);
#ifdef PWM
    pca9685pwm.init("/dev/i2c1", 0x40, 50, -1);
#endif
    the_light.set_direction(Gpio::Direction::OUTPUT);
    
        for ( ; /** forever */ ; )
        {
            the_light.set();
            usleep(1000000);
            the_light.clr();
            usleep(2000000);
          }
    
    // The necessary physical ports must be added to the stack.
    //
    // It is okay to enable multiple physical ports, in which case the stack
    // will behave as a bridge between them. For example enabling both the
    // physical CAN port and the USB port will make this firmware act as an
    // USB-CAN adapter in addition to the producers/consumers created above.
    //
    // If a port is enabled, it must be functional or else the stack will
    // freeze waiting for that port to send the packets out.
#if defined(HAVE_PHYSICAL_CAN_PORT)
    stack.add_can_port_select("/dev/can0");
#endif
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif

    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
