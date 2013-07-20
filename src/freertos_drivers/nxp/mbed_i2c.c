#ifdef __FreeRTOS__

#include "i2c_api.h"

#include "os/os.h"

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
#define I2CCOUNT 3
#elif defined(TARGET_LPC11Cxx)
#define I2CCOUNT 1
#define i2c0_irq I2C_IRQHandler
#define INTERRUPT_ATTRIBUTE
#else
#error CPU undefined
#endif


os_sem_t i2c_sem[I2CCOUNT];

int i2c_get_id(i2c_t* obj) {
  switch ((int)obj->i2c) {
  case I2C_0: return 0;
#if I2CCOUNT > 1
  case I2C_1: return 1;
  case I2C_2: return 2;
#endif
  }
  abort();
}

void i2c0_irq(void) INTERRUPT_ATTRIBUTE;
#if I2CCOUNT > 1
void i2c1_irq(void) INTERRUPT_ATTRIBUTE;
void i2c2_irq(void) INTERRUPT_ATTRIBUTE;
#endif


static const IRQn_Type i2c_irqn[I2CCOUNT] = {
#if I2CCOUNT > 1
  I2C0_IRQn,
  I2C1_IRQn,
  I2C2_IRQn
#else
  I2C_IRQn,
#endif
};

void i2c0_irq(void) {
  os_sem_post_from_isr(i2c_sem+0);
  NVIC_DisableIRQ(i2c_irqn[0]);
#ifdef INTERRUPT_ACK
    LPC_VIC->Address = 0;
#endif
}

#if I2CCOUNT > 1
void i2c1_irq(void) {
  os_sem_post_from_isr(i2c_sem+1);
  NVIC_DisableIRQ(i2c_irqn[1]);
#ifdef INTERRUPT_ACK
    LPC_VIC->Address = 0;
#endif
}

void i2c2_irq(void) {
  os_sem_post_from_isr(i2c_sem+2);
  NVIC_DisableIRQ(i2c_irqn[2]);
#ifdef INTERRUPT_ACK
    LPC_VIC->Address = 0;
#endif
}
#endif

static const uint32_t irqptr[I2CCOUNT] = {
  (uint32_t) &i2c0_irq,
#if I2CCOUNT > 1
  (uint32_t) &i2c1_irq,
  (uint32_t) &i2c2_irq,
#endif
};

void i2c_init_irq(i2c_t *obj) {
  int id = i2c_get_id(obj);
  os_sem_init(&i2c_sem[id], 0);
#if I2CCOUNT > 1
  NVIC_SetVector(i2c_irqn[id], irqptr[id]);
#endif
}

int i2c_wait_SI(i2c_t *obj) {
  static const long long kTimeoutNano = MSEC_TO_NSEC(500);
  int id = i2c_get_id(obj);
#ifdef TARGET_LPC11Cxx
  while (!(obj->i2c->CONSET & (1 << 3))) {
#else
  while (!(obj->i2c->I2CONSET & (1 << 3))) {
#endif
    NVIC_EnableIRQ(i2c_irqn[id]);
    if (os_sem_timedwait(i2c_sem + id, kTimeoutNano) < 0) {
      return -1;
    }
  }
  return 0;
}

#endif // __FreeRTOS__
