#ifdef __FreeRTOS__

#include "i2c_api.h"

#include "os/os.h"

os_sem_t i2c_sem[3];

int i2c_get_id(i2c_t* obj) {
  switch ((int)obj->i2c) {
  case I2C_0: return 0;
  case I2C_1: return 1;
  case I2C_2: return 2;
  }
  abort();
}

void i2c0_irq(void) INTERRUPT_ATTRIBUTE;
void i2c1_irq(void) INTERRUPT_ATTRIBUTE;
void i2c2_irq(void) INTERRUPT_ATTRIBUTE;


static const IRQn_Type i2c_irqn[3] = {
  I2C0_IRQn,
  I2C1_IRQn,
  I2C2_IRQn
};

void i2c0_irq(void) {
  os_sem_post_from_isr(i2c_sem+0);
  NVIC_DisableIRQ(i2c_irqn[0]);
#ifdef INTERRUPT_ACK
    LPC_VIC->Address = 0;
#endif
}

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


static const uint32_t irqptr[3] = {
  (uint32_t) &i2c0_irq,
  (uint32_t) &i2c1_irq,
  (uint32_t) &i2c2_irq,
};

void i2c_init_irq(i2c_t *obj) {
  int id = i2c_get_id(obj);
  os_sem_init(&i2c_sem[id], 0);
  NVIC_SetVector(i2c_irqn[id], irqptr[id]);
}

int i2c_wait_SI(i2c_t *obj) {
  static const long long kTimeoutNano = MSEC_TO_NSEC(500);
  int id = i2c_get_id(obj);
  while (!(obj->i2c->I2CONSET & (1 << 3))) {
    NVIC_EnableIRQ(i2c_irqn[id]);
    if (os_sem_timedwait(i2c_sem + id, kTimeoutNano) < 0) {
      return -1;
    }
  }
  return 0;
}

#endif // __FreeRTOS__
