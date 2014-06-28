#ifndef _FREERTOS_DRIVERS_PIC32MX_BUILTINS_H_
#define _FREERTOS_DRIVERS_PIC32MX_BUILTINS_H_


#define __builtin_mfc0(reg, sel) \
__extension__ ({ \
  register unsigned long __r; \
  __asm__ __volatile__ ("mfc0 %0,$%1,%2" \
                        : "=d" (__r) \
                        : "JK" (reg), "JK" (sel)); \
  __r; \
})

#define __builtin_mtc0(reg, sel, val) \
do { \
    __asm__ __volatile__ ("%(mtc0 %z0,$%1,%2; ehb%)" \
                          : \
                          : "dJ" ((_reg_t)(val)), "JK" (reg), "JK" (sel) \
                          : "memory"); \
} while (0)

/* exchange (swap) VAL and CP0 register REG,SEL */
#define __builtin_mxc0(reg, sel, val) \
__extension__ ({ \
    register _reg_t __o; \
    __o = _mfc0 (reg, sel); \
    _mtc0 (reg, sel, val); \
    __o; \
})

/* bit clear non-zero bits from CLR in CP0 register REG,SEL */
#define __builtin_bcc0(reg, sel, clr) \
__extension__ ({ \
    register _reg_t __o; \
    __o = _mfc0 (reg, sel); \
    _mtc0 (reg, sel, __o & ~(clr)); \
    __o; \
})

/* bit set non-zero bits from SET in CP0 register REG,SEL */
#define __builtin_bsc0(reg, sel, set) \
__extension__ ({ \
    register _reg_t __o; \
    __o = _mfc0 (reg, sel); \
    _mtc0 (reg, sel, __o | (set)); \
    __o; \
})

/* bit clear non-zero bits from CLR and set non-zero bits from SET in REG,SEL */
#define __builtin_bcsc0(reg, sel, clr, set) \
__extension__ ({ \
    register _reg_t __o; \
    __o = _mfc0 (reg, sel); \
    _mtc0 (reg, sel, (__o & ~(clr)) | (set)); \
    __o; \
})

#endif // _FREERTOS_DRIVERS_PIC32MX_BUILTINS_H_
