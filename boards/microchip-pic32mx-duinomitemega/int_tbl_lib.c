/********************************************************************
 * FileName:        int_tbl_lipb.c
 * Dependencies:
 * Processor:       PIC32
 * Hardware:        N/A
 * Assembler:       N/A
 * Linker:          N/A
 * Company:         Microchip Technology Inc.
 *
 * Software License Agreement:
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * $Id:$
 * $Name:  $
 ********************************************************************/
#include <peripheral/int.h>

#  if ((__PIC32_FEATURE_SET__ >= 100) && (__PIC32_FEATURE_SET__ <= 299))
#   include "int_tbl_1xx_2xx_lib.c"
#elif (((__PIC32_FEATURE_SET__ >= 300) && (__PIC32_FEATURE_SET__ <= 499)) || defined(__32MXGENERIC__))
   #include "int_tbl_3xx_4xx_lib.c"
#elif (((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799)) || defined (__32MXPOCONO__))
#   include "int_tbl_5xx_6xx_7xx_lib.h"
#else
#   error "Device not supported by the interrupt peripheral library"
#endif

void INTClearFlag(INT_SOURCE source)
{
    INT_SCR_TBL_ENTRY   *tbl;

    tbl                 = (INT_SCR_TBL_ENTRY *)__IntSrcTbl + source;
    tbl->ifs[SFR_CLR]   = tbl->mask;
}
void INTSetFlag(INT_SOURCE source)
{
    INT_SCR_TBL_ENTRY   *tbl;

    tbl                 = (INT_SCR_TBL_ENTRY *)__IntSrcTbl + source;
    tbl->ifs[SFR_SET]   = tbl->mask;

}
unsigned int INTGetFlag(INT_SOURCE source)
{
    INT_SCR_TBL_ENTRY   *tbl;

    tbl                 = (INT_SCR_TBL_ENTRY *)__IntSrcTbl + source;
    return (*tbl->ifs &  tbl->mask);
}

void INTEnable(INT_SOURCE source, INT_EN_DIS enable)
{
    INT_SCR_TBL_ENTRY   *tbl;

    tbl                 = (INT_SCR_TBL_ENTRY *)__IntSrcTbl + source;

    if(enable)
        tbl->iec[SFR_SET]   = tbl->mask;
    else
        tbl->iec[SFR_CLR]   = tbl->mask;
}
unsigned int INTGetEnable(INT_SOURCE source)
{
    INT_SCR_TBL_ENTRY   *tbl;

    tbl                 = (INT_SCR_TBL_ENTRY *)__IntSrcTbl + source;
    return (*tbl->iec &  tbl->mask);
}
void INTSetVectorPriority(INT_VECTOR source, INT_PRIORITY priority)
{
    INT_VECTOR_TBL_ENTRY   *tbl;

    tbl                 = (INT_VECTOR_TBL_ENTRY *)__IntVectorTbl + source;

    tbl->ipc[SFR_CLR]   = (7 << tbl->pri_shift);
    priority            <<= tbl->pri_shift;
    tbl->ipc[SFR_SET]   = priority;

}
INT_PRIORITY INTGetVectorPriority(INT_VECTOR source)
{
    INT_VECTOR_TBL_ENTRY   *tbl;

    tbl                 = (INT_VECTOR_TBL_ENTRY *)__IntVectorTbl + source;
    return ((*tbl->ipc >>  tbl->pri_shift) & 7);
}
void INTSetVectorSubPriority(INT_VECTOR source, INT_SUB_PRIORITY subPriority)
{
    INT_VECTOR_TBL_ENTRY   *tbl;

    tbl                 = (INT_VECTOR_TBL_ENTRY *)__IntVectorTbl + source;
    tbl->ipc[SFR_CLR]   = (3 << tbl->sub_shift);
    subPriority         <<= tbl->sub_shift;
    tbl->ipc[SFR_SET]   = subPriority;

}
INT_SUB_PRIORITY INTGetVectorSubPriority(INT_VECTOR source)
{
    INT_VECTOR_TBL_ENTRY   *tbl;

    tbl                 = (INT_VECTOR_TBL_ENTRY *)__IntVectorTbl + source;
    return ((*tbl->ipc >>  tbl->sub_shift) & 3);
}


