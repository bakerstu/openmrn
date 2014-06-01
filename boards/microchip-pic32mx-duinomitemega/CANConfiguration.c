#include <xc.h>
#if defined(_CAN1) || defined(_CAN2)

#include "CANTypes.h"
#include <peripheral/CAN.h>


/*******************************************************************

  This array stores the base address of each CAN module on the

  device.

  *******************************************************************/

const CAN_REGISTERS * canModules[CAN_NUM_OF_MODULES] = 

{
#ifdef CAN1_BASE_ADDRESS

	(CAN_REGISTERS *)CAN1_BASE_ADDRESS
#endif
#ifdef CAN2_BASE_ADDRESS

	,(CAN_REGISTERS *)CAN2_BASE_ADDRESS 
#endif

};



/*******************************************************************

  This array stores the mapping between a filter and its filter control

  register. 

  *******************************************************************/

const CAN_FLTCON_BYTES canFilterControlMap[CAN_NUM_OF_FILTERS] = 

{

	{CAN_FILTER_CONTROL0,0,0x80 	 },

	{CAN_FILTER_CONTROL0,1,0x8000 	 },

	{CAN_FILTER_CONTROL0,2,0x800000	 },

	{CAN_FILTER_CONTROL0,3,0x80000000},

	{CAN_FILTER_CONTROL1,0,0x80 	 },

	{CAN_FILTER_CONTROL1,1,0x8000 	 },

	{CAN_FILTER_CONTROL1,2,0x800000	 },

	{CAN_FILTER_CONTROL1,3,0x80000000},

	{CAN_FILTER_CONTROL2,0,0x80 	 },

	{CAN_FILTER_CONTROL2,1,0x8000 	 },

	{CAN_FILTER_CONTROL2,2,0x800000	 },

	{CAN_FILTER_CONTROL2,3,0x80000000},

	{CAN_FILTER_CONTROL3,0,0x80 	 },

	{CAN_FILTER_CONTROL3,1,0x8000 	 },

	{CAN_FILTER_CONTROL3,2,0x800000	 },

	{CAN_FILTER_CONTROL3,3,0x80000000},

	{CAN_FILTER_CONTROL4,0,0x80 	 },

	{CAN_FILTER_CONTROL4,1,0x8000 	 },

	{CAN_FILTER_CONTROL4,2,0x800000	 },

	{CAN_FILTER_CONTROL4,3,0x80000000},

	{CAN_FILTER_CONTROL5,0,0x80 	 },

	{CAN_FILTER_CONTROL5,1,0x8000 	 },

	{CAN_FILTER_CONTROL5,2,0x800000	 },

	{CAN_FILTER_CONTROL5,3,0x80000000},

	{CAN_FILTER_CONTROL6,0,0x80 	 },

	{CAN_FILTER_CONTROL6,1,0x8000 	 },

	{CAN_FILTER_CONTROL6,2,0x800000	 },

	{CAN_FILTER_CONTROL6,3,0x80000000},

	{CAN_FILTER_CONTROL7,0,0x80 	 },

	{CAN_FILTER_CONTROL7,1,0x8000 	 },

	{CAN_FILTER_CONTROL7,2,0x800000	 },

	{CAN_FILTER_CONTROL7,3,0x80000000}

};                        





void CANSetTimeStampValue(CAN_MODULE module,UINT value)

{

	/* This function write the provided value 

	 * to the CAN RX time stamp timer. */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	canRegisters->CxTMRbits.CANTS = value;



}



UINT CANGetTimeStampValue(CAN_MODULE module)

{

	/* This function returns the current value of the

	 * CAN RX time stamp timer. */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	return(canRegisters->CxTMRbits.CANTS);



}



void CANSetTimeStampPrescalar(CAN_MODULE module,UINT prescalar)

{

	/* This function sets the time

	 * stamp time prescalar. 

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	canRegisters->CxTMRbits.CANTSPRE = prescalar;



}





void CANSetSpeed(CAN_MODULE module, const CAN_BIT_CONFIG * canBitConfig, UINT32 sysClock

                    , UINT32 canBusSpeed    )

{

	/* This function sets up the CAN bit time quanta

	 * specifications and the CAN bus speed. */



    UINT totalTq;

	UINT prescalar;



	totalTq = (canBitConfig->phaseSeg1Tq + 1) + (canBitConfig ->phaseSeg2Tq + 1)

		     + (canBitConfig->propagationSegTq + 1) + 1;



	prescalar = (sysClock/(canBusSpeed * totalTq * 2)) - 1;





	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(canBitConfig->phaseSeg2TimeSelect == TRUE)

	{

		canRegisters->CxCFGSET 	= CxCFG_SEG2PHTS_MASK;

	}

	else

	{

		canRegisters->CxCFGCLR 	= CxCFG_SEG2PHTS_MASK;

	}



	if(canBitConfig->sample3Time == TRUE)

	{

		canRegisters->CxCFGSET 	= CxCFG_SAM_MASK;

	}

	else

	{

		canRegisters->CxCFGCLR 	= CxCFG_SAM_MASK;

	}

	canRegisters->CxCFGbits.SEG1PH 		= canBitConfig->phaseSeg1Tq;

	canRegisters->CxCFGbits.SEG2PH 		= canBitConfig->phaseSeg2Tq;

	canRegisters->CxCFGbits.PRSEG 		= canBitConfig->propagationSegTq;

	canRegisters->CxCFGbits.SJW			= canBitConfig->syncJumpWidth;

    canRegisters->CxCFGbits.BRP			= prescalar;	



}









void CANEnableFeature(CAN_MODULE module, CAN_MODULE_FEATURES features, BOOL enable)

{

	/* This function will enable / disable the 

	 * specified CAN module feature. Note that

	 * this function modifies two registers. The

	 * CAN stop in idle and rx time stamp are

	 * located in the CxCON register while the 

	 * rx line filter is located in the CxCFG 

	 * register

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(enable == TRUE)

	{

		/* The logical AND over here isolates the bits

		 * that need to be changed in respective registers */



		canRegisters->CxCONSET = features & (UINT32)(0x102000); 

		canRegisters->CxCFGSET = features & (UINT32)(0x400000);



	}

	else

	{

		/* Disable specified feature. */

		canRegisters->CxCONCLR = features & (UINT32)(0x102000); 

		canRegisters->CxCFGCLR = features & (UINT32)(0x400000);

	}

}





void CANDeviceNetFilter(CAN_MODULE module, CAN_DNET_FILTER_SIZE dncnt)

{

	/* This function sets the size of the

	 * device net filter. */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	canRegisters->CxCONbits.DNCNT = dncnt;



}



CAN_OP_MODE CANGetOperatingMode (CAN_MODULE module)

{

	/* This function returns the current CAN 

	 * operating mode.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	return(canRegisters->CxCONbits.OPMOD);



}

void CANSetOperatingMode(CAN_MODULE module, CAN_OP_MODE opmode)

{

	/* This function sets the CAN module operating

	 * mode.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	canRegisters->CxCONbits.REQOP = opmode;

}





void CANAssignMemoryBuffer(CAN_MODULE module, void * buffer, UINT sizeInBytes)

{

	/* This function converts the provided

	 * buffer address to a physical address

	 * and stores this address in CxFIFOBA

	 * register.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	canRegisters->CxFIFOBA = KVA_TO_PA(buffer);

}





void CANEnableModule(CAN_MODULE module, BOOL enable)

{

	/* This function enables the CAN module (sets the

	 * ON bit in the  CxCON register if enable is TRUE

	 * else it disables the CAN module.*/



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(enable == TRUE)

	{

		canRegisters->CxCONSET = CxCON_ON_MASK;

	}

	else

	{

		canRegisters->CxCONCLR = CxCON_ON_MASK;

	}

}



BOOL CANIsActive(CAN_MODULE module)

{

	/* This function returns the status

	 * of the BUSY bit in the CxCON register.

	 * This bit is polled after the CAN module

	 * is disabled.

	 */

	

	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(canRegisters->CxCONbits.CANBUSY == 1)

	{

		return(TRUE);

	}

	else

	{

		return(FALSE);

	}



}

void CANResetChannel(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function will reset the channel fifo.

	 * Note that this resets the channel fifo

	 * head and tail pointers and hence the channel

	 * events as well. 

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	canRegisters->canFifoRegisters[channel].CxFIFOCONSET = CxFIFOCON_FRESET_MASK;

}




BOOL CANIsChannelReset(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function returns the current

	 * state of the FRESET bit corresponding

	 * to the specified channel. If the reset

	 * bit is clear then the channel reset is 

	 * complete*/

	

	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(canRegisters->canFifoRegisters[channel].CxFIFOCONbits.FRESET == 1)

	{

		return(FALSE);

	}

	else

	{

		return(TRUE);

	}



}


#endif /* defined(_CAN1) || defined(_CAN2) */

