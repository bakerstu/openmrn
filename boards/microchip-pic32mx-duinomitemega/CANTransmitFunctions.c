#include <xc.h>
#if defined(_CAN1) || defined(_CAN2)

#include "CANTypes.h"
#include <peripheral/CAN.h>


void CANConfigureChannelForTx(CAN_MODULE module,CAN_CHANNEL channel, 

		UINT channelSize, CAN_TX_RTR rtren,CAN_TXCHANNEL_PRIORITY priority)

{

	/* This function will configure a channel

	 * for transmit operation.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	canRegisters->canFifoRegisters[channel].CxFIFOCONSET = CxFIFOCON_TXEN_MASK; 

	canRegisters->canFifoRegisters[channel].CxFIFOCONbits.FSIZE = channelSize - 1;

	

	if(rtren == CAN_TX_RTR_ENABLED)

	{

		canRegisters->canFifoRegisters[channel].CxFIFOCONSET = CxFIFOCON_RTREN_MASK;

	}

	else if(rtren == CAN_TX_RTR_DISABLED)

	{

		canRegisters->canFifoRegisters[channel].CxFIFOCONCLR = CxFIFOCON_RTREN_MASK;

	}

    else

    {

    }



	canRegisters->canFifoRegisters[channel].CxFIFOCONbits.TXPRI = priority;

}



void CANAbortPendingTx(CAN_MODULE module, CAN_CHANNEL channel )

{

	/* This function will abort an ongoing

	 * transmission. 	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(channel == CAN_ALL_CHANNELS)

	{

		canRegisters->CxCONSET = CxCON_ABAT_MASK;	

	}

	else

	{

		canRegisters->canFifoRegisters[channel].CxFIFOCONCLR = CxFIFOCON_TXREQ_MASK;

	}	





}



BOOL CANIsTxAborted(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function will return true if the 

	 * CAN transmission abort was successful.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(channel == CAN_ALL_CHANNELS)

	{

		return((BOOL)(0x1 ^ (canRegisters->CxCONbits.ABAT)));	

	}

	else

	{

		return((BOOL)(canRegisters->canFifoRegisters[channel].CxFIFOCONbits.TXABAT));

	}	



}



void CANFlushTxChannel(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function will make the TX channel

	 * send all the messages in the channel.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	canRegisters->canFifoRegisters[channel].CxFIFOCONSET = CxFIFOCON_TXREQ_MASK;

}





CAN_TX_CHANNEL_CONDITION CANGetTxChannelCondition(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function returns the current 

	 * TX channel condition. The return

	 * value can be masked to check if

	 * a specific tx channel condition is

	 * active. 

	 */

	

	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	return((canRegisters->canFifoRegisters[channel].CxFIFOCON) & (UINT32)0x38);

}

	

CANTxMessageBuffer * CANGetTxMessageBuffer(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function returns a pointer

	 * to an available message buffer in

	 * the channel. If the channel is full

	 * the function will return null. */



	

	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	if(canRegisters->canFifoRegisters[channel].CxFIFOINTbits.TXNFULLIF == 1)

	{

		return((CANTxMessageBuffer *)PA_TO_KVA1(canRegisters->canFifoRegisters[channel].CxFIFOUA));

	}

	else

	{

		return(NULL);

	}

}	
#endif /* defined(_CAN1) || defined(_CAN2) */


