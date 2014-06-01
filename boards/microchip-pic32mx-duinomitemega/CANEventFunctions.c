#include <xc.h>
#if defined(_CAN1) || defined(_CAN2)

#include "CANTypes.h"
#include <peripheral/CAN.h>


CAN_EVENT_CODE CANGetPendingEventCode(CAN_MODULE module)

{

	/* This function will return the event

	 * code of the highest priority active event in

	 * the specified CAN module.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	return(canRegisters->CxVECbits.ICODE);





}



CAN_CHANNEL_MASK CANGetAllChannelEventStatus(CAN_MODULE module)

{

	/* This function will return the event

	 * status of all channels. The result can

	 * be masked to check if a specific channel

	 * has an active event.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	return(canRegisters->CxFSTAT);



}



CAN_CHANNEL_MASK CANGetAllChannelOverflowStatus(CAN_MODULE module)

{

	/* This function will return the overflow

	 * status of all channels. The result can

	 * be masked to check if a specific channel

	 * has overflowed.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	return(canRegisters->CxRXOVF);



}



CAN_CHANNEL_EVENT CANGetChannelEvent(CAN_MODULE module, CAN_CHANNEL channel)

{

	/* This function returns the CAN channel

	 * level event flags. The return value can

	 * then be masked to check for specific

	 * conditions.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	return((canRegisters->canFifoRegisters[channel].CxFIFOINT) & (UINT32)0xFFFF);





}

CAN_MODULE_EVENT CANGetModuleEvent (CAN_MODULE module)

{

	/* This function returns the CAN module

	 * level event flags. The return value can

	 * then be masked to check for specific

	 * conditions.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	return((canRegisters->CxINT) & (UINT32)0xFFFF);



}

void CANClearChannelEvent(CAN_MODULE module, CAN_CHANNEL channel, CAN_CHANNEL_EVENT events)

{

	/* This function clears the channel

	 * events. Note that only the RX channel

	 * overflow event can be cleared. Attempting

	 * to clear TX channel event or other

	 * RX channel events will have no effect.

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	canRegisters->canFifoRegisters[channel].CxFIFOINTCLR= events;

}



void CANEnableModuleEvent(CAN_MODULE module, CAN_MODULE_EVENT flags, BOOL enable)

{

	/* This function enables or disables

	 * module level event. It does this

	 * by setting clearing bits in CxINT

	 * register. The flags need to be shifted

	 * to upper 16 bits of the 32 bit word because

	 * the event enable disable bits are located 

	 * in the upper 16 bits of the CxINT register.

	 */



	UINT32 mask;



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	mask = ((UINT32)flags) << 16;



	if(enable == TRUE)

	{

		canRegisters->CxINTSET = mask;

	}

	else

	{

		canRegisters->CxINTCLR = mask;



	}



}



void CANClearModuleEvent(CAN_MODULE module, CAN_MODULE_EVENT flags)

{

	/* This function will clear module level

	 * event flags. It does this by clearing

	 * the event flags in the CxINT register

	 */



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	canRegisters->CxINTCLR = flags;

}



void CANEnableChannelEvent(CAN_MODULE module, CAN_CHANNEL channel, CAN_CHANNEL_EVENT flags, BOOL enable	)

{

	/* This function will enable disable the 

	 * channel level events for the specified

	 * channel. This is done by setting clearing

	 * bits in the CxFIFOINT register. The enable

	 * disable bits are in the upper 16 bits of

	 * the 32 bit of the CxFIFOINT register.

	 */





	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	flags = flags << 16;



	if(enable == TRUE)

	{

		canRegisters->canFifoRegisters[channel].CxFIFOINTSET = flags;

	}

	else

	{

		canRegisters->canFifoRegisters[channel].CxFIFOINTCLR = flags;

	}

}


#endif /* defined(_CAN1) || defined(_CAN2) */

