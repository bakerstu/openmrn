#include <xc.h>
#if defined(_CAN1) || defined(_CAN2)

#include "CANTypes.h"

#include <peripheral/CAN.h>



void CANConfigureFilterMask(CAN_MODULE module, CAN_FILTER_MASK mask, UINT32 maskbits, CAN_ID_TYPE maskType, CAN_FILTER_MASK_TYPE mide)

{

	/* This function will configure the specified 

	 * mask. */



	UINT sid;

	UINT eid;



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(maskType == CAN_EID)

	{

		/* Extract the sid and eid from

		 * the specified id and assign these to

		 * the register fields. */



		maskbits &= 0x1FFFFFFF;

		sid = (maskbits & 0x1FFC0000) >> 18;

		eid = maskbits & 0x3FFFF;

		canRegisters->canFilterMaskRegs[mask].CxRXMbits.SID = sid;

		canRegisters->canFilterMaskRegs[mask].CxRXMbits.EID = eid;

	}

	else if(maskType == CAN_SID)

	{

		maskbits &= 0x7FF;

		canRegisters->canFilterMaskRegs[mask].CxRXMbits.SID = maskbits;

	}

	else

	{

	}



	if(mide == CAN_FILTER_MASK_IDE_TYPE)

	{

		/* This means masking will also

		 * compare the message type.

		 */

	

		canRegisters->canFilterMaskRegs[mask].CxRXMSET = CxRXM_MIDE_MASK ;

	}

	else if(mide == CAN_FILTER_MASK_ANY_TYPE)

	{

		/* This means any message is masked.

		 */

		canRegisters->canFilterMaskRegs[mask].CxRXMCLR = CxRXM_MIDE_MASK ;

	}

	else

	{

	}





}



void CANConfigureFilter(CAN_MODULE module, CAN_FILTER filter, UINT32 id, CAN_ID_TYPE filterType)

{

	/* This function will configure the specified 

	 * filter. */



	UINT sid;

	UINT eid;



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	if(filterType == CAN_EID)

	{

		/* An extended ID filter has a 29 bit id.

		 * The most significant 11 bits are the SID

		 * and the rest of the 18 bits are EID. */



		id &= 0x1FFFFFFF;

		sid = (id & 0x1FFC0000) >> 18;

		eid = id & 0x3FFFF;

		canRegisters->canFilterRegs[filter].CxRXFbits.SID = sid;

		canRegisters->canFilterRegs[filter].CxRXFbits.EID = eid;

		canRegisters->canFilterRegs[filter].CxRXFSET = CxRXF_EXID_MASK ;



	}

	else if(filterType == CAN_SID)

	{

		/* An standard ID has 11 bits. */

		id &= 0x7FF;

		canRegisters->canFilterRegs[filter].CxRXFbits.SID = id;

		canRegisters->canFilterRegs[filter].CxRXFCLR = CxRXF_EXID_MASK ;



	}



}



void CANEnableFilter(CAN_MODULE module, CAN_FILTER filter, BOOL enable)

{

	/* This function enables or disables the specified filter*/



	UINT mask;

	CAN_FILTER_CONTROL fltcon;



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	

	mask 	= canFilterControlMap[filter].fltEnMask;

	fltcon 	= canFilterControlMap[filter].fltcon;



	if(enable == TRUE)

	{

		canRegisters->canFilterControlRegs[fltcon].CxFLTCONSET = mask;

	}

	else

	{

		canRegisters->canFilterControlRegs[fltcon].CxFLTCONCLR = mask;

	}

}



BOOL CANIsFilterDisabled(CAN_MODULE module, CAN_FILTER filter)

{

	/* This function returns TRUE if the filter is disabled */



	UINT mask;

	CAN_FILTER_CONTROL fltcon;

    UINT maskResult;



	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];

	

	mask 	= canFilterControlMap[filter].fltEnMask;

	fltcon 	= canFilterControlMap[filter].fltcon;



    maskResult = canRegisters->canFilterControlRegs[fltcon].CxFLTCONSET & mask;



    if(maskResult != 0)

    {

        /* This means the filter is still enabled.

         */



        return(FALSE);

    }

    else

    {

        return(TRUE);

    }

}





CAN_FILTER CANGetLatestFilterHit(CAN_MODULE module)

{

	/* This function returns the latest

	 * filter which accepted a messge. */

	

	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	return(canRegisters->CxVECbits.FILHIT);

}



void CANLinkFilterToChannel(CAN_MODULE module, CAN_FILTER filter, CAN_FILTER_MASK mask, CAN_CHANNEL channel)

{

	/* This function will link a filter to

	 * a channel. fltcon is the filter control

	 * register that is associated with the

	 * filter and sub index provides the shift

	 * amount with in the filter control register.

	 * The filter control register has four bytes

	 * each controlling a filter. The contents if

	 * the other filter fields should not be changed.

	 * */



	/* The canFilterControlMap provides a mapping

	 * between the bytes in the filter control

	 * register and the specified filter.

	 *

	 * For example, CAN_FILTER6, fltcon would be

	 * CAN_FILTER_CONTROL1. byteIndex would be 3 because

	 * the 3rd byte in CAN_FILTER_CONTROL1 control

	 * CAN_FILTER6. And the fltEnMask would be 0x800000.

	 */



	CAN_FILTER_CONTROL fltcon;

	UINT 	byteIndex;

	UINT32 	controlWord = 0;

	

	CAN_REGISTERS * canRegisters = (CAN_REGISTERS *)canModules[module];



	fltcon 		= canFilterControlMap[filter].fltcon;

	byteIndex 	= canFilterControlMap[filter].byteIndex;

	

	/* Form the control word and clear word and shift

	 * each by the subIndex.*/

	controlWord = (mask << 5)|channel;



	/* Clear the field and then set the bits. */

	canRegisters->canFilterControlRegs[fltcon].CxFLTCONbyte[byteIndex] = controlWord;

}
#endif /* defined(_CAN1) || defined(_CAN2) */


