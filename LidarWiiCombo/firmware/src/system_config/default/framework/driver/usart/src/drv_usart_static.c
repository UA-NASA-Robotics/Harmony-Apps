/*******************************************************************************
  USART Driver Static implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_usart_static.c

  Summary:
    Source code for the USART driver static implementation.

  Description:
    The USART device driver provides a simple interface to manage the USART
    modules on Microchip microcontrollers. This file contains static implementation
    for the USART driver.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver static object . */
DRV_USART_OBJ  gDrvUSART0Obj ;

// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_USART0_Initialize(void)
{
    uint32_t clockSource;

    /* Disable the USART module to configure it*/
    PLIB_USART_Disable (USART_ID_1);

    /* Initialize the USART based on configuration settings */
    PLIB_USART_InitializeModeGeneral(USART_ID_1,
            false,  /*Auto baud*/
            false,  /*LoopBack mode*/
            false,  /*Auto wakeup on start*/
            false,  /*IRDA mode*/
            false);  /*Stop In Idle mode*/

    /* Set the line control mode */
    PLIB_USART_LineControlModeSelect(USART_ID_1, DRV_USART_LINE_CONTROL_8NONE1);

    /* We set the receive interrupt mode to receive an interrupt whenever FIFO
       is not empty */
    PLIB_USART_InitializeOperation(USART_ID_1,
            USART_RECEIVE_FIFO_ONE_CHAR,
            USART_TRANSMIT_FIFO_IDLE,
            USART_ENABLE_TX_RX_USED);

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Set the baud rate and enable the USART */
    PLIB_USART_BaudSetAndEnable(USART_ID_1,
            clockSource,
            115200);  /*Desired Baud rate value*/

    /* Clear the interrupts to be on the safer side*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_ERROR);

    /* Enable the error interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_1_ERROR);

    /* Enable the Receive interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_1_RECEIVE);

    /* Return the driver instance value*/
    return (SYS_MODULE_OBJ)DRV_USART_INDEX_0;
}

void  DRV_USART0_Deinitialize(void)
{
    bool status;

    /* Disable the interrupts */
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_1_TRANSMIT) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_1_RECEIVE) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_1_ERROR);
    /* Ignore the warning */
    (void)status;

    /* Disable USART module */
    PLIB_USART_Disable (USART_ID_1);

}


SYS_STATUS DRV_USART0_Status(void)
{
    /* Return the status as ready always */
    return SYS_STATUS_READY;
}


void DRV_USART0_TasksTransmit(void)
{
    /* This is the USART Driver Transmit tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and performs respective action*/

    /* Reading the transmit interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_TRANSMIT))
    {
        /* Disable the interrupt, to avoid calling ISR continuously*/
        SYS_INT_SourceDisable(INT_SOURCE_USART_1_TRANSMIT);

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
    }
}
void DRV_USART0_TransmitInterruptEnable()
{
    SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
}
void DRV_USART0_TransmitInterruptDisable()
{
    SYS_INT_SourceDisable(INT_SOURCE_USART_1_TRANSMIT);
}
void DRV_USART3_TransmitInterruptEnable()
{
    SYS_INT_SourceEnable(INT_SOURCE_USART_5_TRANSMIT);
}

void DRV_USART0_TasksReceive(void)
{
    /* This is the USART Driver Receive tasks routine. If the receive
       interrupt flag is set, the tasks routines are executed.
     */

    /* Reading the receive interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_RECEIVE))
    {

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
    }
}


void DRV_USART0_TasksError(void)
{
    /* This is the USART Driver Error tasks routine. In this function, the
     * driver checks if an error interrupt has occurred. If so the error
     * condition is cleared.  */

    /* Reading the error interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_ERROR))
    {
        /* This means an error has occurred */
        if(PLIB_USART_ReceiverOverrunHasOccurred(USART_ID_1))
        {
            PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
        }

        /* Clear up the error interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_ERROR);
    }
}

DRV_HANDLE DRV_USART0_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent)
{

    /* Return the driver instance value*/
    return ((DRV_HANDLE)DRV_USART_INDEX_0 );
}

void DRV_USART0_Close(void)
{
    return;
}

DRV_USART_CLIENT_STATUS DRV_USART0_ClientStatus(void)
{
    /* Return the status as ready always*/
    return DRV_USART_CLIENT_STATUS_READY;
}

DRV_USART_TRANSFER_STATUS DRV_USART0_TransferStatus( void )
{
    DRV_USART_TRANSFER_STATUS result = 0;

    /* Check if RX data available */
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_DATA_PRESENT;
    }
    else
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_EMPTY;
    }

    /* Check if TX Buffer is empty */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_1))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY;
    }

    /* Check if the TX buffer is full */
    if(PLIB_USART_TransmitterBufferIsFull(USART_ID_1))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL;
    }

    return(result);
}

DRV_USART_ERROR DRV_USART0_ErrorGet(void)
{
    DRV_USART_ERROR error;
    error = gDrvUSART0Obj.error;

    /* Clear the error before returning */
    gDrvUSART0Obj.error = DRV_USART_ERROR_NONE;

    /* Return the error*/
    return(error);
}


void _DRV_USART0_ErrorConditionClear()
{
    uint8_t dummyData = 0u;
    /* RX length = (FIFO level + RX register) */
    uint8_t RXlength = _DRV_USART_RX_DEPTH;
        
    /* If it's a overrun error then clear it to flush FIFO */
    if(USART_ERROR_RECEIVER_OVERRUN & PLIB_USART_ErrorsGet(USART_ID_1))
    {
        PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
    }
    
    /* Read existing error bytes from FIFO to clear parity and framing error flags*/
    while( (USART_ERROR_PARITY | USART_ERROR_FRAMING) & PLIB_USART_ErrorsGet(USART_ID_1) )
    {
        dummyData = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        RXlength--;
        
        /* Try to flush error bytes for one full FIFO and exit instead of 
         * blocking here if more error bytes are received*/
        if(0u == RXlength)
        {
            break;
        }
    }
    
    /* Ignore the warning */
    (void)dummyData;
    
    /* Clear error interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_ERROR);
            
    /* Clear up the receive interrupt flag so that RX interrupt is not 
     * triggered for error bytes*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
}



DRV_USART_BAUD_SET_RESULT DRV_USART0_BaudSet(uint32_t baud)
{
    uint32_t clockSource;
    int32_t brgValueLow=0;
    int32_t brgValueHigh=0;
    DRV_USART_BAUD_SET_RESULT retVal = DRV_USART_BAUD_SET_SUCCESS;
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Calculate low and high baud values */
    brgValueLow  = ( (clockSource/baud) >> 4 ) - 1;
    brgValueHigh = ( (clockSource/baud) >> 2 ) - 1;

#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_1);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_1);
            while (PLIB_USART_ModuleIsBusy (USART_ID_1));
        }
#endif

    /* Check if the baud value can be set with high baud settings */
    if ((brgValueHigh >= 0) && (brgValueHigh <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighEnable(USART_ID_1);
        PLIB_USART_BaudRateHighSet(USART_ID_1,clockSource,baud);
    }
    
    /* Check if the baud value can be set with low baud settings */
    else if ((brgValueLow >= 0) && (brgValueLow <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighDisable(USART_ID_1);
        PLIB_USART_BaudRateSet(USART_ID_1, clockSource, baud);
    }
    else
    {
            retVal = DRV_USART_BAUD_SET_ERROR;
    }

#if defined (PLIB_USART_ExistsModuleBusyStatus)
    if (isEnabled)
    {
        PLIB_USART_Enable (USART_ID_1);
    }
#endif

    return retVal;
}


DRV_USART_LINE_CONTROL_SET_RESULT DRV_USART0_LineControlSet(DRV_USART_LINE_CONTROL lineControlMode)
{
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_1);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_1);
            while (PLIB_USART_ModuleIsBusy (USART_ID_1));
        }
#endif

    /* Set the Line Control Mode */
    PLIB_USART_LineControlModeSelect(USART_ID_1, lineControlMode);
    
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        if (isEnabled)
        {
            PLIB_USART_Enable (USART_ID_1);
        }
#endif

    /* Return success */
    return(DRV_USART_LINE_CONTROL_SET_SUCCESS);
}


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver static object . */
DRV_USART_OBJ  gDrvUSART1Obj ;

// *****************************************************************************
// *****************************************************************************
// Section: Instance 1 static driver functions
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_USART1_Initialize(void)
{
    uint32_t clockSource;

    /* Disable the USART module to configure it*/
    PLIB_USART_Disable (USART_ID_6);

    /* Initialize the USART based on configuration settings */
    PLIB_USART_InitializeModeGeneral(USART_ID_6,
            false,  /*Auto baud*/
            false,  /*LoopBack mode*/
            false,  /*Auto wakeup on start*/
            false,  /*IRDA mode*/
            false);  /*Stop In Idle mode*/

    /* Set the line control mode */
    PLIB_USART_LineControlModeSelect(USART_ID_6, DRV_USART_LINE_CONTROL_8NONE1);

    /* We set the receive interrupt mode to receive an interrupt whenever FIFO
       is not empty */
    PLIB_USART_InitializeOperation(USART_ID_6,
            USART_RECEIVE_FIFO_ONE_CHAR,
            USART_TRANSMIT_FIFO_IDLE,
            USART_ENABLE_TX_RX_USED);

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Set the baud rate and enable the USART */
    PLIB_USART_BaudSetAndEnable(USART_ID_6,
            clockSource,
            115200);  /*Desired Baud rate value*/

    /* Clear the interrupts to be on the safer side*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_TRANSMIT);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_RECEIVE);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_ERROR);

    /* Enable the error interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_6_ERROR);

    /* Enable the Receive interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_6_RECEIVE);

    /* Return the driver instance value*/
    return (SYS_MODULE_OBJ)DRV_USART_INDEX_1;
}

void  DRV_USART1_Deinitialize(void)
{
    bool status;

    /* Disable the interrupts */
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_6_TRANSMIT) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_6_RECEIVE) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_6_ERROR);
    /* Ignore the warning */
    (void)status;

    /* Disable USART module */
    PLIB_USART_Disable (USART_ID_6);

}


SYS_STATUS DRV_USART1_Status(void)
{
    /* Return the status as ready always */
    return SYS_STATUS_READY;
}


void DRV_USART1_TasksTransmit(void)
{
    /* This is the USART Driver Transmit tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and performs respective action*/

    /* Reading the transmit interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_6_TRANSMIT))
    {
        /* Disable the interrupt, to avoid calling ISR continuously*/
        SYS_INT_SourceDisable(INT_SOURCE_USART_6_TRANSMIT);

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_TRANSMIT);
    }
}

void DRV_USART1_TasksReceive(void)
{
    /* This is the USART Driver Receive tasks routine. If the receive
       interrupt flag is set, the tasks routines are executed.
     */

    /* Reading the receive interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_6_RECEIVE))
    {

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_RECEIVE);
    }
}


void DRV_USART1_TasksError(void)
{
    /* This is the USART Driver Error tasks routine. In this function, the
     * driver checks if an error interrupt has occurred. If so the error
     * condition is cleared.  */

    /* Reading the error interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_6_ERROR))
    {
        /* This means an error has occurred */
        if(PLIB_USART_ReceiverOverrunHasOccurred(USART_ID_6))
        {
            PLIB_USART_ReceiverOverrunErrorClear(USART_ID_6);
        }

        /* Clear up the error interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_ERROR);
    }
}

DRV_HANDLE DRV_USART1_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent)
{

    /* Return the driver instance value*/
    return ((DRV_HANDLE)DRV_USART_INDEX_1 );
}

void DRV_USART1_Close(void)
{
    return;
}

DRV_USART_CLIENT_STATUS DRV_USART1_ClientStatus(void)
{
    /* Return the status as ready always*/
    return DRV_USART_CLIENT_STATUS_READY;
}

DRV_USART_TRANSFER_STATUS DRV_USART1_TransferStatus( void )
{
    DRV_USART_TRANSFER_STATUS result = 0;

    /* Check if RX data available */
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_6))
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_DATA_PRESENT;
    }
    else
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_EMPTY;
    }

    /* Check if TX Buffer is empty */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_6))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY;
    }

    /* Check if the TX buffer is full */
    if(PLIB_USART_TransmitterBufferIsFull(USART_ID_6))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL;
    }

    return(result);
}

DRV_USART_ERROR DRV_USART1_ErrorGet(void)
{
    DRV_USART_ERROR error;
    error = gDrvUSART1Obj.error;

    /* Clear the error before returning */
    gDrvUSART1Obj.error = DRV_USART_ERROR_NONE;

    /* Return the error*/
    return(error);
}


void _DRV_USART1_ErrorConditionClear()
{
    uint8_t dummyData = 0u;
    /* RX length = (FIFO level + RX register) */
    uint8_t RXlength = _DRV_USART_RX_DEPTH;
        
    /* If it's a overrun error then clear it to flush FIFO */
    if(USART_ERROR_RECEIVER_OVERRUN & PLIB_USART_ErrorsGet(USART_ID_6))
    {
        PLIB_USART_ReceiverOverrunErrorClear(USART_ID_6);
    }
    
    /* Read existing error bytes from FIFO to clear parity and framing error flags*/
    while( (USART_ERROR_PARITY | USART_ERROR_FRAMING) & PLIB_USART_ErrorsGet(USART_ID_6) )
    {
        dummyData = PLIB_USART_ReceiverByteReceive(USART_ID_6);
        RXlength--;
        
        /* Try to flush error bytes for one full FIFO and exit instead of 
         * blocking here if more error bytes are received*/
        if(0u == RXlength)
        {
            break;
        }
    }
    
    /* Ignore the warning */
    (void)dummyData;
    
    /* Clear error interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_ERROR);
            
    /* Clear up the receive interrupt flag so that RX interrupt is not 
     * triggered for error bytes*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_RECEIVE);
}



DRV_USART_BAUD_SET_RESULT DRV_USART1_BaudSet(uint32_t baud)
{
    uint32_t clockSource;
    int32_t brgValueLow=0;
    int32_t brgValueHigh=0;
    DRV_USART_BAUD_SET_RESULT retVal = DRV_USART_BAUD_SET_SUCCESS;
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Calculate low and high baud values */
    brgValueLow  = ( (clockSource/baud) >> 4 ) - 1;
    brgValueHigh = ( (clockSource/baud) >> 2 ) - 1;

#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_6);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_6);
            while (PLIB_USART_ModuleIsBusy (USART_ID_6));
        }
#endif

    /* Check if the baud value can be set with high baud settings */
    if ((brgValueHigh >= 0) && (brgValueHigh <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighEnable(USART_ID_6);
        PLIB_USART_BaudRateHighSet(USART_ID_6,clockSource,baud);
    }
    
    /* Check if the baud value can be set with low baud settings */
    else if ((brgValueLow >= 0) && (brgValueLow <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighDisable(USART_ID_6);
        PLIB_USART_BaudRateSet(USART_ID_6, clockSource, baud);
    }
    else
    {
            retVal = DRV_USART_BAUD_SET_ERROR;
    }

#if defined (PLIB_USART_ExistsModuleBusyStatus)
    if (isEnabled)
    {
        PLIB_USART_Enable (USART_ID_6);
    }
#endif

    return retVal;
}


DRV_USART_LINE_CONTROL_SET_RESULT DRV_USART1_LineControlSet(DRV_USART_LINE_CONTROL lineControlMode)
{
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_6);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_6);
            while (PLIB_USART_ModuleIsBusy (USART_ID_6));
        }
#endif

    /* Set the Line Control Mode */
    PLIB_USART_LineControlModeSelect(USART_ID_6, lineControlMode);
    
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        if (isEnabled)
        {
            PLIB_USART_Enable (USART_ID_6);
        }
#endif

    /* Return success */
    return(DRV_USART_LINE_CONTROL_SET_SUCCESS);
}


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver static object . */
DRV_USART_OBJ  gDrvUSART2Obj ;

// *****************************************************************************
// *****************************************************************************
// Section: Instance 2 static driver functions
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_USART2_Initialize(void)
{
    uint32_t clockSource;

    /* Disable the USART module to configure it*/
    PLIB_USART_Disable (USART_ID_4);

    /* Initialize the USART based on configuration settings */
    PLIB_USART_InitializeModeGeneral(USART_ID_4,
            false,  /*Auto baud*/
            false,  /*LoopBack mode*/
            false,  /*Auto wakeup on start*/
            false,  /*IRDA mode*/
            false);  /*Stop In Idle mode*/

    /* Set the line control mode */
    PLIB_USART_LineControlModeSelect(USART_ID_4, DRV_USART_LINE_CONTROL_8NONE1);

    /* We set the receive interrupt mode to receive an interrupt whenever FIFO
       is not empty */
    PLIB_USART_InitializeOperation(USART_ID_4,
            USART_RECEIVE_FIFO_ONE_CHAR,
            USART_TRANSMIT_FIFO_IDLE,
            USART_ENABLE_TX_RX_USED);

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Set the baud rate and enable the USART */
    PLIB_USART_BaudSetAndEnable(USART_ID_4,
            clockSource,
            115200);  /*Desired Baud rate value*/

    /* Clear the interrupts to be on the safer side*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_TRANSMIT);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_RECEIVE);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_ERROR);

    /* Enable the error interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_4_ERROR);

    /* Enable the Receive interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_4_RECEIVE);

    /* Return the driver instance value*/
    return (SYS_MODULE_OBJ)DRV_USART_INDEX_2;
}

void  DRV_USART2_Deinitialize(void)
{
    bool status;

    /* Disable the interrupts */
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_4_TRANSMIT) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_4_RECEIVE) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_4_ERROR);
    /* Ignore the warning */
    (void)status;

    /* Disable USART module */
    PLIB_USART_Disable (USART_ID_4);

}


SYS_STATUS DRV_USART2_Status(void)
{
    /* Return the status as ready always */
    return SYS_STATUS_READY;
}


void DRV_USART2_TasksTransmit(void)
{
    /* This is the USART Driver Transmit tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and performs respective action*/

    /* Reading the transmit interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_4_TRANSMIT))
    {
        /* Disable the interrupt, to avoid calling ISR continuously*/
        SYS_INT_SourceDisable(INT_SOURCE_USART_4_TRANSMIT);

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_TRANSMIT);
    }
}
void DRV_USART2_TransmitInterruptEnable()
{
    SYS_INT_SourceEnable(INT_SOURCE_USART_4_TRANSMIT);
}
void DRV_USART2_TransmitInterruptDisable()
{
    SYS_INT_SourceDisable(INT_SOURCE_USART_4_TRANSMIT);
}
void DRV_USART2_TasksReceive(void)
{
    /* This is the USART Driver Receive tasks routine. If the receive
       interrupt flag is set, the tasks routines are executed.
     */

    /* Reading the receive interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_4_RECEIVE))
    {

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_RECEIVE);
    }
}


void DRV_USART2_TasksError(void)
{
    /* This is the USART Driver Error tasks routine. In this function, the
     * driver checks if an error interrupt has occurred. If so the error
     * condition is cleared.  */

    /* Reading the error interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_4_ERROR))
    {
        /* This means an error has occurred */
        if(PLIB_USART_ReceiverOverrunHasOccurred(USART_ID_4))
        {
            PLIB_USART_ReceiverOverrunErrorClear(USART_ID_4);
        }

        /* Clear up the error interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_ERROR);
    }
}

DRV_HANDLE DRV_USART2_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent)
{

    /* Return the driver instance value*/
    return ((DRV_HANDLE)DRV_USART_INDEX_2 );
}

void DRV_USART2_Close(void)
{
    return;
}

DRV_USART_CLIENT_STATUS DRV_USART2_ClientStatus(void)
{
    /* Return the status as ready always*/
    return DRV_USART_CLIENT_STATUS_READY;
}

DRV_USART_TRANSFER_STATUS DRV_USART2_TransferStatus( void )
{
    DRV_USART_TRANSFER_STATUS result = 0;

    /* Check if RX data available */
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_4))
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_DATA_PRESENT;
    }
    else
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_EMPTY;
    }

    /* Check if TX Buffer is empty */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_4))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY;
    }

    /* Check if the TX buffer is full */
    if(PLIB_USART_TransmitterBufferIsFull(USART_ID_4))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL;
    }

    return(result);
}

DRV_USART_ERROR DRV_USART2_ErrorGet(void)
{
    DRV_USART_ERROR error;
    error = gDrvUSART2Obj.error;

    /* Clear the error before returning */
    gDrvUSART2Obj.error = DRV_USART_ERROR_NONE;

    /* Return the error*/
    return(error);
}


void _DRV_USART2_ErrorConditionClear()
{
    uint8_t dummyData = 0u;
    /* RX length = (FIFO level + RX register) */
    uint8_t RXlength = _DRV_USART_RX_DEPTH;
        
    /* If it's a overrun error then clear it to flush FIFO */
    if(USART_ERROR_RECEIVER_OVERRUN & PLIB_USART_ErrorsGet(USART_ID_4))
    {
        PLIB_USART_ReceiverOverrunErrorClear(USART_ID_4);
    }
    
    /* Read existing error bytes from FIFO to clear parity and framing error flags*/
    while( (USART_ERROR_PARITY | USART_ERROR_FRAMING) & PLIB_USART_ErrorsGet(USART_ID_4) )
    {
        dummyData = PLIB_USART_ReceiverByteReceive(USART_ID_4);
        RXlength--;
        
        /* Try to flush error bytes for one full FIFO and exit instead of 
         * blocking here if more error bytes are received*/
        if(0u == RXlength)
        {
            break;
        }
    }
    
    /* Ignore the warning */
    (void)dummyData;
    
    /* Clear error interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_ERROR);
            
    /* Clear up the receive interrupt flag so that RX interrupt is not 
     * triggered for error bytes*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_RECEIVE);
}



DRV_USART_BAUD_SET_RESULT DRV_USART2_BaudSet(uint32_t baud)
{
    uint32_t clockSource;
    int32_t brgValueLow=0;
    int32_t brgValueHigh=0;
    DRV_USART_BAUD_SET_RESULT retVal = DRV_USART_BAUD_SET_SUCCESS;
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Calculate low and high baud values */
    brgValueLow  = ( (clockSource/baud) >> 4 ) - 1;
    brgValueHigh = ( (clockSource/baud) >> 2 ) - 1;

#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_4);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_4);
            while (PLIB_USART_ModuleIsBusy (USART_ID_4));
        }
#endif

    /* Check if the baud value can be set with high baud settings */
    if ((brgValueHigh >= 0) && (brgValueHigh <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighEnable(USART_ID_4);
        PLIB_USART_BaudRateHighSet(USART_ID_4,clockSource,baud);
    }
    
    /* Check if the baud value can be set with low baud settings */
    else if ((brgValueLow >= 0) && (brgValueLow <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighDisable(USART_ID_4);
        PLIB_USART_BaudRateSet(USART_ID_4, clockSource, baud);
    }
    else
    {
            retVal = DRV_USART_BAUD_SET_ERROR;
    }

#if defined (PLIB_USART_ExistsModuleBusyStatus)
    if (isEnabled)
    {
        PLIB_USART_Enable (USART_ID_4);
    }
#endif

    return retVal;
}


DRV_USART_LINE_CONTROL_SET_RESULT DRV_USART2_LineControlSet(DRV_USART_LINE_CONTROL lineControlMode)
{
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_4);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_4);
            while (PLIB_USART_ModuleIsBusy (USART_ID_4));
        }
#endif

    /* Set the Line Control Mode */
    PLIB_USART_LineControlModeSelect(USART_ID_4, lineControlMode);
    
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        if (isEnabled)
        {
            PLIB_USART_Enable (USART_ID_4);
        }
#endif

    /* Return success */
    return(DRV_USART_LINE_CONTROL_SET_SUCCESS);
}


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver static object . */
DRV_USART_OBJ  gDrvUSART3Obj ;

// *****************************************************************************
// *****************************************************************************
// Section: Instance 3 static driver functions
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_USART3_Initialize(void)
{
    uint32_t clockSource;

    /* Disable the USART module to configure it*/
    PLIB_USART_Disable (USART_ID_5);

    /* Initialize the USART based on configuration settings */
    PLIB_USART_InitializeModeGeneral(USART_ID_5,
            false,  /*Auto baud*/
            false,  /*LoopBack mode*/
            false,  /*Auto wakeup on start*/
            false,  /*IRDA mode*/
            false);  /*Stop In Idle mode*/

    /* Set the line control mode */
    PLIB_USART_LineControlModeSelect(USART_ID_5, DRV_USART_LINE_CONTROL_8NONE1);

    /* We set the receive interrupt mode to receive an interrupt whenever FIFO
       is not empty */
    PLIB_USART_InitializeOperation(USART_ID_5,
            USART_RECEIVE_FIFO_ONE_CHAR,
            USART_TRANSMIT_FIFO_IDLE,
            USART_ENABLE_TX_RX_USED);

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Set the baud rate and enable the USART */
    PLIB_USART_BaudSetAndEnable(USART_ID_5,
            clockSource,
            230400);  /*Desired Baud rate value*/

    /* Clear the interrupts to be on the safer side*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_TRANSMIT);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_RECEIVE);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_ERROR);

    /* Enable the error interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_5_ERROR);

    /* Enable the Receive interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_5_RECEIVE);

    /* Return the driver instance value*/
    return (SYS_MODULE_OBJ)DRV_USART_INDEX_3;
}

void  DRV_USART3_Deinitialize(void)
{
    bool status;

    /* Disable the interrupts */
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_5_TRANSMIT) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_5_RECEIVE) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_5_ERROR);
    /* Ignore the warning */
    (void)status;

    /* Disable USART module */
    PLIB_USART_Disable (USART_ID_5);

}


SYS_STATUS DRV_USART3_Status(void)
{
    /* Return the status as ready always */
    return SYS_STATUS_READY;
}


void DRV_USART3_TasksTransmit(void)
{
    /* This is the USART Driver Transmit tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and performs respective action*/

    /* Reading the transmit interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_5_TRANSMIT))
    {
        /* Disable the interrupt, to avoid calling ISR continuously*/
        SYS_INT_SourceDisable(INT_SOURCE_USART_5_TRANSMIT);

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_TRANSMIT);
    }
}

void DRV_USART3_TasksReceive(void)
{
    /* This is the USART Driver Receive tasks routine. If the receive
       interrupt flag is set, the tasks routines are executed.
     */

    /* Reading the receive interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_5_RECEIVE))
    {

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_RECEIVE);
    }
}


void DRV_USART3_TasksError(void)
{
    /* This is the USART Driver Error tasks routine. In this function, the
     * driver checks if an error interrupt has occurred. If so the error
     * condition is cleared.  */

    /* Reading the error interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_5_ERROR))
    {
        /* This means an error has occurred */
        if(PLIB_USART_ReceiverOverrunHasOccurred(USART_ID_5))
        {
            PLIB_USART_ReceiverOverrunErrorClear(USART_ID_5);
        }

        /* Clear up the error interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_ERROR);
    }
}

DRV_HANDLE DRV_USART3_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent)
{

    /* Return the driver instance value*/
    return ((DRV_HANDLE)DRV_USART_INDEX_3 );
}

void DRV_USART3_Close(void)
{
    return;
}

DRV_USART_CLIENT_STATUS DRV_USART3_ClientStatus(void)
{
    /* Return the status as ready always*/
    return DRV_USART_CLIENT_STATUS_READY;
}

DRV_USART_TRANSFER_STATUS DRV_USART3_TransferStatus( void )
{
    DRV_USART_TRANSFER_STATUS result = 0;

    /* Check if RX data available */
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_5))
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_DATA_PRESENT;
    }
    else
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_EMPTY;
    }

    /* Check if TX Buffer is empty */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_5))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY;
    }

    /* Check if the TX buffer is full */
    if(PLIB_USART_TransmitterBufferIsFull(USART_ID_5))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL;
    }

    return(result);
}

DRV_USART_ERROR DRV_USART3_ErrorGet(void)
{
    DRV_USART_ERROR error;
    error = gDrvUSART3Obj.error;

    /* Clear the error before returning */
    gDrvUSART3Obj.error = DRV_USART_ERROR_NONE;

    /* Return the error*/
    return(error);
}


void _DRV_USART3_ErrorConditionClear()
{
    uint8_t dummyData = 0u;
    /* RX length = (FIFO level + RX register) */
    uint8_t RXlength = _DRV_USART_RX_DEPTH;
        
    /* If it's a overrun error then clear it to flush FIFO */
    if(USART_ERROR_RECEIVER_OVERRUN & PLIB_USART_ErrorsGet(USART_ID_5))
    {
        PLIB_USART_ReceiverOverrunErrorClear(USART_ID_5);
    }
    
    /* Read existing error bytes from FIFO to clear parity and framing error flags*/
    while( (USART_ERROR_PARITY | USART_ERROR_FRAMING) & PLIB_USART_ErrorsGet(USART_ID_5) )
    {
        dummyData = PLIB_USART_ReceiverByteReceive(USART_ID_5);
        RXlength--;
        
        /* Try to flush error bytes for one full FIFO and exit instead of 
         * blocking here if more error bytes are received*/
        if(0u == RXlength)
        {
            break;
        }
    }
    
    /* Ignore the warning */
    (void)dummyData;
    
    /* Clear error interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_ERROR);
            
    /* Clear up the receive interrupt flag so that RX interrupt is not 
     * triggered for error bytes*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_RECEIVE);
}



DRV_USART_BAUD_SET_RESULT DRV_USART3_BaudSet(uint32_t baud)
{
    uint32_t clockSource;
    int32_t brgValueLow=0;
    int32_t brgValueHigh=0;
    DRV_USART_BAUD_SET_RESULT retVal = DRV_USART_BAUD_SET_SUCCESS;
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Calculate low and high baud values */
    brgValueLow  = ( (clockSource/baud) >> 4 ) - 1;
    brgValueHigh = ( (clockSource/baud) >> 2 ) - 1;

#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_5);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_5);
            while (PLIB_USART_ModuleIsBusy (USART_ID_5));
        }
#endif

    /* Check if the baud value can be set with high baud settings */
    if ((brgValueHigh >= 0) && (brgValueHigh <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighEnable(USART_ID_5);
        PLIB_USART_BaudRateHighSet(USART_ID_5,clockSource,baud);
    }
    
    /* Check if the baud value can be set with low baud settings */
    else if ((brgValueLow >= 0) && (brgValueLow <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighDisable(USART_ID_5);
        PLIB_USART_BaudRateSet(USART_ID_5, clockSource, baud);
    }
    else
    {
            retVal = DRV_USART_BAUD_SET_ERROR;
    }

#if defined (PLIB_USART_ExistsModuleBusyStatus)
    if (isEnabled)
    {
        PLIB_USART_Enable (USART_ID_5);
    }
#endif

    return retVal;
}


DRV_USART_LINE_CONTROL_SET_RESULT DRV_USART3_LineControlSet(DRV_USART_LINE_CONTROL lineControlMode)
{
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_5);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_5);
            while (PLIB_USART_ModuleIsBusy (USART_ID_5));
        }
#endif

    /* Set the Line Control Mode */
    PLIB_USART_LineControlModeSelect(USART_ID_5, lineControlMode);
    
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        if (isEnabled)
        {
            PLIB_USART_Enable (USART_ID_5);
        }
#endif

    /* Return success */
    return(DRV_USART_LINE_CONTROL_SET_SUCCESS);
}


// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver static object . */
DRV_USART_OBJ  gDrvUSART4Obj ;

// *****************************************************************************
// *****************************************************************************
// Section: Instance 4 static driver functions
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_USART4_Initialize(void)
{
    uint32_t clockSource;

    /* Disable the USART module to configure it*/
    PLIB_USART_Disable (USART_ID_3);

    /* Initialize the USART based on configuration settings */
    PLIB_USART_InitializeModeGeneral(USART_ID_3,
            false,  /*Auto baud*/
            false,  /*LoopBack mode*/
            false,  /*Auto wakeup on start*/
            false,  /*IRDA mode*/
            false);  /*Stop In Idle mode*/

    /* Set the line control mode */
    PLIB_USART_LineControlModeSelect(USART_ID_3, DRV_USART_LINE_CONTROL_8NONE1);

    /* We set the receive interrupt mode to receive an interrupt whenever FIFO
       is not empty */
    PLIB_USART_InitializeOperation(USART_ID_3,
            USART_RECEIVE_FIFO_ONE_CHAR,
            USART_TRANSMIT_FIFO_IDLE,
            USART_ENABLE_TX_RX_USED);

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Set the baud rate and enable the USART */
    PLIB_USART_BaudSetAndEnable(USART_ID_3,
            clockSource,
            115200);  /*Desired Baud rate value*/

    /* Clear the interrupts to be on the safer side*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_TRANSMIT);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_RECEIVE);
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_ERROR);

    /* Enable the error interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_3_ERROR);

    /* Enable the Receive interrupt source */
    SYS_INT_SourceEnable(INT_SOURCE_USART_3_RECEIVE);

    /* Return the driver instance value*/
    return (SYS_MODULE_OBJ)DRV_USART_INDEX_4;
}

void  DRV_USART4_Deinitialize(void)
{
    bool status;

    /* Disable the interrupts */
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_3_TRANSMIT) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_3_RECEIVE) ;
    status = SYS_INT_SourceDisable(INT_SOURCE_USART_3_ERROR);
    /* Ignore the warning */
    (void)status;

    /* Disable USART module */
    PLIB_USART_Disable (USART_ID_3);

}


SYS_STATUS DRV_USART4_Status(void)
{
    /* Return the status as ready always */
    return SYS_STATUS_READY;
}


void DRV_USART4_TasksTransmit(void)
{
    /* This is the USART Driver Transmit tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and performs respective action*/

    /* Reading the transmit interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_3_TRANSMIT))
    {
        /* Disable the interrupt, to avoid calling ISR continuously*/
        SYS_INT_SourceDisable(INT_SOURCE_USART_3_TRANSMIT);

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_TRANSMIT);
    }
}

void DRV_USART4_TasksReceive(void)
{
    /* This is the USART Driver Receive tasks routine. If the receive
       interrupt flag is set, the tasks routines are executed.
     */

    /* Reading the receive interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_3_RECEIVE))
    {

        /* Clear up the interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_RECEIVE);
    }
}


void DRV_USART4_TasksError(void)
{
    /* This is the USART Driver Error tasks routine. In this function, the
     * driver checks if an error interrupt has occurred. If so the error
     * condition is cleared.  */

    /* Reading the error interrupt flag */
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_3_ERROR))
    {
        /* This means an error has occurred */
        if(PLIB_USART_ReceiverOverrunHasOccurred(USART_ID_3))
        {
            PLIB_USART_ReceiverOverrunErrorClear(USART_ID_3);
        }

        /* Clear up the error interrupt flag */
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_ERROR);
    }
}

DRV_HANDLE DRV_USART4_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent)
{

    /* Return the driver instance value*/
    return ((DRV_HANDLE)DRV_USART_INDEX_4 );
}

void DRV_USART4_Close(void)
{
    return;
}

DRV_USART_CLIENT_STATUS DRV_USART4_ClientStatus(void)
{
    /* Return the status as ready always*/
    return DRV_USART_CLIENT_STATUS_READY;
}

DRV_USART_TRANSFER_STATUS DRV_USART4_TransferStatus( void )
{
    DRV_USART_TRANSFER_STATUS result = 0;

    /* Check if RX data available */
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_3))
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_DATA_PRESENT;
    }
    else
    {
        result|= DRV_USART_TRANSFER_STATUS_RECEIVER_EMPTY;
    }

    /* Check if TX Buffer is empty */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_3))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_EMPTY;
    }

    /* Check if the TX buffer is full */
    if(PLIB_USART_TransmitterBufferIsFull(USART_ID_3))
    {
        result|= DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL;
    }

    return(result);
}

DRV_USART_ERROR DRV_USART4_ErrorGet(void)
{
    DRV_USART_ERROR error;
    error = gDrvUSART4Obj.error;

    /* Clear the error before returning */
    gDrvUSART4Obj.error = DRV_USART_ERROR_NONE;

    /* Return the error*/
    return(error);
}


void _DRV_USART4_ErrorConditionClear()
{
    uint8_t dummyData = 0u;
    /* RX length = (FIFO level + RX register) */
    uint8_t RXlength = _DRV_USART_RX_DEPTH;
        
    /* If it's a overrun error then clear it to flush FIFO */
    if(USART_ERROR_RECEIVER_OVERRUN & PLIB_USART_ErrorsGet(USART_ID_3))
    {
        PLIB_USART_ReceiverOverrunErrorClear(USART_ID_3);
    }
    
    /* Read existing error bytes from FIFO to clear parity and framing error flags*/
    while( (USART_ERROR_PARITY | USART_ERROR_FRAMING) & PLIB_USART_ErrorsGet(USART_ID_3) )
    {
        dummyData = PLIB_USART_ReceiverByteReceive(USART_ID_3);
        RXlength--;
        
        /* Try to flush error bytes for one full FIFO and exit instead of 
         * blocking here if more error bytes are received*/
        if(0u == RXlength)
        {
            break;
        }
    }
    
    /* Ignore the warning */
    (void)dummyData;
    
    /* Clear error interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_ERROR);
            
    /* Clear up the receive interrupt flag so that RX interrupt is not 
     * triggered for error bytes*/
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_RECEIVE);
}



DRV_USART_BAUD_SET_RESULT DRV_USART4_BaudSet(uint32_t baud)
{
    uint32_t clockSource;
    int32_t brgValueLow=0;
    int32_t brgValueHigh=0;
    DRV_USART_BAUD_SET_RESULT retVal = DRV_USART_BAUD_SET_SUCCESS;
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif

    /* Get the USART clock source value*/
    clockSource = SYS_CLK_PeripheralFrequencyGet ( CLK_BUS_PERIPHERAL_2 );

    /* Calculate low and high baud values */
    brgValueLow  = ( (clockSource/baud) >> 4 ) - 1;
    brgValueHigh = ( (clockSource/baud) >> 2 ) - 1;

#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_3);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_3);
            while (PLIB_USART_ModuleIsBusy (USART_ID_3));
        }
#endif

    /* Check if the baud value can be set with high baud settings */
    if ((brgValueHigh >= 0) && (brgValueHigh <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighEnable(USART_ID_3);
        PLIB_USART_BaudRateHighSet(USART_ID_3,clockSource,baud);
    }
    
    /* Check if the baud value can be set with low baud settings */
    else if ((brgValueLow >= 0) && (brgValueLow <= UINT16_MAX))
    {
        PLIB_USART_BaudRateHighDisable(USART_ID_3);
        PLIB_USART_BaudRateSet(USART_ID_3, clockSource, baud);
    }
    else
    {
            retVal = DRV_USART_BAUD_SET_ERROR;
    }

#if defined (PLIB_USART_ExistsModuleBusyStatus)
    if (isEnabled)
    {
        PLIB_USART_Enable (USART_ID_3);
    }
#endif

    return retVal;
}


DRV_USART_LINE_CONTROL_SET_RESULT DRV_USART4_LineControlSet(DRV_USART_LINE_CONTROL lineControlMode)
{
#if defined (PLIB_USART_ExistsModuleBusyStatus)
    bool isEnabled = false;
#endif
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        isEnabled = PLIB_USART_ModuleIsBusy (USART_ID_3);
        if (isEnabled)
        {
            PLIB_USART_Disable (USART_ID_3);
            while (PLIB_USART_ModuleIsBusy (USART_ID_3));
        }
#endif

    /* Set the Line Control Mode */
    PLIB_USART_LineControlModeSelect(USART_ID_3, lineControlMode);
    
#if defined (PLIB_USART_ExistsModuleBusyStatus)
        if (isEnabled)
        {
            PLIB_USART_Enable (USART_ID_3);
        }
#endif

    /* Return success */
    return(DRV_USART_LINE_CONTROL_SET_SUCCESS);
}

/*******************************************************************************
 End of File
*/
