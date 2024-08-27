/*
 * Lora.c
 *
 *      Author: Meriem
 */


#include "Lora.h"

extern SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef* Lora_SpiHandler= &hspi1;

extern Lora_RxStates_ten Lora_RxCurrentState_en ;
extern Lora_TxStates_ten Lora_TxCurrentState_en ;

uint8_t Lora_status_u8 = 0 ;
uint8_t Lora_TxDoneHw_u8 = 0 ;
uint8_t Lora_TxDoneSw_u8 = 0 ;
uint8_t Lora_RxDoneHw_u8 = 0 ;
uint8_t Lora_RxDoneSw_u8 = 0 ;
volatile uint8_t Lora_Receive_Complete = 0;
volatile uint8_t Lora_Transmit_Complete = 0;
//static uint32_t Lora_Rxcounter_u32 = 0 ;

/**
  * @brief	Set the slave select pin to high state or low state
  * @note	Necessary for the SPI communication with the Sx1278
  * @param	value takes 0 Low state or 1 for High state
  * @retval None
  */
void Lora_vSetNSS(int value) {
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,
			(value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief	Initialization of the Lora Modem
  * @note	The LoRa Modem doesn't work without it
  * @retval	None
  */
void Lora_vInit(void) {
	Lora_vSetNSS(1);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
}

/**
  * @brief	Reset the Lora Modem using the Reset pin
  * @note	uses Hal_Delay so it needs to be modified if used with other timers
  * @retval	None
  */
void Lora_vReset(void) {
	Lora_vSetNSS(1);

	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);

	HAL_Delay(5);
}


void Lora_WaitForTx()
{
	while( 0 == Lora_Transmit_Complete )
		;
	Lora_Transmit_Complete=0;

}

void Lora_WaitForRx()
{
	while( 0 == Lora_Receive_Complete )
		;
	Lora_Receive_Complete=0;
}



/**
  * @brief	Writes the byte data in a register
  * @note	Uses the spi interface
  * @param	RegAdress is the address of the register in the Lora Modem
  * @param	data is the value to be written in the register
  * @retval	None
  */
void Lora_vWriteReg(uint8_t RegAdress, uint8_t data){
	uint8_t payload[2]={0.0};
	payload[0] = RegAdress | 0x80 ;
	payload[1] = data;

	Lora_vSetNSS( 0 );

	while( HAL_SPI_Transmit_DMA( Lora_SpiHandler, payload, 2) != HAL_OK );
	Lora_WaitForTx();

	Lora_vSetNSS( 1 );

}

/**
  * @brief	Returns the byte written in a register
  * @note	Uses the spi interface
  * @param	RegAdress is the address of the register in the Lora Modem
  * @retval	uint8_t
  */
uint8_t Lora_u8ReadReg(uint8_t RegAdress){
	uint8_t payload[2]={0.0};
	payload[0] = RegAdress;

	Lora_vSetNSS( 0 );

	while( HAL_SPI_Transmit_DMA( Lora_SpiHandler, payload, 1 ) !=HAL_OK );
	Lora_WaitForTx();

	while( HAL_SPI_Receive_DMA( Lora_SpiHandler, payload+1, 1 ) !=HAL_OK );
	Lora_WaitForRx();

	Lora_vSetNSS( 1 );
	return payload[1];

}
/**
  * @brief	Writes the packet to be sent in the Fifo
  * @note	uses the single packet mode instead of Fifo mode of the SPI communication
  * @note 	Fifo mode is the optimal mode in this case
  * @param	TxBuffer is the pointer on the first byte of the packet to be written in the Fifo
  * @param	packet size is the amount of data to be sent
  * @retval None
  */

void Lora_vWriteFifo(uint8_t * TxBuffer, uint8_t payloadLength) {

	if (payloadLength < 1) {
		return;
	} else {
			for(int i =0; i<payloadLength ; i++){
				Lora_vWriteReg(RegFifo,TxBuffer[i]);
			}
	}
}

/**
  * @brief	Reads the packet received in the Fifo
  * @note	uses the Fifo mode of the SPI communication
  * @param	RxBuffer is the pointer on the first byte of the packet received from the Fifo
  * @param	payloadLength is the amount of data received
  * @retval None
  */

void Lora_vReadFifo(uint8_t RegAdr, uint8_t * RxBuffer, uint8_t payloadLength){
		if (payloadLength < 1) {
			return;
		} else {
			Lora_vSetNSS(0);
			HAL_SPI_Transmit(Lora_SpiHandler, &RegAdr, 1,HAL_MAX_DELAY);
			while (HAL_SPI_GetState(Lora_SpiHandler) != HAL_SPI_STATE_READY)
					;
				HAL_SPI_Receive(Lora_SpiHandler, RxBuffer, payloadLength,HAL_MAX_DELAY);
			    while (HAL_SPI_GetState(Lora_SpiHandler) != HAL_SPI_STATE_READY)
			    			;
			Lora_vSetNSS(1);
		}
}
/**
  * @brief	returns the state of the hardware interrupt of the Lora Modem
  * @note	to be configured as an external interrupt when the Lora Modem is used with other peripherals
  * @retval	returns the state of the DIO0 pin
  */
uint8_t Lora_u8GetDIO0(void) {
	return (HAL_GPIO_ReadPin(DIO0_GPIO_Port, DIO0_Pin) == GPIO_PIN_SET);
}
/**
  * @brief	Configuration of the parameters of the Lora Modem
  * @note	ends with the Lora Modem in standby mode
  * @retval None
  */
void Lora_vConfig(void){
	Lora_vWriteReg(RegOpMode,SleepMode);		//Set Sleep Mode
	HAL_Delay(15);

	Lora_vWriteReg(RegOpMode,LoraMode);				//Set LORA mode / High Frequency Mode

	Lora_vWriteReg(RegFrMsb,FrMsb);
	Lora_vWriteReg(RegFrMid,FrMid);
	Lora_vWriteReg(RegFrLsb,FrLsb);

	Lora_vWriteReg(RegSyncWord,SyncWord);

	Lora_vWriteReg(RegPaConfig,DefaultMaxPower); 	//Default max output power config

	Lora_vWriteReg(RegOcp,Ocp);
	Lora_vWriteReg(RegLna,Lna);

	Lora_vWriteReg(RegModemConfig1,Config1);
	Lora_vWriteReg(RegModemConfig2,Config2);
	Lora_vWriteReg(RegModemConfig3,Config3);

	Lora_vWriteReg(RegSymbTimeoutLsb,SymbTimeoutLsb);

	Lora_vWriteReg(RegPreambleLsb,PreambleLsb);
	Lora_vWriteReg(RegPreambleMsb,PreambleMsb);

	Lora_vWriteReg(RegOpMode,StdbyMode);

}
/**
  * @brief	clears the interruption request flags
  * @retval	None
  */
void Lora_clearIrq(void) {
	Lora_vWriteReg(RegIrqFlags,ClearIrqFlags);
}

/**
  * @brief	Configuration of the Rx parameters of the Lora Modem
  * @param	payloadLength is the amount of data to be received
  * @retval	None
  */
void Lora_vRxConfig(uint8_t payloadLength){
	uint8_t adr;

	Lora_vWriteReg(RegPaDac,RxPaDac);
	Lora_vWriteReg(RegHopPeriod,RxHopPeriod);
	Lora_vWriteReg(RegDioMapping1,RxDioMapping1);
	Lora_vWriteReg(RegIrqFlagsMask,RxIrqFlagsMask);
	Lora_clearIrq();
	Lora_vWriteReg(RegPayloadLength, payloadLength);
	Lora_vWriteReg(RegFifoRxBaseAdr,RxBaseAdr);
	adr= Lora_u8ReadReg(RegFifoRxBaseAdr);
	Lora_vWriteReg(RegFifoAdrPtr,adr);
}

/**
  * @brief	Configuration of the Tx parameters of the Lora Modem
  * @param	payloadLength is the amount of data to be sent
  * @retval	None
  */
void Lora_vTxConfig(uint8_t payloadLength){
	uint8_t adr;

	Lora_vWriteReg(RegPaDac,TxPaDac);
	Lora_vWriteReg(RegHopPeriod,TxHopPeriod);
	Lora_vWriteReg(RegDioMapping1,TxDioMapping1);
	Lora_vWriteReg(RegIrqFlagsMask,TxIrqFlagsMask);
	Lora_clearIrq();
	Lora_vWriteReg(RegPayloadLength, payloadLength); //Set the payload length
	Lora_vWriteReg(RegFifoTxBaseAdr,TxBaseAdr);
	adr= Lora_u8ReadReg(RegFifoTxBaseAdr);
	Lora_vWriteReg(RegFifoAdrPtr,adr);
}

/**
  * @brief	state machine describing the functioning of the Lora Modem in Rx mode
  * @param	RxBuffer is a pointer to an array containing the received packet
  * @param	payloadLength is the amount of received data
  * @retval	Lora_RxMode state
  */
Lora_RxStates_ten Lora_tenRxMode(uint8_t* RxBuffer, uint8_t payloadLength){

	uint8_t RegAdr ;
	uint8_t packetSize;

	switch(Lora_RxCurrentState_en){
	case Lora_RxIdle_en:{
		Lora_RxCurrentState_en = Lora_ModemConfigRx;
		}break;
	case Lora_ModemConfigRx:{
		Lora_vConfig();
		Lora_RxCurrentState_en = Lora_RxConfig_en;
	}break;
	case Lora_RxConfig_en :{

		Lora_vRxConfig(payloadLength);
		Lora_RxCurrentState_en = Lora_EntryRxMode_en;

	}break;
	case Lora_EntryRxMode_en :{

			Lora_vWriteReg(RegOpMode,RxContMode);
			Lora_RxCurrentState_en = Lora_RxModeTest_en;

	}break;
	case Lora_RxModeTest_en:{
		Lora_status_u8 =( (Lora_u8ReadReg(RegModemStat) & 0x04) == 0x04);
		Lora_RxCurrentState_en = (Lora_status_u8==1 )? Lora_RxWaiting_en :Lora_RxReset_en ;

	}break;
	case Lora_RxWaiting_en:{

		Lora_RxDoneHw_u8 = Lora_u8GetDIO0();
		if( Lora_RxDoneHw_u8 == 1){
			Lora_RxDoneSw_u8 = ( (Lora_u8ReadReg(RegIrqFlags) & 0x40) == 0x40);
			Lora_RxCurrentState_en = Lora_RxReadPacket_en;
		}

	}break;
	case Lora_RxReadPacket_en: {

		RegAdr = Lora_u8ReadReg(RegFifoRxCurrentAdr);
		Lora_vWriteReg(RegFifoAdrPtr,RegAdr);
		packetSize = Lora_u8ReadReg(RegRxNbBytes);
		Lora_vReadFifo(RegFifo, RxBuffer, packetSize);
		Lora_RxCurrentState_en = Lora_RxDone_en;
	}break;
	case Lora_RxDone_en:{
		Lora_clearIrq();
		Lora_vWriteReg(RegOpMode,StdbyMode);
		Lora_RxCurrentState_en = Lora_RxReady_en ;
	}break;
	case Lora_RxTimeout_en:{

	}break;
	case Lora_RxReset_en:{
		Lora_vReset() ;
		Lora_RxCurrentState_en = Lora_ModemConfigRx;
	}break;
	case Lora_RxReady_en:{

	}break;

	}
	return Lora_RxCurrentState_en;
}
/**
  * @brief	state machine describing the functioning of the Lora Modem in Tx mode
  * @param	TxBuffer is a pointer to an array containing the packet to be sent
  * @param	payloadLength is the amount of data to be sent
  * @retval	Lora_TxMode state
  */
Lora_TxStates_ten Lora_tenTxMode(uint8_t* TxBuffer, uint8_t payloadLength){


	switch(Lora_TxCurrentState_en){
	case Lora_TxIdle_en:{
		Lora_TxCurrentState_en = Lora_ModemConfigTx;
		}break;
	case Lora_ModemConfigTx:{
		Lora_vConfig();
		Lora_TxCurrentState_en = Lora_TxConfig_en;
	}break;
	case Lora_TxConfig_en :{
		Lora_vTxConfig(payloadLength);
		Lora_TxCurrentState_en = Lora_TxConfigTest_en;
	}break;
	case Lora_TxConfigTest_en:{
		Lora_status_u8 = (Lora_u8ReadReg(RegPayloadLength) == payloadLength);
		Lora_TxCurrentState_en =  (Lora_status_u8==1 )? Lora_EntryTxMode_en :Lora_TxReset_en ;

		}break;
	case Lora_EntryTxMode_en:{
		Lora_vWriteFifo(TxBuffer, payloadLength);
		Lora_vWriteReg(RegOpMode,TxMode);
		Lora_TxCurrentState_en = Lora_TxTest_en ;
	}break;
	case Lora_TxTest_en:{
		Lora_TxDoneHw_u8 = Lora_u8GetDIO0();
		  if ( Lora_TxDoneHw_u8 == 1 )  {
			  Lora_TxCurrentState_en = Lora_TxDone_en;
		  }

	}break;
	case Lora_TxDone_en:{

		Lora_TxDoneSw_u8 = ( (Lora_u8ReadReg(RegIrqFlags) & 0x08) == 0x08);
		Lora_clearIrq();
		Lora_vWriteReg(RegOpMode,StdbyMode);
		Lora_TxCurrentState_en = Lora_TxReady_en ;

	}break;
	case Lora_TxReset_en:{
		Lora_vReset() ;
		Lora_TxCurrentState_en=Lora_TxConfig_en ;

	}break;
	case Lora_TxReady_en :{

	}break;

	}

	return Lora_TxCurrentState_en;
}

