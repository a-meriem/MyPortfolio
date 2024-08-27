/*
 * Lora.h
 *
 *      Author: Meriem
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_


#include "stdint.h"
#include "main.h"

#define RegFifo					0x00
#define RegOpMode 				0x01
#define RegPaConfig 			0x09
#define RegFifoAdrPtr 			0x0D
#define RegFifoTxBaseAdr 		0x0E
#define RegFifoRxBaseAdr 		0x0F
#define RegFifoRxCurrentAdr 	0X10
#define RegIrqFlags				0X12
#define RegRxNbBytes 			0X13
#define RegFifoRxByteAdr 		0x25
#define RegModemConfig1			0X1D
#define RegModemConfig2			0X1E
#define RegModemConfig3			0X26
#define RegRxHeaderCntValueMsb	0x14
#define RegRxHeaderCntValueLsb	0x15
#define RegRxPacketCntValueMsb	0x16
#define RegRxPacketCntValueLsb	0x17
#define RegModemStat			0x18
#define RegPayloadLength		0x22
#define RegPreambleMsb			0x20
#define RegPreambleLsb			0x21
#define RegFrMsb				0x06
#define RegFrMid				0x07
#define RegFrLsb				0x08
#define RegSyncWord				0x39
#define	RegOcp					0X0B
#define	RegLna					0X0C
#define RegSymbTimeoutLsb		0x1F
#define RegDioMapping1			0x40
#define RegDioMapping2			0x41
#define RegPaDac				0x4D
#define	RegHopPeriod			0x24
#define	RegIrqFlagsMask			0x11
#define	RegPayloadLength	0x22

#define SleepMode			0x00
#define LoraMode			0x88
#define StdbyMode			0x89
#define RxContMode			0x85
#define TxMode				0x83
#define DefaultMaxPower		0xFC
#define Config1				0x72 //BW 125KHz, CR 4/5, Exlicit header mode
#define Config2				0x74 //SF 7, enable crc
#define Config3				0x04
#define TxBaseAdr			0x00
#define RxBaseAdr			0x80
#define PayloadLength		0x01
#define PreambleLsb			0x08
#define PreambleMsb			0x00
#define FrMsb				0x6C
#define FrMid				0x80
#define FrLsb				0x00
//#define FrMsb				0x6E
//#define FrMid				0x00
//#define FrLsb				0x00

#define SyncWord			0x34
#define	Ocp					0x0B
#define	Lna					0x23
#define SymbTimeoutLsb		0x08
#define DioMapping2			0x01
#define ClearIrqFlags		0xFF
#define RxPaDac				0x4D
#define	RxHopPeriod			0xFF
#define	RxDioMapping1		0X01
#define	RxIrqFlagsMask		0x3F
#define TxPaDac				0x87
#define	TxHopPeriod			0x00
#define	TxDioMapping1		0X41
#define	TxIrqFlagsMask		0xF7



typedef enum{
	Lora_RxIdle_en,
	Lora_ModemConfigRx,
	Lora_RxReady_en,
	Lora_RxConfig_en,
	Lora_EntryRxMode_en,
	Lora_RxModeTest_en,
	Lora_RxWaiting_en,
	Lora_RxReadPacket_en,
	Lora_RxDone_en,
	Lora_RxTimeout_en,
	Lora_RxReset_en
}Lora_RxStates_ten;

typedef enum{
	Lora_TxIdle_en,
	Lora_ModemConfigTx,
	Lora_TxReady_en,
	Lora_TxConfig_en,
	Lora_TxConfigTest_en,
	Lora_EntryTxMode_en,
	Lora_TxTest_en,
	Lora_TxDone_en,
	Lora_TxReset_en
}Lora_TxStates_ten;

extern volatile uint8_t Lora_Receive_Complete;
extern volatile uint8_t Lora_Transmit_Complete;

void Lora_vSetNSS(int value);

void Lora_vInit();

void Lora_vReset() ;

void Lora_WaitForTx();

void Lora_WaitForRx();

void Lora_vWriteReg(uint8_t regAdress, uint8_t data);

uint8_t Lora_u8ReadReg(uint8_t regAdress);

void Lora_vBurstWriteReg( uint8_t RegAdr, uint8_t * data, uint8_t length);

void Lora_vBurstReadReg(uint8_t RegAdr, uint8_t * data, uint8_t length);

uint8_t Lora_u8GetDIO0() ;

void Lora_vConfig();

void Lora_clearIrq();

void Lora_vRxConfig(uint8_t length);

void Lora_vTxConfig(uint8_t length);

Lora_RxStates_ten Lora_tenRxMode(uint8_t* RxBuffer, uint8_t payloadLength);

Lora_TxStates_ten Lora_tenTxMode(uint8_t* TxBuffer, uint8_t payloadLength);

#endif /* INC_LORA_H_ */
