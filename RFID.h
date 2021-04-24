
#ifndef RFID_H
#define	RFID_H

#include <xc.h> 
#include <stdint.h>
#include <stdbool.h>
#define MFRC552                  0x28   // chip 0101b ADDRESS
#define CommandReg				 0x01 	// starts and stops command execution
#define ComIEnReg				 0x02 	// enable and disable interrupt request control bits
#define DivIEnReg				 0x03 	// enable and disable interrupt request control bits
#define ComIrqReg				 0x04 	// interrupt request bits
#define DivIrqReg				 0x05 	// interrupt request bits
#define ErrorReg				 0x06 	// error bits showing the error status of the last command executed
#define Status1Reg				 0x07 	// communication status bits
#define Status2Reg				 0x08 	// receiver and transmitter status bits
#define FIFODataReg				 0x09 	// input and output of 64 byte FIFO buffer
#define FIFOLevelReg			 0x0A 	// number of bytes stored in the FIFO buffer
#define WaterLevelReg			 0x0B 	// level for FIFO underflow and overflow warning
#define ControlReg				 0x0C 	// miscellaneous control registers
#define BitFramingReg			 0x0D 	// adjustments for bit-oriented frames
#define CollReg					 0x0E 	// bit position of the first bit-collision detected on the RF interface

// Page 1: Command
#define ModeReg					 0x11 	// defines general modes for transmitting and receiving
#define TxModeReg				 0x12 	// defines transmission data rate and framing
#define RxModeReg				 0x13 	// defines reception data rate and framing
#define TxControlReg			 0x14 	// controls the logical behavior of the antenna driver pins TX1 and TX2
#define TxASKReg				 0x15 	// controls the setting of the transmission modulation
#define TxSelReg				 0x16 	// selects the internal sources for the antenna driver
#define RxSelReg				 0x17 	// selects internal receiver settings
#define RxThresholdReg			 0x18 	// selects thresholds for the bit decoder
#define DemodReg				 0x19 	// defines demodulator settings
#define MfTxReg					 0x1C 	// controls some MIFARE communication transmit parameters
#define MfRxReg					 0x1D 	// controls some MIFARE communication receive parameters
#define SerialSpeedReg			 0x1F 	// selects the speed of the serial UART interface

// Page 2: Configuration
#define CRCResultRegM			 0x21 	// shows the MSB and LSB values of the CRC calculation
#define CRCResultRegL			 0x22 
#define ModWidthReg				 0x24 	// controls the ModWidth setting?
#define RFCfgReg				 0x26 	// configures the receiver gain
#define GsNReg					 0x27 	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define CWGsPReg				 0x28 	// defines the conductance of the p-driver output during periods of no modulation
#define ModGsPReg				 0x29 	// defines the conductance of the p-driver output during periods of modulation
#define TModeReg				 0x2A 	// defines settings for the internal timer
#define TPrescalerReg			 0x2B 	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
#define TReloadRegH				 0x2C 	// defines the 16-bit timer reload value
#define TReloadRegL				 0x2D 
#define TCounterValueRegH		 0x2E 	// shows the 16-bit timer value
#define TCounterValueRegL		 0x2F 

// Page 3: Test Registers
#define TestSel1Reg				 0x31 	// general test signal configuration
#define TestSel2Reg				 0x32 	// general test signal configuration
#define TestPinEnReg			 0x33 	// enables pin output driver on pins D1 to D7
#define TestPinValueReg			 0x34 	// defines the values for D1 to D7 when it is used as an I/O bus
#define TestBusReg				 0x35 	// shows the status of the internal test bus
#define AutoTestReg				 0x36 	// controls the digital self test
#define VersionReg				 0x37 	// shows the software version
#define AnalogTestReg			 0x38 	// controls the pins AUX1 and AUX2
#define TestDAC1Reg				 0x39 	// defines the test value for TestDAC1
#define TestDAC2Reg				 0x3A 	// defines the test value for TestDAC2
#define TestADCReg				 0x3B 		// shows the value of ADC I and Q channels

// MFRC522 commands. Described in chapter 10 of the datasheet.
#define PCD_IDLE                 0x00   //NO action; Cancel the current command
#define PCD_Mem					 0x01		// stores 25 bytes into the internal buffer
#define PCD_GenerateRandomID	 0x02		// generates a 10-byte random ID number
#define PCD_CALCCRC				 0x03		// activates the CRC coprocessor or performs a self test
#define PCD_Transmit			 0x04		// transmits data from the FIFO buffer
#define PCD_NoCmdChange			 0x07		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define PCD_Receive				 0x08		// activates the receiver circuits
#define PCD_TRANSCEIVE 			 0x0C		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define PCD_AUTHENT 			 0x0E		// performs the MIFARE standard authentication as a reader
#define PCD_RESETPHASE           0x0F       //Reset

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD). Described in 9.3.3.6 / table 98 of the datasheet
#define RxGain_18dB				 0x00 << 4	// 000b - 18 dB, minimum
#define RxGain_23dB				 0x01 << 4	// 001b - 23 dB
#define RxGain_18dB_2			 0x02 << 4	// 010b - 18 dB, it seems 010b is a duplicate for 000b
#define RxGain_23dB_2			 0x03 << 4	// 011b - 23 dB, it seems 011b is a duplicate for 001b
#define RxGain_33dB				 0x04 << 4	// 100b - 33 dB, average, and typical default
#define RxGain_38dB				 0x05 << 4	// 101b - 38 dB
#define RxGain_43dB				 0x06 << 4	// 110b - 43 dB
#define RxGain_48dB				 0x07 << 4	// 111b - 48 dB, maximum
#define RxGain_min				 0x00 << 4	// 000b - 18 dB, minimum, convenience for RxGain_18dB
#define RxGain_avg				 0x04 << 4	// 100b - 33 dB, average, convenience for RxGain_33dB
#define RxGain_max				 0x07 << 4		// 111b - 48 dB, maximum, convenience for RxGain_48dB

// Commands sent to the PICC.
// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
#define PICC_REQIDL              0x26		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define PICC_READ                0x30       // Read Block
#define PICC_CMD_WUPA			 0x52		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_CT				 0x88		// Cascade Tag. Not really a command, but used during anti collision.
#define PICC_ANTICOLL            0x93       // anti-collision#define PICC_CMD_SEL_CL2	
#define PICC_CMD_SEL_CL3		 0x97		// Anti collision/Select, Cascade Level 3
#define PICC_HALT                0x50		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
#define PICC_SElECTTAG           0x93       // election card

// The commands used for MIFARE Classic (from http://www.nxp.com/documents/data_sheet/MF1S503x.pdf, Section 9)
// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
// The read/write commands can also be used for MIFARE Ultralight.
#define PICC_CMD_MF_AUTH_KEY_A	 0x60		// Perform authentication with Key A
#define PICC_CMD_MF_AUTH_KEY_B	 0x61		// Perform authentication with Key B
#define PICC_CMD_MF_READ		 0x30		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define PICC_WRITE               0xA0		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define PICC_CMD_MF_DECREMENT	 0xC0		// Decrements the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_INCREMENT	 0xC1		// Increments the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_RESTORE		 0xC2		// Reads the contents of a block into the internal data register.
#define PICC_CMD_MF_TRANSFER	 0xB0		// Writes the contents of the internal data register to a block.

// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
#define PICC_CMD_UL_WRITE		 0xA2		// Writes one 4 byte page to the PICC.
#define MAX_LEN 16

// status stuff And MF522 The error code is returned when communication
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2
const unsigned short    inputStringLen = 1;
unsigned char           inputString[1] = "";               // a string to hold incoming data
bool                    stringComplete;                                 // whether the string is complete
unsigned short          bytevar3, bytevar4, Output1, receiv_cnt = 0;
unsigned short          receive[8];
unsigned char           bytevar1, bytevar2 = 0;
unsigned char           serNum[5];
unsigned char           uchar_send[8];
unsigned char           read[10];
unsigned char           keyI[7]={65, 67, 50, 48, 57, 70, 49, 54};
unsigned char           keyII[7]={50, 67, 50, 48, 50, 50, 50, 51};


// Hint: if needed, you can remove unused self-test data to save flash memory
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 Sefttest
//const uint8_t MFRC522_firmware_referenceV0_0[64]  = {
//	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
//	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
//	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
//	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
//	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
//	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
//	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
//	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
//};
//// Version 1.0 (0x91)
//// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 Self test
//const uint8_t MFRC522_firmware_referenceV1_0[64]  = {
//	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
//	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
//	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
//	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
//	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
//	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
//	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
//	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
//};
//// Version 2.0 (0x92)
//// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 Self test
//const uint8_t MFRC522_firmware_referenceV2_0[64]  = {
//	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
//	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
//	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
//	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
//	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
//	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
//	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
//	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
//};
//// Clone
//// Fudan Semiconductor FM17522 (0x88)
//const uint8_t FM17522_firmware_reference[64]  = {
//	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
//	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
//	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
//	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
//	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
//	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
//	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
//	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
//};


void RFID_ReadCard(uint8_t value);
bool RFID_CheckData(uint8_t result[64], uint8_t reference[64]);
void MFRC522_Init(void);
void PCD_Writeregister(	uint8_t reg, uint8_t data );
void PCD_WriteRegister(uint8_t reg, uint8_t length, uint8_t data);
uint8_t PCD_Readregister( uint8_t reg );
//void PCD_ReadRegister( uint8_t reg, uint8_t values , uint8_t count);
void SetFormatRDM630(void);
char Separate_hexP10(char val);
char Separate_hexP1(char val);
void SetBitMask(uint8_t reg, uint8_t mask);
void ClearBitMask(uint8_t reg, uint8_t mask);
void AntennaOn(void);
void AntennaOff(void);
void MFRC522_Reset(void);
unsigned char MFRC522_Request(unsigned char reqMode, unsigned char *TagType);
unsigned char MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen);
unsigned char MFRC522_Anticoll(unsigned char *serNum);
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData);
unsigned char MFRC522_SelectTag(unsigned char *serNum);
unsigned char MFRC522_Auth(unsigned char authMode, unsigned char BlockAddr, unsigned char *Sectorkey, unsigned char *serNum);
unsigned char MFRC522_Read(unsigned char blockAddr, unsigned char *recvData);
unsigned char MFRC522_Write(unsigned char blockAddr, unsigned char *writeData);
void MFRC522_Halt(void);


bool checkCRC = false;
uint8_t rxAlign = 0;
uint8_t *validBits = NULL;
uint8_t *backLen = NULL;
uint8_t *backData = NULL;


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

