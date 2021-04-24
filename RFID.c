#include <xc.h>
#include "RFID.h"
#include <stdbool.h>
#include "mcc_generated_files/device_config.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "math.h"


/*
 * Function Name: InitMFRC522
 * Description: Initialize RC522
 * Input: None
 * Return value: None
*/
void MFRC522_Init(void)
{
  IO_RA5_SetHigh();
  //NRSTPD = 1;// reset pin high

  MFRC522_Reset();

  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  PCD_Writeregister(TModeReg, 0x8D);                //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  PCD_Writeregister(TPrescalerReg, 0x3E);        //TModeReg[3..0] + TPrescalerReg
  PCD_Writeregister(TReloadRegL, 30);
  PCD_Writeregister(TReloadRegH, 0);

  PCD_Writeregister(TxASKReg, 0x40);                //100%ASK
  PCD_Writeregister(ModeReg, 0x3D);                //CRC Initial value 0x6363        ???

  //ClearBitMask(Status2Reg, 0x08);                //MFCrypto1On=0
  //Write_MFRC522(RxSelReg, 0x86);                //RxWait = RxSelReg[5..0]
  //Write_MFRC522(RFCfgReg, 0x7F);                   //RxGain = 48dB

  AntennaOn();                //Open the antenna
}

void PCD_Writeregister(	uint8_t reg, uint8_t data ) 
{
    ///< The register to write to. One of the PCD_Register enums.
    ///< The value to write.
    I2C1_Write1ByteRegister(MFRC552, reg, data);
}

void PCD_WriteRegister(uint8_t reg, uint8_t length, uint8_t data)
{
    I2C1_Write1ByteRegister(MFRC552,  reg,  data);
}

uint8_t PCD_Readregister( uint8_t reg ) 
{
    ///< The register to read from. One of the PCD_Register enums.
	uint8_t value = I2C1_Read1ByteRegister(MFRC552, reg);
}

//void PCD_ReadRegister( uint8_t reg, uint8_t *values , uint8_t count) 
//{
//    ///< The register to read from. One of the PCD_Register enums.
//    ///< Byte array to store the values in.
//    ///< The number of bytes to read
//    I2C1_ReadDataBlock(MFRC552, reg, *values, count);
//}


void SetFormatRDM630(void)
{
  /*uchar checksum1;

  checksum1 = serNum[0] ^ serNum[1] ^ serNum[2] ^ serNum[3] ^ serNum[4];*/
  //uchar_send[0] = 2;

  uchar_send[0] = Separate_hexP10(serNum[0]);
  uchar_send[1] = Separate_hexP1(serNum[0]);
  uchar_send[2] = Separate_hexP10(serNum[1]);
  uchar_send[3] = Separate_hexP1(serNum[1]);
  uchar_send[4] = Separate_hexP10(serNum[2]);
  uchar_send[5] = Separate_hexP1(serNum[2]);
  uchar_send[6] = Separate_hexP10(serNum[3]);
  uchar_send[7] = Separate_hexP1(serNum[3]);
  //uchar_send[9] = Separate_hexP10(serNum[4]);
  //uchar_send[10] = Separate_hexP1(serNum[4]);

  //uchar_send[11] = Separate_hexP10(checksum1);
  //uchar_send[12] = Separate_hexP1(checksum1);

  //uchar_send[13] = 3;
  //uchar_send[14] = 0;
 read[0]= uchar_send[0];    //serNum[0] 4MSB of PICC_ANTICOLL(0x93)== 1001 == 9
 read[1]= uchar_send[1];    //serNum[0] 4LSB of PICC_ANTICOLL(0x93)== 0011 == 3
 read[2]= uchar_send[2];    //serNum[1] 4MSB of 0x20 == 10 == 2
 read[3]= uchar_send[3];    //serNum[1] 4LSB of 0x20 == 0 == 0
 read[4]= uchar_send[4];    //serNum[2] 4MSB
 read[5]= uchar_send[5];    //serNum[2] 4LSB
 read[6]= uchar_send[6];    //serNum[3] 4MSB
 read[7]= uchar_send[7];    //serNum[3] 4LSB
 //read becomes an array of 4bytes
}

/*
 * Function Name: Separate_hexP10
 * Function Description: To strip data high from full data
 * Input Parameters: val - the value to be strip
 * Return value: Part High
 */
char Separate_hexP10(char val) 
{ //checks 4MSB
  val = val & 0xF0;
  val = val >> 4; //shitf 4 most significant
  if (val < 10)
  {
      return val + 48;
  }
  else
  {
      return val + 55;
  }
}

/*
 * Function Name: Separate_hexP1
 * Function Description: To strip data low from full data
 * Input Parameters: val - the value to be strip
 * Return value: Part Low
 */
char Separate_hexP1(char val)
{ //check 4LSB
  val = val & 0x0F;
  if (val < 10)
  {
      return val + 48;
  }
  else
  {
      return val + 55;
  }
}




/*
 * Function Name: SetBitMask
 * Description: Set RC522 register bit
 * Input parameters: reg - register address; mask - set value
 * Return value: None
 */
void SetBitMask(uint8_t reg, uint8_t mask)
{
  uint8_t tmp;
  tmp = PCD_Readregister(reg);
  PCD_Writeregister(reg, tmp | mask);  // set bit mask
}


/*
 * Function Name: ClearBitMask
 * Description: clear RC522 register bit
 * Input parameters: reg - register address; mask - clear bit value
 * Return value: None
*/
void ClearBitMask(uint8_t reg, uint8_t mask)
{
  uint8_t tmp;
  tmp = PCD_Readregister(reg);
  PCD_Writeregister(reg, tmp & (~mask));  // clear bit mask
}


/*
 * Function Name: AntennaOn
 * Description: Open antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
 * Input: None
 * Return value: None
 */
void AntennaOn(void)
  {
  uint8_t temp;
  temp = PCD_Readregister(TxControlReg);
  if (!(temp & 0x03))
  {
    SetBitMask(TxControlReg, 0x03);
  }
}


/*
  * Function Name: AntennaOff
  * Description: Close antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
  * Input: None
  * Return value: None
 */
void AntennaOff(void)
{
  ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function Name: ResetMFRC522
 * Description: Reset RC522
 * Input: None
 * Return value: None
 */
void MFRC522_Reset(void)
{
  PCD_Writeregister(CommandReg, PCD_RESETPHASE);
}


/*
 * Function Name: MFRC522_Request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *                         TagType - Return Card Type
 *                                0x4400 = Mifare_UltraLight
 *                                0x0400 = Mifare_One(S50)
 *                                0x0200 = Mifare_One(S70)
 *                                0x0800 = Mifare_Pro(X)
 *                                0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Request(unsigned char reqMode, unsigned char *TagType)
{
  unsigned char status;
  uint8_t backBits;                        //The received data bits

  PCD_Writeregister(BitFramingReg, 0x07);                //TxLastBists = BitFramingReg[2..0]        ???

  TagType[0] = reqMode;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10))
  {
    status = MI_ERR;
  }

  return status;                                      //returns tag type that includes FIFO data that was read
}


/*
 * Function Name: MFRC522_ToCard
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 *                         sendData--RC522 sent to the card by the data
 *                         sendLen--Length of data sent
 *                         backData--Received the card returns data,
 *                         backLen--Return data bit length
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen)
{
  unsigned char status = MI_ERR;
  uint8_t irqEn = 0x00;
  uint8_t waitIRq = 0x00;
  uint8_t lastBits;
  uint8_t n;
  int i;

  switch (command)
  {
    case PCD_AUTHENT:                //Certification cards close
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:        //Transmit FIFO data
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
        break;
  }

  PCD_Writeregister(ComIEnReg, irqEn|0x80);             //Interrupt request
  ClearBitMask(ComIrqReg, 0x80);                        //Clear all interrupt request bit
  SetBitMask(FIFOLevelReg, 0x80);                       //FlushBuffer=1, FIFO Initialization

  PCD_Writeregister(CommandReg, PCD_IDLE);              //NO action; Cancel the current command???

  //Writing data to the FIFO
  for (i=0; i<sendLen; i++)
  {
    PCD_Writeregister(FIFODataReg, sendData[i]);        //tell FIFO to go ready
  }

  //Execute the command
  PCD_Writeregister(CommandReg, command);           //// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
  if (command == PCD_TRANSCEIVE)
  {
    SetBitMask(BitFramingReg, 0x80);                //StartSend=1,transmission of data starts
  }

  //Waiting to receive data to complete
  i = 2000;        //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
  do
  {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = PCD_Readregister(ComIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  ClearBitMask(BitFramingReg, 0x80);                        //StartSend=0

  if (i != 0)
  {
    if(!(PCD_Readregister(ErrorReg) & 0x1B))        //BufferOvflow Collerr CRCErr ProtecolErr
    {
      status = MI_OK;
      if (n & irqEn & 0x01)
      {
        status = MI_NOTAGERR;                        //??
      }

      if (command == PCD_TRANSCEIVE)
      {
        n = PCD_Readregister(FIFOLevelReg);             // number of bytes stored in the FIFO buffer
        lastBits = PCD_Readregister(ControlReg) & 0x07; //read the first three bits of control register Binary AND Operator copies a bit to the result if it exists in both operands. 
        if (lastBits)
        {
          *backLen = (n-1)*8 + lastBits;
        }
        else
        {
          *backLen = n*8;
        }

        if (n == 0)
        {
          n = 1;
        }
        if (n > MAX_LEN)
        {
          n = MAX_LEN;  //max length is 16
        }

        //Reading the received data in FIFO
        for (i=0; i<n; i++)
        {
          backData[i] = PCD_Readregister(FIFODataReg); //what is being returned. up to 16 bits will be stored in return.
        }
      }
    }
    else
    {
      status = MI_ERR;
    }

  }

  //SetBitMask(ControlReg,0x80);           //timer stops
  //Write_MFRC522(CommandReg, PCD_IDLE);

  return status;
}


/*
 * Function Name: MFRC522_Anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Anticoll(unsigned char *serNum)
{
  unsigned char status;
  unsigned char i;
  unsigned char serNumCheck=0;
  uint8_t unLen;


  //ClearBitMask(Status2Reg, 0x08);                //TempSensclear
  //ClearBitMask(CollReg,0x80);                    //ValuesAfterColl
  PCD_Writeregister(BitFramingReg, 0x00);          //TxLastBists = BitFramingReg[2..0] //transmit evething in right fashion

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen); //redefine status with already stored tag data to recieve back 

  if (status == MI_OK)
  {
    //Check card serial number
    for (i=0; i<4; i++) // checks the third bit of serNum
    {
      serNumCheck ^= serNum[i];             //stores in serNumcheck Bitwise exclusive OR and assignment operator. ie(0 or 0x44=0x44)
    }
    if (serNumCheck != serNum[i])
    {
      status = MI_ERR;
    }
  }

  //SetBitMask(CollReg, 0x80);                //ValuesAfterColl=1

  return status;
}


/*
 * Function Name: CalulateCRC
 * Description: CRC calculation with MF522
 * Input parameters: pIndata - To read the CRC data, len - the data length, pOutData - CRC calculation results
 * Return value: None
 */
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
  unsigned char i, n;

  ClearBitMask(DivIrqReg, 0x04);                        //CRCIrq = 0
  SetBitMask(FIFOLevelReg, 0x80);                        //Clear the FIFO pointer
  //Write_MFRC522(CommandReg, PCD_IDLE);

  //Writing data to the FIFO
  for (i=0; i<len; i++)
  {
    PCD_Writeregister(FIFODataReg, *(pIndata+i));
  }
  PCD_Writeregister(CommandReg, PCD_CALCCRC);

  //Wait CRC calculation is complete
  i = 0xFF;
  do
  {
    n = PCD_Readregister(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04));                        //CRCIrq = 1

  //Read CRC calculation result
  pOutData[0] = PCD_Readregister(CRCResultRegL);
  pOutData[1] = PCD_Readregister(CRCResultRegM);
}


/*
 * Function Name: MFRC522_SelectTag
 * Description: election card, read the card memory capacity
 * Input parameters: serNum - Incoming card serial number
 * Return value: the successful return of card capacity
 */
unsigned char MFRC522_SelectTag(unsigned char *serNum)
{
  unsigned char i;
  unsigned char status;
  unsigned char size;
  uint8_t recvBits;
  unsigned char buffer[9];

  //ClearBitMask(Status2Reg, 0x08);                        //MFCrypto1On=0

  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  for (i=0; i<5; i++)
  {
    buffer[i+2] = *(serNum+i);
  }
  CalulateCRC(buffer, 7, &buffer[7]);                //??
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

  if ((status == MI_OK) && (recvBits == 0x18))
  {
    size = buffer[0];
  }
  else
  {
    size = 0;
  }

  return size;
}


/*
 * Function Name: MFRC522_Auth
 * Description: Verify card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = A key authentication
                 0x61 = Authentication Key B
             BlockAddr--Block address
             Sectorkey--Sector password
             serNum--Card serial number, 4-byte
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Auth(unsigned char authMode, unsigned char BlockAddr, unsigned char *Sectorkey, unsigned char *serNum)
{
  unsigned char status;
  uint8_t recvBits;
  unsigned char i;
  unsigned char buff[12];

  //Verify the command block address + sector + password + card serial number
  buff[0] = authMode;
  buff[1] = BlockAddr;
  for (i=0; i<6; i++)
  {
    buff[i+2] = *(Sectorkey+i);
  }
  for (i=0; i<4; i++)
  {
    buff[i+8] = *(serNum+i);
  }
  status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

  if ((status != MI_OK) || (!(PCD_Readregister(Status2Reg) & 0x08)))
  {
    status = MI_ERR;
  }

  return status;
}


/*
 * Function Name: MFRC522_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Read(unsigned char blockAddr, unsigned char *recvData)
{
  unsigned char status;
  uint8_t unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  CalulateCRC(recvData,2, &recvData[2]);
  status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

  if ((status != MI_OK) || (unLen != 0x90))
  {
    status = MI_ERR;
  }

  return status;
}


/*
 * Function Name: MFRC522_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Write(unsigned char blockAddr, unsigned char *writeData)
{
  unsigned char status;
  uint8_t recvBits;
  unsigned char i;
  unsigned char buff[18];

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  CalulateCRC(buff, 2, &buff[2]);
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

  if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
  {
    status = MI_ERR;
  }

  if (status == MI_OK)
  {
    for (i=0; i<16; i++)                //Data to the FIFO write 16Byte
    {
      buff[i] = *(writeData+i);
    }
    CalulateCRC(buff, 16, &buff[16]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
      status = MI_ERR;
    }
  }

  return status;
}


/*
 * Function Name: MFRC522_Halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void MFRC522_Halt(void)
{
  unsigned char status;
  uint8_t unLen;
  unsigned char buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  CalulateCRC(buff, 2, &buff[2]);

  status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}