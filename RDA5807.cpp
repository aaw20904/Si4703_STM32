/****!!!!! FOR SiLabs SI4703 chip*******************/
#include "main.h"
#include "RDA5807.h"
/*to init interface for I2C - 
1) set RST to LOW and delay 1mS
2)- set SDIO to LOW
3) connect SEN to HIGH
4) delay 1ms
5) set RST to HIGH
6) delay
7)set RST to HIGH
If this bspeps has been done -  you can write to registers of Si4703
for more details - see MX_I2C1_Init() function in 'main.c' file*/

/***IMPORTANT    - the adress of si4703 must been shifted to the left on 1 digit*/  
extern I2C_HandleTypeDef hi2c1;

    void transmittBufferToRDA5807(void){
      
    }
   
   
  /*****02h*****P O W E R C F G************************0-1***/
    
    void SI4703_SetAtomicPOWERCFGR (uint16_t x) {
       /**hight byte*/
      si4703WriteBuffer[0] = (x & 0xff00) >> 8;
      /***low byte**/
      si4703WriteBuffer[1] = x & 0x00ff; 
    }
    /***03h**C H A N N EL************************2-3*/
    #define SI4703_03_TUNE (1 << 15)
    void SI4703_SetAtomicCHANNEL (uint16_t x) {
      /*hight*/
      /*clear*/
      si4703WriteBuffer[2] &= ~(0x80);
      /*set*/
      si4703WriteBuffer[2] |= x >> 8;
    }
    void SI4703_SetCHANNEL(uint16_t ch) {
      /*low byte*/
      /*clear*/
      si4703WriteBuffer[3] &= 0x00;
      /*set*/
      si4703WriteBuffer[3] |= ch & 0x00FF;
      /*high byte*/
      /*clear*/
      si4703WriteBuffer[2] &= ~(0x03);
      /*set*/
      si4703WriteBuffer[2] |= (ch >> 8 ) & 0x03;
    }
    /**b0-b9 - channel*/
    /***04h**S Y S C O N F I G 1*******4-5*****/
    void SI4703_SetAtomicSYSCONFIG1 (uint16_t x) {
      /*hight*/
      si4703WriteBuffer[4] = (x & 0xFF00) >> 8;
    }
    void SI4703_SetBLNDADJ (uint8_t val) {
      /*low*/
      /*clear*/
      si4703WriteBuffer[5] &= ~(0xC0);
      /*set*/
      si4703WriteBuffer[5] |= (val & 0x03) << 6;
    }
    void SI4703_SetGPIO (uint8_t GP1, uint8_t GP2, uint8_t GP3) {
      /*low*/
      /*clear*/
      si4703WriteBuffer[5] &= ~(0x3F);
      /*set*/
      si4703WriteBuffer[5] |= (GP3 >> 4) | (GP2 >>2) | GP1;
    }
    /*BLNDADJ[1:0] GPIO3[1:0] GPIO2[1:0] GPIO1[1:0]*/
   
    /*****05h**************S Y S C O N F I G 2********6-7*******/
    void SI4703_SetSEEKTH (uint8_t x) {
      /*high*/
      si4703WriteBuffer[6] = x;
    }
    void SI4703_SetBAND (uint8_t x) {
      /*low*/
      /*clear*/
      si4703WriteBuffer[7] &= ~(0xC0);
      /*set*/
      si4703WriteBuffer[7] |= (x << 6);
    }
    
    void SI4307_SetSPACE (uint8_t x) {
      /*low - clear*/
      si4703WriteBuffer[7] &= ~(0x30);
      /*set*/
      si4703WriteBuffer[7] |= x << 4;
    }
    
    void SI4703_SetVOLUME(uint8_t x) {
      /*low -clear*/
      si4703WriteBuffer[7] &= ~(0x0F);
      /*set*/
      si4703WriteBuffer[7] |= x;
    }
      /*SEEKTH[7:0] BAND[1:0] SPACE[1:0] VOLUME[3:0]*/
    /****06h**S Y S C O N F I G 3***********************8-9******/
    void SI4703_SetAtomicSYSCFGR3 (unsigned short x) {
      /***high - clear**/
      si4703WriteBuffer[8]  &= ~(0x01);
      /*set*/
      si4703WriteBuffer[8] |= x;
    }
    
    void SI4703_SetSKSNR (uint8_t x) {
      /*low - clear*/
      si4703WriteBuffer[9] &= ~(0xF0);
      /*set*/
      si4703WriteBuffer[9] |= x << 4;
    }
    
    void SI4703_SetSKCNT (uint8_t x) {
      /*low-clear*/
      si4703WriteBuffer[9] &= ~(0x0F);
      /*set*/
      si4703WriteBuffer[9] |= x;
    }
    
    void SI4703_SetSMUTER (uint8_t x) {
      /*high - clear*/
      si4703WriteBuffer[8] &= ~(0xC0);
      /*set*/
      si4703WriteBuffer[8] |= x << 6;
    }
    
    void SI4703_SetSMUTEA (uint8_t x) {
      /*high - clear*/
      si4703WriteBuffer[8] &= ~(0x30);
      /*set*/
      si4703WriteBuffer[8] |= x << 4;
    }
    /*SKSNR[3:0] SKCNT[3:0] SMUTER[1:0] SMUTEA[1:0]*/
    
    /*****07h****T E S T 1*************************10-11*******/
     void SI4703_SetAtomicTEST1 (unsigned short x) {
       //clear XOSCEN
       si4703WriteBuffer[10] &= ~(1 << 7);
      /*high - set*/
      si4703WriteBuffer[10] |= (x >> 8);
    }
    
    
    /****08h**T E S T 2:12-13******/
    /*****09h****B O O T C O N F I G:14-15**/
    
    /**_____________FOR   R e A d_______***/
    /***0Ah**S T A T U S R S S I********************0-1**/
    unsigned char SI4703_GetRSSI (void) {
      /*low*/
      return si4703WriteBuffer[1];
    }
    
    uint8_t SI4703_GetBLERA (void) {
      return (si4703WriteBuffer[0] & 0x06) >> 1;
    }
    
    uint16_t SI4703_GetAtomicSTATUSRSSI (void) {
      /*high - excluding BLERA*/
      uint16_t z = (si4703WriteBuffer[0] & 0xf9);
      z <<= 8;
      return z;
    }
    
   
    /**RSSI[7:0]**/
   /****0Bh*****R E A D C H A N*******************2-3******/
    uint16_t SI4703_GetREADCHAN(void) {
      /*high byte 0,1*/
      uint16_t z =  si4703ReadBuffer[2] & 0x03;
      z <<= 8;
      /*low byte 0..7*/
      z |=  si4703ReadBuffer[3];
      return z;
    }
    
    uint8_t SI4703_GetBLERD (void) {
    /*high*/
      return ( si4703ReadBuffer[2] & 0x0C) >> 2;
    }
    uint8_t SI4703_GetBLERC (void) {
    /*high*/
      return ( si4703ReadBuffer[2] & 0x30) >> 4;
    }
    uint8_t SI4703_GetBLERB (void) {
    /*high*/
      return ( si4703ReadBuffer[2] & 0xC0) >> 6;
    }
    
    /*******BLERB[1:0]2,3 BLERC[1:0]2,3 BLERD[1:0]2,3 READCHAN[9:0]**/
    /***0Ch****R D S A***4-5*************/
    /*****0Dh****R D S B***6-7****/
    /****0Eh*****R D S C**8-9******/
    /***0Fh******R D S D***10-11*****/
    /****00h*******D E V I C E I D*******12-13******/
   /****PN[3:0] MFGID[11:0]** */
    uint16_t SI4703_GetPN (void) {
      /*high*/
      return ( si4703ReadBuffer[12] & 0xF0) >> 4;
    }
    
    uint16_t SI4703_GetMFGID (void) {
      /*high*/
      uint16_t res =  si4703ReadBuffer[12] & 0x0F;
      res <<= 8;
      /*low*/
      res |=  si4703ReadBuffer[13];
      return res;
    }
   /****01h******C H I P I D*******************14-15--*/
    uint16_t SI4703_GetFIRMWARE (void) {
      /*low*/
      return  si4703ReadBuffer[15] & 0x3F;
    }
    uint16_t SI4703_GetRevision (void) {
      /*high*/
      return ( si4703ReadBuffer[14] & 0xFC) >> 2;
    }
    
    uint16_t SI4703_GetDEV (void) {
      /*low*/
      uint16_t res =  si4703ReadBuffer[15] >> 6;
      /*high*/
      res |= ( si4703ReadBuffer[14] & 0x03) << 2;
      return res;
    }
    
    /* REV[5:0] DEV[3:0] FIRMWARE[5:0]*/
    uint16_t SI4703_Init(void){
      uint16_t error;
      //reading for save defaults in  0x07h - it will be in [26,27]
      SI4703_readFromChip();
      //aplly the data tht has been written from 0x07
      //0x07: highh 8bits
      si4703WriteBuffer[10] = si4703ReadBuffer[26];
      //0x07: low 8bits
      si4703WriteBuffer[11] = si4703ReadBuffer[27];
      //0x08: high
      si4703WriteBuffer[12] = si4703ReadBuffer[28];
      //0x08: low
      si4703WriteBuffer[13] = si4703ReadBuffer[29];
      //0x09 high:
      si4703WriteBuffer[14] = si4703ReadBuffer[30];
      //0x09 low
      si4703WriteBuffer[15] = si4703ReadBuffer[31];
      HAL_Delay(4);
      SI4703_SetAtomicTEST1(SI4703_07_XOSCEN);
      HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,12,10);
      HAL_Delay(1000);
      error = HAL_I2C_GetError(&hi2c1);
      if(error != 0) {
        /*if transmission error hasbeen happened*/
        return error;
      }
      SI4703_SetAtomicPOWERCFGR( SI4703_02_DMUTE| SI4703_02_ENABLE);
       HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,2,10);
       HAL_Delay(150);
       //clear volext
       si4703WriteBuffer[8] = 0;
       si4703WriteBuffer[9] = 0;
        HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,8,10);
        HAL_Delay(50);
       error = HAL_I2C_GetError(&hi2c1);
       SI4703_readFromChip();
      HAL_Delay(4);
      return error;
    }
    
    uint16_t SI4703_readFromChip(void) {
      uint16_t error;
       HAL_I2C_Master_Receive(&hi2c1,0x10<<1,si4703ReadBuffer,32,10);
       HAL_Delay(4);
       error = HAL_I2C_GetError(&hi2c1);
       return error;
    }
    
    uint16_t SI4703_sendVolume(char vol) {
       uint16_t error;
       vol &=  0x0f;
       SI4703_SetVOLUME(vol);
       HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,8,10);
       HAL_Delay(4);
       error = HAL_I2C_GetError(&hi2c1);
      return error;
    }
    
     uint16_t SI4703_WriteShort(void) {
       uint16_t error;
       HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,8,10);
       HAL_Delay(4);
       error = HAL_I2C_GetError(&hi2c1);
       return error; 
  
    }
    uint16_t SI4703_writeToChip(void) {
     uint16_t error;
     HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,10,10);
     HAL_Delay(4);
     error = HAL_I2C_GetError(&hi2c1);
      return error;
    }
    
    uint16_t SI4703_powerDown(void) {
      //Clear the DMUTE bit to enable mute. 
      SI4703_SetAtomicPOWERCFGR( SI4703_02_DISABLE);
      HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,2,10);
      return 0;
    }
    
    uint16_t SI4703_channelSelecton(uint16_t channel) {
      uint16_t error;
      /*clear tune flag and write channel*/
      SI4703_SetAtomicCHANNEL(0);
      SI4703_SetCHANNEL(channel);
      /*transmit POWERCFGR-2h and CHANNEL-03h*/
       HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,8,10);
       HAL_Delay(4);
       /*set tune flag ant put it again to the chip*/
      SI4703_SetAtomicCHANNEL(SI4703_03_TUNE);
      HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,8,10);
       HAL_Delay(60);
      /*read flag - check that  tune has been completed^ wait until set STC*/  
      while(( si4703ReadBuffer[0] & 0x40) == 0) {
        /*reading 0Ah*/
           HAL_I2C_Master_Receive(&hi2c1,0x10<<1, si4703ReadBuffer,2,10);
           HAL_Delay(10);
      }
      /*clear tune flag*/
      SI4703_SetAtomicCHANNEL(0);
      HAL_Delay(6);
      HAL_I2C_Master_Transmit(&hi2c1,0x10<<1,si4703WriteBuffer,8,10);
      
       HAL_Delay(4);
      /*read flag - check that  tune has been completed - the STC will been cleared*/  
      while( si4703ReadBuffer[0] == 0x40) {
        /*reading 0Ah*/
           HAL_I2C_Master_Receive(&hi2c1,0x10<<1, si4703ReadBuffer,2,10);
           HAL_Delay(10);
      }
      error = HAL_I2C_GetError(&hi2c1);
      return error;
    }
    void SI4703_02h (uint16_t bits) { 
                 //  si4703WriteBuffer[0] |= (RESET & 0x01) << 1;
                   si4703WriteBuffer[0] |= (ENABLE & 0x01);
        }
    
    void SI4703_DummyDelay(uint16_t delay){
        for(uint16_t q = 0; q < delay; q++)
        {   }
        }
