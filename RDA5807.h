 /*******write buffer -it contais  data to be sent into SI4703*/
 /*for example: POWERCFGR_HighByte = si4703WriteBuffer[0], POWERCFGR_LowByte = si4703WriteBuffer[1]*/
 /***CHANNEL_HiByte  = si4703WriteBuffer[2], CHANNEL_LowByte  = si4703WriteBuffer[3]*/
 static unsigned char si4703WriteBuffer[16];
 /**read buffer : for example- STATUSRSSI_hi = si4703ReadBuffer[0], STATUSRSSI_low = si4703ReadBuffer[1]**/
 static unsigned char si4703ReadBuffer[32];
    /***communication with Si4703 si4703WriteBuffer[0]-upper (high) byte,
	si4703WriteBuffer[1] lower byte  e.t.c.*****/
   
  /*****02h*****P O W E R C F G************************0-1***/
    #define SI4703_02_ENABLE 1
    #define SI4703_02_DISABLE (1 << 6)
    #define SI4703_02_SEEK (1 << 8)
    #define SI4703_02_SEEKUP (1 << 9)
    #define SI4703_02_SKMODE (1 << 10)
    #define SI4703_02_RDSM (1 << 11)
    #define SI4703_02_MONO (1 << 13)
    #define SI4703_02_DMUTE (1 << 14)
    #define SI4703_02_DSMUTE (1 << 15)
    void SI4703_SetAtomicPOWERCFGR (uint16_t x);
    /***03h**C H A N N E L **************************2-3*/
    #define SI4703_03_TUNE (1 << 15)
    void SI4703_SetAtomicCHANNEL (uint16_t x);
    void SI4703_SetCHANNEL(uint16_t ch);
    /**b0-b9 - channel*/
    /**04h***S Y S C O N F I G 1*******4-5*****/
    void SI4703_SetAtomicSYSCONFIG1 (uint16_t x);
    void SI4703_SetBLNDADJ (uint8_t val);
    void SI4703_SetGPIO (uint8_t GP1, uint8_t GP2, uint8_t GP3);
    /*BLNDADJ[1:0] GPIO3[1:0] GPIO2[1:0] GPIO1[1:0]*/
    #define SI4703_04_AGCD (1 << 10)
    #define SI4703_04_DE (1 << 11)
    #define SI4703_04_RDS (1 << 12)
    #define SI4703_04_STCIEN (1 << 14)
    #define SI4703_04_RDSIEN (1 << 15)
    /****05h***************S Y S C O N F I G 2********6-7*******/
    void SI4703_SetSEEKTH (uint8_t x);
    void SI4703_SetBAND (uint8_t x);
    void SI4307_SetSPACE (uint8_t x);
    void SI4703_SetVOLUME(uint8_t x);
      /*SEEKTH[7:0] BAND[1:0] SPACE[1:0] VOLUME[3:0]*/
    /**06h**S Y S C O N F I G 3***********************8-9******/
    void SI4703_SetAtomicSYSCFGR3 (unsigned short x);
    void SI4703_SetSKSNR (uint8_t x);
    void SI4703_SetSKCNT (uint8_t x);
    
    void SI4703_SetSMUTER (uint8_t x);
    
    void SI4703_SetSMUTEA (uint8_t x);
    /*SKSNR[3:0] SKCNT[3:0] SMUTER[1:0] SMUTEA[1:0]*/
    #define SI4703_06_VOLEXT (1 << 8)
    /****07h*****T E S T 1*************************10-11*******/
     void SI4703_SetAtomicTEST1 (unsigned short x);
    #define SI4703_07_XOSCEN (1 << 15)
    #define SI4703_07_AHIZEN (1 << 14)
    
    /***08h***T E S T 2:12-13*******/
     /*****09h****B O O T C O N F I G:14-15*****/
    
    /**_____________FOR   R e A d_______***/
    /**0A***S T A T U S R S S I********************0-1**/
    unsigned char SI4703_GetRSSI (void);
    
    uint8_t SI4703_GetBLERA (void);
    uint16_t SI4703_GetAtomicSTATUSRSSI (void);
    
    #define SI4703_AH_RDSR (1 << 15)
    #define SI4703_AH_STC (1 << 14)
    #define SI4703_AH_SF_BL (1 << 13)
    #define SI4703_AH_AFCRL (1 << 12)
    #define SI4703_AH_RDSS2 (1 << 11)
    #define SI4703_AH_ST (1 << 8)
    /**RSSI[7:0]**/
   /****0b*****R E A D C H A N*******************2-3******/
    uint16_t SI4703_GetREADCHAN(void);
    
    uint8_t SI4703_GetBLERD (void);
    uint8_t SI4703_GetBLERC (void);
    uint8_t SI4703_GetBLERB (void);
    
    /**BLERB[1:0]2,3 BLERC[1:0]2,3 BLERD[1:0]2,3 READCHAN[9:0]**/
    /**0C***R D S A**4-5*****/
    /**0D*R D S B ***6-7***/
    /***0E**R D S C***8-9**/
    /****0F**R D S D****10-11*****/
    /**00h*******D E V I C E I D*******12-13******/
   /****PN[3:0] MFGID[11:0]** */
    
    uint16_t SI4703_GetPN (void);
    uint16_t SI4703_GetMFGID (void);
   /***01h*******CHIPID*******************14-15**/
    uint16_t SI4703_GetFIRMWARE (void);
    uint16_t SI4703_GetRevision (void);
    uint16_t SI4703_GetDEV (void);
    
    /* REV[5:0] DEV[3:0] FIRMWARE[5:0]*/
    void si4703_02h (uint16_t bits);
       uint16_t SI4703_Init(void);
       uint16_t SI4703_readFromChip(void);
       uint16_t SI4703_sendVolume(char vol);
       uint16_t SI4703_writeToChip(void);
       uint16_t SI4703_channelSelecton(uint16_t channel);
        uint16_t SI4703_WriteShort(void);
        
        void SI4703_DummyDelay(uint16_t delay);
         uint16_t SI4703_powerDown(void);
