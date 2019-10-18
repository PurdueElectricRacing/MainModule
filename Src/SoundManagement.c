/**
  ******************************************************************************
  * @file    SoundManagement.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    16-January-2014
  * @brief   This file is really awesome.
  ******************************************************************************
  * @attention
  *
  * Licensed under the church of Tyler Marchok
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32072B_EVAL_Demo
  * @{
  */

/** @defgroup WAVEPLAYER
  * @{
  */

#ifdef AUDIO_SUBDEMO

/** @defgroup WAVEPLAYER_Private_Types
  * @{
  */
/**
  * @}
  */

/** @defgroup WAVEPLAYER_Private_Defines
  * @{
  */

#define MESSAGE5P  "     Now Playing    "
#define MESSAGE6P  "       Paused       "

#define MESSAGE3P  "SEL |DWN|LF/TK(0)|RG/TK(1)"
#define MESSAGE4P  "PAUS|STP|  BWD   |  FWD   "
#define MESSAGE4R  "PLAY|STP|  BWD   |  FWD   "

#define MESSAGE7P  "To play wav,use TouchKeys "
#define MESSAGE8P  "or use Up/Down, Press SEL "

/**
  * @}
  */

/** @defgroup WAVEPLAYER_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup WAVEPLAYER_Private_Variables
  * @{
  */
WAVE_FormatTypeDef WAVE_Format;
static __IO ErrorCode WaveFileStatus = Unvalid_RIFF_ID;
uint16_t TIM6ARRValue = 6000;
__IO uint32_t WaveDataLength = 0;
extern __IO uint32_t WaveCounter;
static __IO uint32_t SpeechDataOffset = 0x00;
static uint32_t wavelen = 0;
extern uint32_t Buffer1[BUFFER_SIZE_WORD];
uint32_t Buffer2[BUFFER_SIZE_WORD]= {0};
extern FATFS fs;
extern DIR dir;
extern char* DirectoryFiles[MAX_FILES];
extern __IO uint8_t NumberOfFiles;
extern uint32_t bmplen;
extern WavePlayList PlayList[MAX_FILES];
extern FIL F;
extern UINT BytesWritten;
extern UINT BytesRead;
__IO uint32_t BufferSelection = 1;
__IO uint32_t BufferRead = 0;

/**
  * @}
  */

/** @defgroup WAVEPLAYER_Private_FunctionPrototypes
  * @{
  */

static ErrorCode WavePlayer_WaveParsing(const char* WavName, uint32_t *FileLen);
static ErrorCode Get_WaveFileStatus(void);
static void ToggleBufferSign(uint32_t* pBuffer, uint32_t BufferSize);
static void Audio_MAL_Play(uint32_t Addr, uint32_t Size);
/**
  * @}
  */

/** @defgroup WAVEPLAYER_Private_Functions
  * @{
  */

/**
  * @brief  Plays wave files stored on the SDcard.
  * @param  None
  * @retval None
  */
void Menu_WavePlayerFunc(void)
{
  uint32_t str[20], pressedkey = 0, pSelectMedia = 0, CountMedia = 0;
  int32_t index = 0;
  int32_t SelectMedia = 0;

  /* Clear LCD */
  LCD_Clear(LCD_COLOR_WHITE);

  Demo_IntExtOnOffCmd(DISABLE);

  while (Menu_ReadKey() != NOKEY)
  {}

  for (index = 0; index < MAX_FILES; index++)
  {
    DirectoryFiles[index] = malloc(13); /* Initialize the DirectoryFiles pointers (heap) */
  }

  /* Get the .WAV file names on root directory  Maximum 10 files */
  NumberOfFiles = Storage_GetDirectoryWaveFiles ("/USER", DirectoryFiles);
  NumberOfFiles = NumberOfFiles + Storage_GetDirectoryWaveFiles ("/Rec", DirectoryFiles + NumberOfFiles);

  if(NumberOfFiles == 0)
  {
    for (index = 0; index < MAX_FILES; index++)
    {
      free(DirectoryFiles[index]);
    }
    LCD_Clear(LCD_COLOR_WHITE);
    /* Set the Back Color */
    LCD_SetBackColor(LCD_COLOR_BLACK);
    /* Set the Text Color */
    LCD_SetTextColor(LCD_COLOR_WHITE);
    LCD_DisplayStringLine(LCD_LINE_0,(uint8_t*) "No wave files...    ");
    LCD_DisplayStringLine(LCD_LINE_1,(uint8_t*) "Exit:  Push JoyStick");
    while (Menu_ReadKey() == NOKEY)
    {}
    LCD_Clear(LCD_COLOR_WHITE);
    Menu_DisplayMenu();

    Demo_IntExtOnOffCmd(ENABLE);
    return;
  }

  LCD_Clear(LCD_COLOR_BLACK);
  /* Display Image */
  f_mount(&fs, (TCHAR const*)"", 0);
  LCD_SetDisplayWindow(130, 310, 64, 64);
  Storage_OpenReadFile(130, 310, "STFILES/Music2.BMP");
  LCD_WindowModeDisable();

  /* Set the LCD Back Color */
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  /* Display the Titles */
  LCD_DisplayStringLine(LCD_LINE_0,(uint8_t*) "     Wave Player    ");
  index = 0;
  LCD_SetBackColor(LCD_COLOR_BLUE);
  LCD_SetTextColor(LCD_COLOR_BLACK);

  LCD_DrawFullRect(32,182, 165, 180);

  LCD_SetBackColor(LCD_COLOR_WHITE);

  LCD_DrawFullRect(35,179, 158, 174);

  /* Load all files name in PlayList table */
  for(index = 0 ; index < NumberOfFiles ;index++)
  {
    /* Format the string */
    sprintf ((char*)str, "%-13.13s", DirectoryFiles[index]);
    strcpy ((char *)&PlayList[index].wavelist[0], (char *)str);
  }

  SelectMedia = 0;
  Media_List_Display(PlayList, NumberOfFiles, SelectMedia);

  /* Displays MESSAGE1 on line 5 */
  LCD_SetFont(&Font12x12);
  /* Set the LCD Back Color */
  LCD_SetBackColor(LCD_COLOR_MAGENTA);
  /* Set the LCD Text Color */
  LCD_SetTextColor(LCD_COLOR_BLACK);
  LCD_DisplayStringLine(LINE(18), (uint8_t *)MESSAGE7P);
  LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE8P);
  LCD_SetFont(&Font16x24);

  /* Init STMTouch driver */
  TSL_user_Init();

  /* Initialize variables */
  CountMedia = 0;
  pressedkey = NOKEY;

  while (SelectMedia < NumberOfFiles)
  {
    pressedkey = Menu_ReadKey();

    /* Execute STMTouch Driver state machine */
    if(TSL_user_Action() == TSL_STATUS_OK)
    {
      if(TEST_TKEY(0))
      {
        if(SelectMedia == (NumberOfFiles - 1))
        {
          SelectMedia = -1;
        }
        SelectMedia++;
      }
      else if (TEST_TKEY(1))
      {
        if(SelectMedia == 0)
        {
          SelectMedia = NumberOfFiles;
        }
        SelectMedia--;
      }
      else
      {
        pSelectMedia = SelectMedia;

        if(CountMedia == 10)
        {
          pressedkey = SEL;
        }
      }

      /* Display the playlist */
      Media_List_Display(PlayList, NumberOfFiles, SelectMedia);

      if((pSelectMedia == SelectMedia) && (MyTKeys[0].p_Data->StateId) != TSL_STATEID_RELEASE)
      {
        CountMedia++;
      }
      else if ((pSelectMedia == SelectMedia) && (MyTKeys[1].p_Data->StateId) != TSL_STATEID_RELEASE)
      {
         CountMedia++;
      }
    }

    if(pressedkey == UP)
    {
      if(SelectMedia == 0)
      {
        SelectMedia = NumberOfFiles;
      }
      SelectMedia--;

      /* Display the playlist */
      Media_List_Display(PlayList, NumberOfFiles, SelectMedia);

    }
    else if (pressedkey == DOWN)
    {
      if(SelectMedia == (NumberOfFiles - 1))
      {
        SelectMedia = -1;
      }
      SelectMedia++;

      /* Display the playlist */
      Media_List_Display(PlayList, NumberOfFiles, SelectMedia);
    }
    else if (pressedkey == RIGHT)
    {
      SelectMedia = NumberOfFiles;
    }

    if(pressedkey == SEL)
    {
      /* Display Image */
      LCD_Clear(LCD_COLOR_BLACK);
      /* Set the LCD Back Color */
      LCD_SetBackColor(Black);
      LCD_SetTextColor(White);
      /* Display the Titles */
      LCD_DisplayStringLine(LCD_LINE_0,(uint8_t*) "     Wave Player    ");
      /** Format the string */
      sprintf ((char*)str, "     ../%-13.13s", DirectoryFiles[SelectMedia]);
      LCD_SetBackColor(Black);
      LCD_SetTextColor(Magenta);
      LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *) str);
      LCD_SetFont(&Font16x24);

      LCD_SetDisplayWindow(130, 310, 64, 64);
      Storage_OpenReadFile(130, 310, "STFILES/Play2.BMP");
      LCD_WindowModeDisable();
      LCD_SetFont(&Font16x24);

      /* Set the LCD Back Color */
      LCD_SetBackColor(Black);
      LCD_SetTextColor(Magenta);
      /* Display the Titles */
      LCD_DisplayStringLine(LCD_LINE_7,(uint8_t*)  MESSAGE5P);
      /* Displays MESSAGE1 on line 5 */
      LCD_SetFont(&Font12x12);
      /* Set the LCD Back Color */
      LCD_SetBackColor(LCD_COLOR_CYAN);
      /* Set the LCD Text Color */
      LCD_SetTextColor(LCD_COLOR_BLACK);
      LCD_DisplayStringLine(LINE(18), (uint8_t *)MESSAGE3P);
      /* Set the LCD Back Color */
      LCD_SetBackColor(LCD_COLOR_BLUE);
      LCD_SetTextColor(LCD_COLOR_WHITE);
      LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE4P);
      LCD_SetFont(&Font16x24);
      LCD_SetBackColor(LCD_COLOR_BLACK);

      if(strcmp(DirectoryFiles[SelectMedia] , "REC_WAVE.WAV") == 0)
      {
        /* Format the string */
        sprintf ((char*)str, "/Rec/%-13.13s", DirectoryFiles[SelectMedia]);
        pressedkey = WaveRecPlayerMenu_Start((const char*)str, &bmplen);
      }
      else
      {
        /* Format the string */
        sprintf ((char*)str, "/USER/%-13.13s", DirectoryFiles[SelectMedia]);
        pressedkey = WavePlayerMenu_Start((const char*)str, &bmplen);
      }

      CountMedia = 0;
      pressedkey = NOKEY;

      if(pressedkey == NOKEY)
      {
        if(Get_WaveFileStatus() != Valid_WAVE_File)
        {
          LCD_Clear(LCD_COLOR_WHITE);
          /* Set the Back Color */
          LCD_SetBackColor(LCD_COLOR_BLUE);
          /* Set the Text LCD_COLOR_WHITE */
          LCD_SetTextColor(LCD_COLOR_WHITE);
          LCD_ClearLine(LCD_LINE_3);
          LCD_DisplayStringLine(LCD_LINE_3, (uint8_t *) str);
          LCD_DisplayStringLine(LCD_LINE_4,(uint8_t*) "Wave file is not    ");
          LCD_DisplayStringLine(LCD_LINE_5,(uint8_t*) "supported.          ");
          LCD_DisplayStringLine(LCD_LINE_6,(uint8_t*) "Press JoyStick to   ");
          LCD_DisplayStringLine(LCD_LINE_7,(uint8_t*) "continue.           ");
          while (Menu_ReadKey() != NOKEY)
          {}
          pressedkey = Menu_ReadKey();
          while (pressedkey == NOKEY)
          {
            pressedkey = Menu_ReadKey();
          }

          LCD_Clear(LCD_COLOR_BLACK);
          /* Display Image */
          f_mount(&fs, (TCHAR const*)"", 0);
          LCD_SetDisplayWindow(130, 310, 64, 64);
          Storage_OpenReadFile(130, 310, "STFILES/Music2.BMP");
          LCD_WindowModeDisable();

          /* Set the LCD Back Color */
          LCD_SetBackColor(Black);
          LCD_SetTextColor(White);
          /* Display the Titles */
          LCD_DisplayStringLine(LCD_LINE_0,(uint8_t*) "     Wave Player    ");

          LCD_SetBackColor(LCD_COLOR_BLUE);
          LCD_SetTextColor(LCD_COLOR_BLACK);

          LCD_DrawFullRect(32,182, 165, 180);

          LCD_SetBackColor(LCD_COLOR_WHITE);

          LCD_DrawFullRect(35,179, 158, 174);

          /* Load all files name in PlayList table */
          for(index = 0 ; index < NumberOfFiles ;index++)
          {
            /* Format the string */
            sprintf ((char*)str, "%-13.13s", DirectoryFiles[index]);
            strcpy ((char *)&PlayList[index].wavelist[0], (char *)str);
          }

          Media_List_Display(PlayList, NumberOfFiles, SelectMedia);

          /* Displays MESSAGE1 on line 5 */
          LCD_SetFont(&Font12x12);
          /* Set the LCD Back Color */
          LCD_SetBackColor(LCD_COLOR_MAGENTA);
          /* Set the LCD Text Color */
          LCD_SetTextColor(LCD_COLOR_BLACK);
          LCD_DisplayStringLine(LINE(18), (uint8_t *)MESSAGE7P);
          LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE8P);
          LCD_SetFont(&Font16x24);
        }
        else
        {
          SelectMedia = 1;
        }
      }
      pressedkey = NOKEY;
    }
  }
  for (index = 0; index < MAX_FILES; index++)
  {
    free(DirectoryFiles[index]);
  }

  LCD_Clear(LCD_COLOR_WHITE);
  Menu_DisplayMenu();

  Demo_IntExtOnOffCmd(ENABLE);
}

/**
  * @brief  Wave player Initialization
  * @param  None
  * @retval None
  */
void WavePlayer_Init(void)
{
  DAC_InitTypeDef  DAC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  /******************************* TIM6 Configuration *************************/
  TIM_DeInit(TIM6);
  /* Set the timer period */
  TIM_SetAutoreload(TIM6, TIM6ARRValue);

  /* TIM6 TRGO update selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  DAC_DeInit();

  /* GPIOA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /******************************* Init DAC channel 1 (PA4) *******************/
  /* Configure PA.04 (DAC_OUT1) in analog mode */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* Enable DAC Channel 1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  /************************** Init DAC channel 2 (PA5) ************************/
  /* Configure PA.05 (DAC_OUT2) in analog mode */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel 2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);

  /********************************* DMA Config *******************************/
  /* DMA1 clock enable (to be used with DAC) */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* DMA channel3 Configuration is managed by Audio_MAL_Play() function */

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable Transfer Complete on channel 3 */
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
}

/**
  * @brief  Returns the Wave file status.
  * @param  None
  * @retval Zero value if the function succeed, otherwise it returns a nonzero
  *         value which specifies the error code.
  */
static ErrorCode Get_WaveFileStatus(void)
{
  return (WaveFileStatus);
}

/**
  * @brief  Checks the format of the .WAV file and gets information about the audio
  *         format. This is done by reading the value of a number of parameters
  *         stored in the file header and comparing these to the values expected
  *         authenticates the format of a standard .WAV  file (44 bytes will be read).
  *         If  it is a valid .WAV file format, it continues reading the header
  *         to determine the  audio format such as the sample rate and the sampled
  *         data size. If the audio format is supported by this application, it
  *         retrieves the audio format in WAVE_Format structure and returns a zero
  *         value. Otherwise the function fails and the return value is nonzero.
  *         In this case, the return value specifies the cause of  the function
  *         fails. The error codes that can be returned by this function are declared
  *         in the header file.
  * @param  WavName: wav file name
  * @param  FileLen: wav file length
  * @retval Zero value if the function succeed, otherwise it returns a nonzero
  *         value which specifies the error code.
  */
static ErrorCode WavePlayer_WaveParsing(const char* WavName, uint32_t *FileLen)
{
  uint32_t temp = 0x00;
  uint32_t extraformatbytes = 0;
  uint16_t TempBuffer[_MAX_SS];

  f_mount(&fs, (TCHAR const*)"", 0);

  f_open(&F, WavName, FA_READ);

  f_read(&F, TempBuffer, _MAX_SS, &BytesRead);

  /* Read chunkID, must be 'RIFF'  -------------------------------------------*/
  temp = ReadUnit((uint8_t*)TempBuffer, 0, 4, BigEndian);
  if(temp != CHUNK_ID)
  {
    return(Unvalid_RIFF_ID);
  }

  /* Read the file length ----------------------------------------------------*/
  WAVE_Format.RIFFchunksize = ReadUnit((uint8_t*)TempBuffer, 4, 4, LittleEndian);

  /* Read the file format, must be 'WAVE' ------------------------------------*/
  temp = ReadUnit((uint8_t*)TempBuffer, 8, 4, BigEndian);
  if(temp != FILE_FORMAT)
  {
    return(Unvalid_WAVE_Format);
  }

  /* Read the format chunk, must be'fmt ' ------------------------------------*/
  temp = ReadUnit((uint8_t*)TempBuffer, 12, 4, BigEndian);
  if(temp != FORMAT_ID)
  {
    return(Unvalid_FormatChunk_ID);
  }
  /* Read the length of the 'fmt' data, must be 0x10 -------------------------*/
  temp = ReadUnit((uint8_t*)TempBuffer, 16, 4, LittleEndian);
  if(temp != 0x10)
  {
    extraformatbytes = 1;
  }
  /* Read the audio format, must be 0x01 (PCM) -------------------------------*/
  WAVE_Format.FormatTag = ReadUnit((uint8_t*)TempBuffer, 20, 2, LittleEndian);
  if(WAVE_Format.FormatTag != WAVE_FORMAT_PCM)
  {
    return(Unsupporetd_FormatTag);
  }

  /* Read the number of channels, must be 0x02 (Stereo) ----------------------*/
  WAVE_Format.NumChannels = ReadUnit((uint8_t*)TempBuffer, 22, 2, LittleEndian);

  if(WAVE_Format.NumChannels != CHANNEL_STEREO)
  {
    return(Unsupporetd_Number_Of_Channel);
  }

  /* Read the Sample Rate ----------------------------------------------------*/
  WAVE_Format.SampleRate = ReadUnit((uint8_t*)TempBuffer, 24, 4, LittleEndian);
  /* Update the OCA value according to the .WAV file Sample Rate */
  switch (WAVE_Format.SampleRate)
  {
  case SAMPLE_RATE_8000 :
    TIM6ARRValue = 6000;
    break; /* 8KHz = 48MHz / 6000 */
  case SAMPLE_RATE_16000 :
    TIM6ARRValue = 3000;
    break; /* 8KHz = 48MHz / 6000 */
  case SAMPLE_RATE_11025:
    TIM6ARRValue = 4353;
    break; /* 11.025KHz = 48MHz / 4353 */
  case SAMPLE_RATE_22050:
    TIM6ARRValue = 2176;
    break; /* 22.05KHz = 48MHz / 2176 */
  case SAMPLE_RATE_32000:
    TIM6ARRValue = 1500;
    break; /* 32KHz = 48MHz / 1500 */
  case SAMPLE_RATE_44100:
    TIM6ARRValue = 1088;
    break; /* 44.1KHz = 48MHz / 1088 */

  default:
    return(Unsupporetd_Sample_Rate);
  }

  /* Read the Byte Rate ------------------------------------------------------*/
  WAVE_Format.ByteRate = ReadUnit((uint8_t*)TempBuffer, 28, 4, LittleEndian);

  /* Read the block alignment ------------------------------------------------*/
  WAVE_Format.BlockAlign = ReadUnit((uint8_t*)TempBuffer, 32, 2, LittleEndian);

  /* Read the number of bits per sample --------------------------------------*/
  WAVE_Format.BitsPerSample = ReadUnit((uint8_t*)TempBuffer, 34, 2, LittleEndian);

  if(WAVE_Format.BitsPerSample != BITS_PER_SAMPLE_16)
  {
    return(Unsupporetd_Bits_Per_Sample);
  }
  SpeechDataOffset = 36;
  /* If there is Extra format bytes, these bytes will be defined in "Fact Chunk" */
  if(extraformatbytes == 1)
  {
    /* Read th Extra format bytes, must be 0x00 ------------------------------*/
    temp = ReadUnit((uint8_t*)TempBuffer, 36, 2, LittleEndian);
    if(temp != 0x00)
    {
      return(Unsupporetd_ExtraFormatBytes);
    }
    /* Read the Fact chunk, must be 'fact' -----------------------------------*/
    temp = ReadUnit((uint8_t*)TempBuffer, 38, 4, BigEndian);
    if(temp != FACT_ID)
    {
      return(Unvalid_FactChunk_ID);
    }
    /* Read Fact chunk data Size ---------------------------------------------*/
    temp = ReadUnit((uint8_t*)TempBuffer, 42, 4, LittleEndian);

    SpeechDataOffset += 10 + temp;
  }
  /* Read the Data chunk, must be 'data' -------------------------------------*/
  temp = ReadUnit((uint8_t*)TempBuffer, SpeechDataOffset, 4, BigEndian);
  SpeechDataOffset += 4;
  if(temp != DATA_ID)
  {
    return(Unvalid_DataChunk_ID);
  }

  /* Read the number of sample data ------------------------------------------*/
  WAVE_Format.DataSize = ReadUnit((uint8_t*)TempBuffer, SpeechDataOffset, 4, LittleEndian);
  SpeechDataOffset += 4;
  WaveCounter =  SpeechDataOffset;
  return(Valid_WAVE_File);
}

/**
  * @brief  Starts playing wave.
  * @param  FileName: pointer to the wave file name to be read
  * @param  FileLen: pointer to the wave file length to be read
  * @retval None
  */
uint8_t WavePlayerMenu_Start(const char* FileName, uint32_t *FileLen)
{
  uint32_t tmp, KeyState = NOKEY;
  int32_t SelectMedia = 0;

  WaveFileStatus = Unvalid_RIFF_ID;

  /* Read the Speech wave file status */
  WaveFileStatus = WavePlayer_WaveParsing((const char*)FileName, &wavelen);

  if(WaveFileStatus == Valid_WAVE_File)  /* the .WAV file is valid */
  {
    /* Set WaveDataLenght to the Speech wave length */
    WaveDataLength = WAVE_Format.DataSize;
  }
  else
  {
    return NOKEY;
  }

  WavePlayer_Init();

  /* Jump to wave data */
  f_lseek(&F, SpeechDataOffset);


  /* Store data in buffer 2 */
  f_read(&F, Buffer1, BUFFER_SIZE_BYTE, &BytesRead);

  /* Convert signed to unsigned format */
  ToggleBufferSign(Buffer1, BUFFER_SIZE_WORD);

  /* Play buffer 1 */
  Audio_MAL_Play((uint32_t)Buffer1, BUFFER_SIZE_WORD);

  /* Store data in buffer 2 */
  f_read(&F, Buffer2, BUFFER_SIZE_BYTE, &BytesRead);

  /* Invert sign bit: PCM format is 16-bit signed and DAC is 12-bit unsgined */
  ToggleBufferSign(Buffer2, BUFFER_SIZE_WORD);


  BufferSelection = 2;
  WaveDataLength = WAVE_Format.DataSize;
  /* while not end of file */
  while (F.fptr < (F.fsize - 2*BUFFER_SIZE_BYTE))
  {
    if(BufferRead != 0)
    {
      if(BufferSelection == 2)
      {
        /* Play buffer 2 */
        Audio_MAL_Play((uint32_t)Buffer2, BUFFER_SIZE_WORD);
        /* Store data in buffer 1 */
        if(f_read(&F, Buffer1, BUFFER_SIZE_BYTE, &BytesRead) != FR_OK)
        {
          while(1);
        }

        /* Convert signed to unsigned format */
        ToggleBufferSign(Buffer1, BUFFER_SIZE_WORD);

        BufferRead = 0;
      }
      else if (BufferSelection == 1)
      {
        /* Play buffer 1 */
        Audio_MAL_Play((uint32_t)Buffer1, BUFFER_SIZE_WORD);
        /* Store data in buffer 2 */
        if(f_read(&F, Buffer2, BUFFER_SIZE_BYTE, &BytesRead) != FR_OK)
        {
          while(1);
        }

        /* Convert signed to unsigned format */
        ToggleBufferSign(Buffer2, BUFFER_SIZE_WORD);

        BufferRead = 0;
      }
      WaveDataLength -= BUFFER_SIZE_BYTE;
      tmp = (uint8_t) ((uint32_t)((WAVE_Format.DataSize - WaveDataLength) * 100) / WAVE_Format.DataSize);
    }

    KeyState = LCD_Update();
    if(KeyState == DOWN)
    {
      return DOWN;
    }
  }

  DMA1_Channel3->CCR = 0x0;

  /* Disable TIM6 */
  TIM_Cmd(TIM6, DISABLE);

  WaveDataLength = 0;

  return NOKEY;
}

/**
  * @brief  Start wave playing
  * @param  None
  * @retval None
  */
uint8_t WaveRecPlayerMenu_Start(const char* FileName, uint32_t *FileLen)
{
  uint32_t tmp, KeyState = NOKEY;
  int32_t SelectMedia = 0;

  WaveFileStatus = Unvalid_RIFF_ID;

  /* Read the Speech wave file status */
  WaveFileStatus = WavePlayer_WaveParsing((const char*)FileName, &wavelen);

  /* Check if the .WAV file is valid */
  if(WaveFileStatus == Valid_WAVE_File)
  {
    /* Set WaveDataLenght to the Speech wave length */
    WaveDataLength = WAVE_Format.DataSize;
  }
  else
  {
    return NOKEY;
  }

  WavePlayer_Init();

  /* Jump to wave data */
  f_lseek(&F, SpeechDataOffset);

  LCD_SetTextColor(LCD_COLOR_MAGENTA);
  /* Set the Back Color */
  LCD_SetBackColor(LCD_COLOR_BLUE);
  LCD_DrawRect(LCD_LINE_8, 310, 16, 300);

  /* Store data in buffer 2 */
  f_read(&F, Buffer1, BUFFER_SIZE_BYTE, &BytesRead);

  /* Play buffer 1 */
  Audio_MAL_Play((uint32_t)Buffer1, BUFFER_SIZE_WORD);

  /* Store data in buffer 2 */
  f_read(&F, Buffer2, BUFFER_SIZE_BYTE, &BytesRead);

  BufferSelection = 2;
  WaveDataLength = WAVE_Format.DataSize;
  /* while not end of file */
  while (F.fptr < (F.fsize - 2*BUFFER_SIZE_BYTE))
  {
    if(BufferRead != 0)
    {
      if(BufferSelection == 2)
      {
        /* Play buffer 2 */
        Audio_MAL_Play((uint32_t)Buffer2, BUFFER_SIZE_WORD);
        /* Store data in buffer 1 */
        if(f_read(&F, Buffer1, BUFFER_SIZE_BYTE, &BytesRead) != FR_OK)
        {
          while(1);
        }

        BufferRead = 0;
      }
      else if(BufferSelection == 1)
      {
        /* Play buffer 1 */
        Audio_MAL_Play((uint32_t)Buffer1, BUFFER_SIZE_WORD);
        /* Store data in buffer 2 */
        if(f_read(&F, Buffer2, BUFFER_SIZE_BYTE, &BytesRead) != FR_OK)
        {
          while(1);
        }

        BufferRead = 0;
      }
      WaveDataLength -= BUFFER_SIZE_BYTE;
      tmp = (uint8_t) ((uint32_t)((WAVE_Format.DataSize - WaveDataLength) * 100) / WAVE_Format.DataSize);
      LCD_SetTextColor(LCD_COLOR_MAGENTA);
      LCD_DrawLine(LCD_LINE_8, 310 - ((tmp) * 3), 16, LCD_DIR_VERTICAL);
    }

    KeyState = LCD_Update();
    if(KeyState == DOWN)
    {
      return DOWN;
    }
  }

  DMA1_Channel3->CCR = 0x0;

  /* Disable TIM6 */
  TIM_Cmd(TIM6, DISABLE);

  WaveDataLength = 0;

  return NOKEY;
}

/**
  * @brief  DMA transfer complete
  * @param  None
  * @retval None
  */
void WavePlayer_DMATxComplete_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA1_IT_TC3))
  {
    if(BufferSelection == 2)
    {
      BufferSelection = 1;
      BufferRead = 1;
    }
    else
    {
      BufferSelection = 2;
      BufferRead = 1;
    }
   DMA_ClearITPendingBit(DMA1_IT_TC3);
  }
}

/**
  * @brief  Reads a number of bytes from the SPI Flash and reorder them in Big
  *         or little endian.
  * @param  NbrOfBytes: number of bytes to read.
  *         This parameter must be a number between 1 and 4.
  * @param  ReadAddr: external memory address to read from.
  * @param  Endians: specifies the bytes endianness.
  *         This parameter can be one of the following values:
  *             - LittleEndian
  *             - BigEndian
  * @retval Bytes read from the SPI Flash.
  */
uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat)
{
  uint32_t index = 0;
  uint32_t temp = 0;

  for (index = 0; index < NbrOfBytes; index++)
  {
    temp |= buffer[idx + index] << (index * 8);
  }

  if(BytesFormat == BigEndian)
  {
    temp = __REV(temp);
  }
  return temp;
}

/**
  * @brief  Controls the wave player application LCD display messages.
  * @param  None
  * @retval Returns the key state.
  */
uint8_t LCD_Update(void)
{
  uint32_t KeyState = NOKEY;
  uint32_t tmp = 0, index = 0;
  int32_t SelectMedia = 0;

  KeyState = Menu_ReadKey();

  if(KeyState == SEL)
  {
    DMA1_Channel3->CCR = 0x0;

    /* Disable TIM6 */
    TIM_Cmd(TIM6, DISABLE);

    /* Configure Font*/
    LCD_SetFont(&Font12x12);
    /* Set the LCD Back Color */
    LCD_SetBackColor(LCD_COLOR_BLUE);
    LCD_SetTextColor(LCD_COLOR_WHITE);
    /* Display Replay message */
    LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE4R);

    LCD_SetFont(&Font16x24);

    /* Set the LCD Back Color */
    LCD_SetBackColor(Black);
    LCD_SetTextColor(Magenta);
    /* Display the Titles */
    LCD_DisplayStringLine(LCD_LINE_7,(uint8_t*) MESSAGE6P);

    KeyState = Menu_ReadKey();

    while ((KeyState != SEL) && (KeyState != DOWN))
    {
      KeyState = Menu_ReadKey();
    }
    if(KeyState == SEL)
    {
      LCD_SetFont(&Font12x12);
      /* Set the LCD Back Color */
      LCD_SetBackColor(LCD_COLOR_BLUE);
      LCD_SetTextColor(LCD_COLOR_WHITE);
      LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE4P);
      LCD_WindowModeDisable();
      LCD_SetFont(&Font16x24);

      /* Set the LCD Back Color */
      LCD_SetBackColor(Black);
      LCD_SetTextColor(Magenta);
      /* Display the Titles */
      LCD_DisplayStringLine(LCD_LINE_7,(uint8_t*) MESSAGE5P);

      /* Enable Transfer Complete on channel 3 */
      DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
      /* Enable TIM6 */
      TIM_Cmd(TIM6, ENABLE);

      return KeyState;
    }
    else if (KeyState == DOWN)
    {
      LCD_Clear(LCD_COLOR_BLACK);
      /* Display Image */
      f_mount(&fs, (TCHAR const*)"", 0);
      LCD_SetDisplayWindow(130, 310, 64, 64);
      Storage_OpenReadFile(130, 310, "STFILES/Music2.BMP");
      LCD_WindowModeDisable();

      /* Set the LCD Back Color */
      LCD_SetBackColor(Black);
      LCD_SetTextColor(White);
      /* Display the Titles */
      LCD_DisplayStringLine(LCD_LINE_0,(uint8_t*) "     Wave Player    ");
      LCD_SetBackColor(LCD_COLOR_BLUE);
      LCD_SetTextColor(LCD_COLOR_BLACK);

      LCD_DrawFullRect(32,182, 165, 180);

      LCD_SetBackColor(LCD_COLOR_WHITE);

      LCD_DrawFullRect(35,179, 158, 174);

      SelectMedia = 0;
      Media_List_Display(PlayList, NumberOfFiles, SelectMedia);

      /* Displays MESSAGE1 on line 5 */
      LCD_SetFont(&Font12x12);
      /* Set the LCD Back Color */
      LCD_SetBackColor(LCD_COLOR_MAGENTA);
      /* Set the LCD Text Color */
      LCD_SetTextColor(LCD_COLOR_BLACK);
      LCD_DisplayStringLine(LINE(18), (uint8_t *)MESSAGE7P);
      LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE8P);
      LCD_SetFont(&Font16x24);

      return KeyState;
    }
  }
  else if (KeyState == DOWN)
  {
      WaveDataLength = 0;
    LCD_Clear(LCD_COLOR_BLACK);
    /* Display Image */
    f_mount(&fs, (TCHAR const*)"", 0);
    LCD_SetDisplayWindow(130, 310, 64, 64);
    Storage_OpenReadFile(130, 310, "STFILES/Music2.BMP");
    LCD_WindowModeDisable();

    /* Set the LCD Back Color */
    LCD_SetBackColor(Black);
    LCD_SetTextColor(White);
    /* Display the Titles */
    LCD_DisplayStringLine(LCD_LINE_0,(uint8_t*) "     Wave Player    ");
    LCD_SetBackColor(LCD_COLOR_BLUE);
    LCD_SetTextColor(LCD_COLOR_BLACK);

    LCD_DrawFullRect(32,182, 165, 180);

    LCD_SetBackColor(LCD_COLOR_WHITE);

    LCD_DrawFullRect(35,179, 158, 174);

    SelectMedia = 0;
    Media_List_Display(PlayList, NumberOfFiles, SelectMedia);

    /* Displays MESSAGE1 on line 5 */
    LCD_SetFont(&Font12x12);
    /* Set the LCD Back Color */
    LCD_SetBackColor(LCD_COLOR_MAGENTA);
    /* Set the LCD Text Color */
    LCD_SetTextColor(LCD_COLOR_BLACK);
    LCD_DisplayStringLine(LINE(18), (uint8_t *)MESSAGE7P);
    LCD_DisplayStringLine(LINE(19), (uint8_t *)MESSAGE8P);
    LCD_SetFont(&Font16x24);

    return KeyState;
  }

  /* Execute STMTouch Driver state machine */
  if(TSL_user_Action() == TSL_STATUS_OK)
  {
    if(TEST_TKEY(1))
    {
      LCD_WriteRAM_Prepare();
      tmp = F.fptr;
      if(tmp > (F.fsize / 20))
      {
        tmp -= F.fsize / 20;
      }
      else
      {
        tmp = 0;
      }

      /* Set LCD control line(/CS) */
      LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

      f_lseek(&F, tmp);

      LCD_WriteRAM_Prepare();

      WaveDataLength += (WAVE_Format.DataSize / 20);
      if(WaveDataLength > WAVE_Format.DataSize)
      {
        WaveDataLength = WAVE_Format.DataSize;
      }
      LCD_SetBackColor(LCD_COLOR_BLACK);
      LCD_ClearLine(LCD_LINE_8);
      LCD_SetTextColor(LCD_COLOR_MAGENTA);

      /* Set the Back Color */
      LCD_SetBackColor(LCD_COLOR_BLACK);
      LCD_DrawRect(LCD_LINE_8, 310, 16, 300);
      tmp = (uint8_t) ((uint32_t)((WAVE_Format.DataSize - WaveDataLength) * 100) / WAVE_Format.DataSize);
      LCD_SetTextColor(LCD_COLOR_MAGENTA);

      for (index = 0; index < tmp; index++)
      {
        LCD_DrawLine(LCD_LINE_8, 310 - ((index) * 3), 16, LCD_DIR_VERTICAL);
      }

    }
    else  if(TEST_TKEY(0))
    {
      LCD_WriteRAM_Prepare();
      tmp = F.fptr;
      tmp += F.fsize / 20;
      if(tmp >  F.fsize)
      {
        tmp = F.fsize;
      }

      /* Set LCD control line(/CS) */
      LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

      f_lseek(&F, tmp);

      LCD_WriteRAM_Prepare();

      if(WaveDataLength < (WAVE_Format.DataSize / 20))
      {
        WaveDataLength = 0;
      }
      else
      {
        WaveDataLength -= (WAVE_Format.DataSize / 20);
      }
      LCD_SetBackColor(LCD_COLOR_BLACK);
      LCD_ClearLine(LCD_LINE_8);
      LCD_SetTextColor(LCD_COLOR_MAGENTA);
      /* Set the Back Color */
      LCD_SetBackColor(LCD_COLOR_BLACK);
      LCD_DrawRect(LCD_LINE_8, 310, 16, 300);
      tmp = (uint8_t) ((uint32_t)((WAVE_Format.DataSize - WaveDataLength) * 100) / WAVE_Format.DataSize);
      LCD_SetTextColor(LCD_COLOR_MAGENTA);
      for (index = 0; index < tmp; index++)
      {
        LCD_DrawLine(LCD_LINE_8, 310 - ((index) * 3), 16, LCD_DIR_VERTICAL);
      }
    }
  }

  if(KeyState == LEFT)
  {
    LCD_WriteRAM_Prepare();
    tmp = F.fptr;
    if(tmp > (F.fsize / 20))
    {
      tmp -= F.fsize / 20;
    }
    else
    {
      tmp = 0;
    }

    /* Set LCD control line(/CS) */
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

    f_lseek(&F, tmp);

    LCD_WriteRAM_Prepare();

    WaveDataLength += (WAVE_Format.DataSize / 20);
    if(WaveDataLength > WAVE_Format.DataSize)
    {
      WaveDataLength = WAVE_Format.DataSize;
    }
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_ClearLine(LCD_LINE_8);
    LCD_SetTextColor(LCD_COLOR_MAGENTA);

    /* Set the Back Color */
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DrawRect(LCD_LINE_8, 310, 16, 300);
    tmp = (uint8_t) ((uint32_t)((WAVE_Format.DataSize - WaveDataLength) * 100) / WAVE_Format.DataSize);
    LCD_SetTextColor(LCD_COLOR_MAGENTA);

    for (index = 0; index < tmp; index++)
    {
      LCD_DrawLine(LCD_LINE_8, 310 - ((index) * 3), 16, LCD_DIR_VERTICAL);
    }
  }
  else if (KeyState == RIGHT)
  {
    LCD_WriteRAM_Prepare();
    tmp = F.fptr;
    tmp += F.fsize / 20;
    if(tmp >  F.fsize)
    {
      tmp = F.fsize;
    }

    /* Set LCD control line(/CS) */
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

    f_lseek(&F, tmp);

    LCD_WriteRAM_Prepare();

    if(WaveDataLength < (WAVE_Format.DataSize / 20))
    {
      WaveDataLength = 0;
    }
    else
    {
      WaveDataLength -= (WAVE_Format.DataSize / 20);
    }
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_ClearLine(LCD_LINE_8);
    LCD_SetTextColor(LCD_COLOR_MAGENTA);
    /* Set the Back Color */
    LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DrawRect(LCD_LINE_8, 310, 16, 300);
    tmp = (uint8_t) ((uint32_t)((WAVE_Format.DataSize - WaveDataLength) * 100) / WAVE_Format.DataSize);
    LCD_SetTextColor(LCD_COLOR_MAGENTA);
    for (index = 0; index < tmp; index++)
    {
      LCD_DrawLine(LCD_LINE_8, 310 - ((index) * 3), 16, LCD_DIR_VERTICAL);
    }
  }
  return NOKEY ;
}

/**
  * @brief  Toggles sign bit of input buffer.
  * @param  pBuffer: pointer to the input buffer
  * @param  BufferSize: the size of the buffer in words
  * @retval None
  */
static void ToggleBufferSign(uint32_t* pBuffer, uint32_t BufferSize)
{
  uint32_t readdata = (uint32_t)pBuffer;
  uint32_t loopcounter = 0;

  /* Invert sign bit: PCM format is 16-bit signed and DAC is 12-bit unsigned */
  for(loopcounter = 0; loopcounter < BufferSize; loopcounter++)
  {
    *(uint32_t*)readdata ^= 0x80008000;
    readdata+=4;
  }
}

/**
  * @brief  Starts playing audio stream from the audio Media.
  * @param  Add: Buffer address
  * @param  Size: Buffer size
  * @retval None
  */
static void Audio_MAL_Play(uint32_t Addr, uint32_t Size)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* Enable DMA for DAC Channel 2 */
  DAC_DMACmd(DAC_Channel_1, DISABLE);
  /* Enable DMA1 Channel3 */
  DMA_Cmd(DMA1_Channel3, DISABLE);

  /* Disable TIM6 counter */
  TIM_Cmd(TIM6, DISABLE);

  /* Configure the buffer address and size */
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Addr;
  DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12LD_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_InitStructure.DMA_BufferSize = (uint32_t)Size;
  /* Configure the DMA Channel with the new parameters */
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
  /* Enable the DMA Channel */
  DMA_Cmd(DMA1_Channel3, ENABLE);

  /* Enable DMA for DAC Channel 1 */
  DAC_DMACmd(DAC_Channel_1, ENABLE);

  /* Enable TIM6 counter */
  TIM_Cmd(TIM6, ENABLE);
}

/**
  * @brief  Executed at each timer interruption (option must be enabled)
  * @param  None
  * @retval None
  */
void TSL_CallBack_TimerTick(void)
{
}

/**
  * @}
  */
#endif /* AUDIO_SUBDEMO */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
