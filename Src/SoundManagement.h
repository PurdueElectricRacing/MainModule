/**
  ******************************************************************************
  * @file    waveplayer.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    16-January-2014
  * @brief   This file contains all the functions prototypes for the wave player
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVEPLAYER_H
#define __WAVEPLAYER_H

{
#endif

/**
@code
  .WAV file format :
 Endian      Offset      Length      Contents
  big         0           4 bytes     'RIFF'             // 0x52494646
  little      4           4 bytes     <file length - 8>
  big         8           4 bytes     'WAVE'             // 0x57415645
Next, the fmt chunk describes the sample format:
  big         12          4 bytes     'fmt '          // 0x666D7420
  little      16          4 bytes     0x00000010      // Length of the fmt data (16 bytes)
  little      20          2 bytes     0x0001          // Format tag: 1 = PCM
  little      22          2 bytes     <channels>      // Channels: 1 = mono, 2 = stereo
  little      24          4 bytes     <sample rate>   // Samples per second: e.g., 22050
  little      28          4 bytes     <bytes/second>  // sample rate * block align
  little      32          2 bytes     <block align>   // channels * bits/sample / 8
  little      34          2 bytes     <bits/sample>   // 8 or 16
Finally, the data chunk contains the sample data:
  big         36          4 bytes     'data'        // 0x64617461
  little      40          4 bytes     <length of the data block>
  little      44          *           <sample data>
@endcode
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"


/** @addtogroup STM32072B_EVAL_Demo
  * @{
  */

/** @addtogroup WAVEPLAYER
  * @{
  */

/** @defgroup WAVEPLAYER_Exported_Types
  * @{
  */

typedef enum
{
  LittleEndian,
  BigEndian
}Endianness;

typedef struct
{
  uint32_t  RIFFchunksize;
  uint16_t  FormatTag;
  uint16_t  NumChannels;
  uint32_t  SampleRate;
  uint32_t  ByteRate;
  uint16_t  BlockAlign;
  uint16_t  BitsPerSample;
  uint32_t  DataSize;
}
WAVE_FormatTypeDef;
typedef enum
{
  Valid_WAVE_File = 0,
  Unvalid_RIFF_ID,
  Unvalid_WAVE_Format,
  Unvalid_FormatChunk_ID,
  Unsupporetd_FormatTag,
  Unsupporetd_Number_Of_Channel,
  Unsupporetd_Sample_Rate,
  Unsupporetd_Bits_Per_Sample,
  Unvalid_DataChunk_ID,
  Unsupporetd_ExtraFormatBytes,
  Unvalid_FactChunk_ID
} ErrorCode;

/**
  * @}
  */



/** @defgroup WAVEPLAYER_Exported_Constants
  * @{
  */
#define  CHUNK_ID                            0x52494646  /* correspond to the letters 'RIFF' */
#define  FILE_FORMAT                         0x57415645  /* correspond to the letters 'WAVE' */
#define  FORMAT_ID                           0x666D7420  /* correspond to the letters 'fmt ' */
#define  DATA_ID                             0x64617461  /* correspond to the letters 'data' */
#define  FACT_ID                             0x66616374  /* correspond to the letters 'fact' */
#define  WAVE_FORMAT_PCM                     0x01
#define  FORMAT_CHNUK_SIZE                   0x10
#define  CHANNEL_MONO                        0x01
#define  CHANNEL_STEREO                      0x02
#define  SAMPLE_RATE_8000                    8000
#define  SAMPLE_RATE_16000                   16000
#define  SAMPLE_RATE_11025                   11025
#define  SAMPLE_RATE_22050                   22050
#define SAMPLE_RATE_32000                    32000
#define  SAMPLE_RATE_44100                   44100
#define  SAMPLE_RATE_48000                   48000
#define  BITS_PER_SAMPLE_8                   8
#define  BITS_PER_SAMPLE_16                  16
#define  WAVE_DUMMY_BYTE                     0xA5

/* 12-bit Left alignment (dual channels) used in case of stereo and 16-bit data */
#define DAC_DHR12LD_ADDRESS     0x40007424
/* 8-bit right alignment (dual channels) used in case of stereo and 8-bit data */
#define DAC_DHR8RD_ADDRESS      0x40007428
/* 12-bit left alignment (channel1) used in case of mono and 16-bit data */
#define DAC_DHR12L1_ADDRESS     0x4000740C
/* 8-bit right alignment (channel1) used in case of mono and 8-bit data */
#define DAC_DHR8R1_ADDRESS      0x40007410
/**
  * @}
  */

/** @defgroup WAVEPLAYER_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup WAVEPLAYER_Exported_functions
  * @{
  */
void Menu_WavePlayerFunc(void);
void WavePlayer_Init(void);
uint8_t WavePlayerMenu_Start(const char* FileName, uint32_t *FileLen);
void WavePlayer_DMATxComplete_IRQHandler(void);
uint8_t LCD_Update(void);
uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat);
uint8_t WaveRecPlayerMenu_Start(const char* FileName, uint32_t *FileLen);
#ifdef __cplusplus
}
#endif

#endif /*__WAVEPLAYER_H */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
