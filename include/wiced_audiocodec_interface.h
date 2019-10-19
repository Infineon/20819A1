//  *****************************************************************************
//  Copyright (c), 2009-13 Cypress Semiconductor.
//
//          ALL RIGHTS RESERVED
//
//  *****************************************************************************
// ******************************************************************************
//  File Name: wiced_hal_audiocodec_interface.h
//
//  Abstract:  This file defines the Audio Codec Interface.
//
//  History:
//  *******************************************************************************

#ifndef __WICED_AUDIOCODEC_INTERFACE_H__
#define __WICED_AUDIOCODEC_INTERFACE_H__

#include "brcm_fw_types.h"
#include "wiced.h"

/**
* \addtogroup  interfaces   Audio Codec Interface
* \ingroup     wicedbt_av
*/
/*! @{ */
/**
* Defines an audio codec interface.  Application uses this
* interface to control external audio codec activities.
*/

typedef struct
{
    void (*initialize)(uint16_t NumChannles, uint32_t SampleRate, uint16_t BitsPerSample, uint8_t gain, uint8_t boost);
    void (*init)(void);

    uint8_t (*readSR)(void);
    uint8_t (*readBPS)(void);
    uint8_t (*readPCM)(void);
    uint16_t (*readHPF)(void);
    int16_t (*readPGA)(void);

    uint8_t (*writeSR)(uint8_t p);
    uint8_t (*writeBPS)(uint8_t p);
    uint8_t (*writePCM)(uint8_t p);
    uint8_t (*writeHPF)(uint16_t h);
    uint8_t (*writePGA)(int16_t g);

    uint16_t (*regRead)(int8_t index);
    void (*regWrite)(int8_t index, uint16_t data);
} wiced_audio_codec_interface_func_tbl;

#endif
