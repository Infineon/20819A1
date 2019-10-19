#ifndef __SBC_H__
#define __SBC_H__
/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor
 *  Corporation. All rights reserved. This software, including source code, documentation and  related
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection
 * (United States and foreign), United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit
 * products. Any reproduction, modification, translation, compilation,  or representation of this
 * Software except as specified above is prohibited without the express written permission of
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to
 * the Software without notice. Cypress does not assume any liability arising out of the application
 * or use of the Software or any product or circuit  described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or failure of the
 * Cypress product may reasonably be expected to result  in significant property damage, injury
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees
 * to indemnify Cypress against all liability.
 */

/****************************************************************
*
*    File Name: sbc.h
*
*    Abstract: This header file publishes the A2DP SBC common
*    API and data structures
*
*
*****************************************************************/

/*******************************************************************
 *   Macro
 *******************************************************************/

/* SBC Sampling Frequency */
#define SBC_SAMP_FREQ_16000         0
#define SBC_SAMP_FREQ_32000         1
#define SBC_SAMP_FREQ_44100         2
#define SBC_SAMP_FREQ_48000         3

/* SBC Block Size */
#define SBC_BLOCK_SIZE_04           4
#define SBC_BLOCK_SIZE_08           8
#define SBC_BLOCK_SIZE_12           12
#define SBC_BLOCK_SIZE_16           16
#define SBC_BLOCK_SIZE_RESERVE           0
#define SBC_BLOCK_SIZE_10           10
#define SBC_BLOCK_SIZE_15           15
#define SBC_BLOCK_SIZE_20           20

/* SBC Channel Mode */
#define SBC_CH_MODE_MONO            0
#define SBC_CH_MODE_DUAL            1
#define SBC_CH_MODE_STEREO          2
#define SBC_CH_MODE_JOINT_STEREO    3

/* SBC Allocation Method */
#define SBC_ALLOC_METHOD_LOUDNESS   0
#define SBC_ALLOC_METHOD_SNR        1

/* SBC Number of Subbands */
#define SBC_NUM_SUBBANDS_4          4
#define SBC_NUM_SUBBANDS_8          8

/* PCM format */
#define SBC_PCM_FORMAT_R1L1         0
#define SBC_PCM_FORMAT_LMRM         1
#define SBC_PCM_FORMAT_MONO         2

/*******************************************************************
 *   Data Structures
 *******************************************************************/
typedef struct
{
    unsigned char syncword;
    unsigned char sampling_frequency:2;
    unsigned char blocks:2;
    unsigned char channel_mode:2;
    unsigned char allocation_method:1;
    unsigned char subbands:1;
    unsigned char bitpool;
    unsigned char crc_check;
    unsigned char join;
}SBC_FRAME_HEADER;

/*******************************************************************
 *   Function Prototypes
 *******************************************************************/


#endif // #ifndef __SBC_H__
