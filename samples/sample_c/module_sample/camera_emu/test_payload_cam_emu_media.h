/**
 ********************************************************************
 * @file    test_payload_cam_emu_media.h
 * @brief   This is the header file for "test_payload_cam_media.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEST_PAYLOAD_CAM_EMU_MEDIA_H
#define TEST_PAYLOAD_CAM_EMU_MEDIA_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "dji_payload_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef enum {
    NALU_SPS                    = 0x27, /*!< Media file JPEG type. */
    NALU_PPS                    = 0x28, /*!< Media file DNG type. */
    NALU_IDR                    = 0x25, /*!< Media file MOV type. */
    NALU_SILICE                 = 0x21, /*!< Media file MP4 type. */
} E_NaluType;

typedef struct {
    E_NaluType NALUtype; /*!< Specifies the type of media file, #E_DjiCameraMediaFileType. */
    size_t startPos;
    size_t endPos;
    uint32_t frameSize; /*!< Specifies the size of media file, uint:byte. */
} T_NaluInfo;


/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode DjiTest_CameraEmuMediaStartService(void);
T_DjiReturnCode DjiTest_CameraEmuSetMediaFilePath(const char *path);
T_DjiReturnCode DjiTest_CameraMediaGetFileInfo(const char *filePath, T_DjiCameraMediaFileInfo *fileInfo);

#ifdef __cplusplus
}
#endif

#endif // TEST_PAYLOAD_CAM_EMU_MEDIA_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
