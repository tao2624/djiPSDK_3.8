/**
 ********************************************************************
 * @file    hms_manager_entry.cpp
 * @brief
 *
 * @copyright (c) 2018 DJI. All rights reserved.
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

/* Includes ------------------------------------------------------------------*/
#include "hms_manager_entry.h"
#include "dji_platform.h"
#include <iostream>
#include "dji_logger.h"
#include "hms/test_hms.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
void DjiUser_RunHmsManagerSample(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    char inputSelectSample;

start:
    osalHandler->TaskSleepMs(100);

    std::cout
        << "\n"
        << "| Available commands:                                                                              |\n"
        << "| [0] Hms manager sample - Chinese language                                                        |\n"
        << "| [1] Hms manager sample - English language                                                        |\n"
        << "| [2] Hms manager sample - Japanese language                                                       |\n"
        << "| [3] Hms manager sample - French language                                                         |\n"
        << std::endl;

    std::cin >> inputSelectSample;
    switch (inputSelectSample) {
        case '0':
            DjiTest_HmsManagerRunSample(DJI_MOBILE_APP_LANGUAGE_CHINESE);
            goto start;
        case '1':
            DjiTest_HmsManagerRunSample(DJI_MOBILE_APP_LANGUAGE_ENGLISH);
            goto start;
        case '2':
            DjiTest_HmsManagerRunSample(DJI_MOBILE_APP_LANGUAGE_JAPANESE);
            goto start;
        case '3':
            DjiTest_HmsManagerRunSample(DJI_MOBILE_APP_LANGUAGE_FRENCH);
            goto start;
        case 'q':
            break;
        default:
            USER_LOG_ERROR("Input command is invalid");
            goto start;
    }
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
