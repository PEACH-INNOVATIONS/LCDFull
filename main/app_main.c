#include <stdio.h>
#include <string.h>
#include "app_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "PeachDefs.h"
#include "PacketParser.h"
#include "PacketProcessor.h"
#include "UartHW.h"
#include "FileSystemSD.h"
#include "FileConfig.h"
#include "MessageProcessing.h"
#include "EndPoints.h"
#include "lcd.h"
#include "GuiLvgl.h"
#include "SpiSlaveLink.h"
#include "DebugConsole.h"

#define MAIN_TASK_STACK   6144

#define PING_PONG_TIMEOUT_ms 1000

void RegisterCoachingParams(uint16_t loggerID)
{
    /* Periodic seated: sweep angle (2), sweep force (4) */
    static const uint8_t periodicSeatedIDs[] = { eSIDPer_SweepAngle, eSIDPer_SweepForceX,eSIDPer_ScullPortForceX,eSIDPer_ScullStarForceX,9,10};
    for (uint8_t si = 0; si < sizeof(periodicSeatedIDs); si++)
        AddPhysicalParameter(loggerID, periodicSeatedIDs[si], true, ePerOrAper_Periodic, NULL);

    /* Periodic non-seated: boat acceleration (3), for the Accel vs Time screen */
    AddPhysicalParameter(loggerID, eSIDPer_Acceleration, false, ePerOrAper_Periodic, NULL);

    /* Aperiodic non-seated: rating (0), avg speed (1) */
    AddPhysicalParameter(loggerID, eSIDAPer_Rating, false, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(loggerID, eSIDAPer_AvgBoatSpeed, false, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(loggerID, eSIDAPer_SweepPower, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(loggerID, eSIDAPer_ScullPowerPort, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(loggerID, eSIDAPer_ScullPowerStar, true, ePerOrAper_Aperiodic, NULL);

    /* Aperiodic seated: drive start sweep (22) and scull port/star (20/21),
     * used for the crew-wide drive-start marker on the Angle vs Time screen */
    AddPhysicalParameter(loggerID, eSIDAPer_DriveStartSweep,     true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(loggerID, eSIDAPer_DriveStartScullPort, true, ePerOrAper_Aperiodic, NULL);
    AddPhysicalParameter(loggerID, eSIDAPer_DriveStartScullStar, true, ePerOrAper_Aperiodic, NULL);

    /* Aperiodic seated: min/max angle and catch/finish slip, for the Arc Length screen */
    static const uint8_t arcLengthIDs[] = {
        eSIDAPer_MinScullAnglePort, eSIDAPer_MinScullAngleStar, eSIDAPer_MinSweepAngle,
        eSIDAPer_MaxScullAnglePort, eSIDAPer_MaxScullAngleStar, eSIDAPer_MaxSweepAngle,
        eSIDAPer_CatchSlipScullPort, eSIDAPer_CatchSlipScullStar, eSIDAPer_CatchSlipSweep,
        eSIDAPer_FinishSlipScullPort, eSIDAPer_FinishSlipScullStar, eSIDAPer_FinishSlipSweep,
    };
    for (uint8_t si = 0; si < sizeof(arcLengthIDs); si++)
        AddPhysicalParameter(loggerID, arcLengthIDs[si], true, ePerOrAper_Aperiodic, NULL);
}

static bool s_cr_ready = false;
bool GetCRReady(void) { return s_cr_ready; }

/* Coach-radio communication state machine (identical to LCD project) */
/* Deals with the actual readiness of the CD <-> CR link*/
static void CoachDisplayStateMachine(void)
{
    typedef enum {
        eState_initial,
        eState_pinging,
        eState_waiting_pong,
        eState_resetting,
        eState_pinging2,
        eState_waiting_pong2,
        eState_operating
    } eState;
    static eState state = eState_initial;
    static TickType_t ping_t;

    switch (state) {
        case eState_initial:
            state = eState_pinging;
            break;
        case eState_pinging:
            DBG("CDSM: sending ping to CR\r\n");
            SendPingTo(eEntityCoachRadio, ENDPOINT_LOCAL_SYSTEM);
            ping_t = xTaskGetTickCount();
            state = eState_waiting_pong;
            break;
        case eState_waiting_pong:
            if (GetPongReceived()) {
                DBG("CDSM: pong received -> resetting CR\r\n");
                state = eState_resetting;
            } else if ((xTaskGetTickCount() - ping_t) >= pdMS_TO_TICKS(PING_PONG_TIMEOUT_ms)) {
                DBG("CDSM: ping timeout, retrying\r\n");
                state = eState_pinging;
            }
            break;
        case eState_resetting:
            DBG("CDSM: sending radio reset\r\n");
            SendRatioResetTo(eEntityCoachRadio, ENDPOINT_LOCAL_SYSTEM, eResetAction_Hard);
            state = eState_pinging2;
            break;
        case eState_pinging2:
            DBG("CDSM: sending ping2 to CR (post-reset)\r\n");
            SendPingTo(eEntityCoachRadio, ENDPOINT_LOCAL_SYSTEM);
            ping_t = xTaskGetTickCount();
            state = eState_waiting_pong2;
            break;
        case eState_waiting_pong2:
            if (GetPongReceived()) {
                DBG("CDSM: pong2 received -> operating\r\n");
                s_cr_ready = true;
                state = eState_operating;
            } else if ((xTaskGetTickCount() - ping_t) >= pdMS_TO_TICKS(PING_PONG_TIMEOUT_ms)) {
                DBG("CDSM: ping2 timeout, retrying\r\n");
                state = eState_pinging2;
            }
            break;
        case eState_operating:
            break;
    }
}

static void MainTask(void *arg)
{
    GuiLvgl_Start(SD_CARD_MOUNT_POINT);

    for (;;) {
        GuiLvgl_Tick();
        CoachDisplayStateMachine();
        vTaskDelay(1);
    }
}

void app_main(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    InitDebugConsole();

    /* Hardware + LVGL init (creates LVGL mutex, starts display) */
    LcdLvgl_Init();

    SetHoldAtResponsive(false);

    if (!InitMessageProcessing()) { printf("InitMessageProcessing failed\n"); return; }
    if (!FileSystemSdInit())      { printf("SD init failed — continuing without config\n"); }
    if (!InitFileConfig())        { printf("FileConfig init failed — using defaults\n"); }
    if (!InitEPs())               { printf("InitEPs failed\n"); }

#ifdef CD_DATA_TRANSFER_SPI
    /* Hand SPI2 from SD card master → SPI slave for the CR link. */
    FileSystemSdReleaseBus();
    if (!SpiSlaveLink_Init()) {
        printf("SpiSlaveLink_Init failed\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    if (!PktParserInit()) {
        printf("PktParserInit failed\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    if (!PktProcessorInit()) {
        printf("PktProcessorInit failed\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Start LVGL task before MainTask so the screen is ready */
    LcdLvgl_StartTask();

    BaseType_t ret = xTaskCreate(MainTask, "MainTask", MAIN_TASK_STACK, NULL, 10, NULL);
    if (ret != pdPASS)
        printf("MainTask create failed\n");
}
