cmake_minimum_required(VERSION 3.16)
project(lcdfull_sim C)
set(CMAKE_C_STANDARD 99)

# ---- LVGL configuration -------------------------------------------------------
# lv_conf.h is in this directory (sim/).
set(LV_BUILD_CONF_DIR  ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "" FORCE)
set(CONFIG_LV_BUILD_DEMOS    OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# ---- SDL2 ---------------------------------------------------------------------
find_package(SDL2 REQUIRED)

# ---- LVGL library -------------------------------------------------------------
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/lvgl__lvgl
    ${CMAKE_CURRENT_BINARY_DIR}/lvgl_build
)
target_link_libraries(lvgl PRIVATE SDL2::SDL2)

# ---- Simulator executable -----------------------------------------------------
set(GUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../components/GUI)

set(PACKET_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../PacketParser)
set(ADMIN_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/../PeachAdminMessages)

add_executable(lcdfull_sim
    main_pc.c
    sim_stubs.c
    sim_pipeline.c
    ${PACKET_DIR}/PhysicalParams.c
    ${ADMIN_DIR}/PeachMessages.c
    ${ADMIN_DIR}/SerUtils.c
    ${ADMIN_DIR}/DeSerUtils.c
    ${GUI_DIR}/GuiLvgl.c
    ${GUI_DIR}/BoatSelectLvgl.c
    ${GUI_DIR}/ForceAngleLvgl.c
    ${GUI_DIR}/HandleSpeedLvgl.c
    ${GUI_DIR}/PowerLvgl.c
    ${GUI_DIR}/ArcLengthLvgl.c
    ${GUI_DIR}/AccelTimeLvgl.c
    ${GUI_DIR}/FleetViewLvgl.c
    ${GUI_DIR}/CalibLvgl.c
    ${GUI_DIR}/GraphConfig.c
)

# Stub headers in sim/include/ MUST come before the real component directories
# so that freertos/*.h, esp_attr.h, lcd.h, FileSystemSD.h are resolved first.
target_include_directories(lcdfull_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}                                   # lv_conf.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include                           # stub headers (FreeRTOS, esp_attr, lcd, FileSystemSD)
    ${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/lvgl__lvgl  # lvgl.h
    ${GUI_DIR}                                                    # GraphScreen.h, GraphConfig.h
    ${GUI_DIR}/include                                            # GuiLvgl.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../PacketParser                   # EndPoints.h, PacketProcessor.h, PhysicalParams.h
    ${ADMIN_DIR}                                                   # AdminCommon.h, AdminMsgs.h, PeachMessages.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../FileSystem                     # FileConfig.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../Display                        # DisplayDriver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../main                           # PeachDefs.h
)

target_link_libraries(lcdfull_sim PRIVATE lvgl SDL2::SDL2 m pthread)

target_compile_options(lcdfull_sim PRIVATE
    -Wall
    -Wno-unused-parameter
    -Wno-unused-function
)
