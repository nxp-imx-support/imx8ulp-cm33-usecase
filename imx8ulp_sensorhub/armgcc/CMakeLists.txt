# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(sensor_hub_rtos)

set(MCUX_BUILD_TYPES debug release flash_debug flash_release)

set(MCUX_SDK_PROJECT_NAME sensor_hub_rtos.elf)

if (NOT DEFINED SdkRootDirPath)
    SET(SdkRootDirPath ${ProjDirPath}/../../../../..)
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../rsc_table.c"
"${ProjDirPath}/../rsc_table.h"
"${ProjDirPath}/../remoteproc.h"
"${ProjDirPath}/../rpmsg_config.h"
"${ProjDirPath}/../srtm_config.h"
"${ProjDirPath}/../sensor_hub_rtos.c"
"${ProjDirPath}/../sensor_hub_rtos.h"
"${ProjDirPath}/../lpm.c"
"${ProjDirPath}/../lpm.h"
"${ProjDirPath}/../FreeRTOSConfig.h"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../empty_rsc_table.c"
"${ProjDirPath}/../app_srtm.c"
"${ProjDirPath}/../app_srtm.h"
"${ProjDirPath}/../pwm_app.c"
"${ProjDirPath}/../lpspi_app.c"
"${ProjDirPath}/../gpio_app.c"
"${ProjDirPath}/../sensor_app.c"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.h"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.c"
"${ProjDirPath}/../srtm/services/srtm_audio_service.h"
"${ProjDirPath}/../srtm/services/srtm_audio_service.c"
"${ProjDirPath}/../srtm/services/srtm_sai_edma_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_sai_edma_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.h"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.c"
"${ProjDirPath}/../srtm/services/srtm_sensor_service.h"
"${ProjDirPath}/../srtm/services/srtm_sensor_service.c"
"${ProjDirPath}/../srtm/services/srtm_io_service.h"
"${ProjDirPath}/../srtm/services/srtm_io_service.c"
"${ProjDirPath}/../srtm/services/srtm_keypad_service.h"
"${ProjDirPath}/../srtm/services/srtm_keypad_service.c"
"${ProjDirPath}/../srtm/services/srtm_pwm_service.h"
"${ProjDirPath}/../srtm/services/srtm_pwm_service.c"
"${ProjDirPath}/../srtm/services/srtm_pwm_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_pwm_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_rtc_service.h"
"${ProjDirPath}/../srtm/services/srtm_rtc_service.c"
"${ProjDirPath}/../srtm/services/srtm_rtc_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_rtc_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.h"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.c"
"${ProjDirPath}/../paint/debug_paint.h"
"${ProjDirPath}/../paint/font12.c"
"${ProjDirPath}/../paint/font12CN.c"
"${ProjDirPath}/../paint/font16.c"
"${ProjDirPath}/../paint/font20.c"
"${ProjDirPath}/../paint/font24.c"
"${ProjDirPath}/../paint/font24CN.c"
"${ProjDirPath}/../paint/font8.c"
"${ProjDirPath}/../paint/fonts.h"
"${ProjDirPath}/../paint/gui_paint.c"
"${ProjDirPath}/../paint/gui_paint.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
    ${SdkRootDirPath}/boards/evkmimx8ulp
    ${ProjDirPath}/../srtm/channels
    ${ProjDirPath}/../srtm/services
    ${ProjDirPath}/../paint
)

set(CMAKE_MODULE_PATH
    ${SdkRootDirPath}/components/codec/i2c
    ${SdkRootDirPath}/devices/MIMX8UD7/drivers
    ${SdkRootDirPath}/components/i2c
    ${SdkRootDirPath}/components/codec
    ${SdkRootDirPath}/components/codec/
    ${SdkRootDirPath}/rtos/freertos/freertos-kernel
    ${SdkRootDirPath}/middleware/multicore
    ${SdkRootDirPath}/devices/MIMX8UD7
    ${SdkRootDirPath}/devices/MIMX8UD7/utilities
    ${SdkRootDirPath}/components/uart
    ${SdkRootDirPath}/components/serial_manager
    ${SdkRootDirPath}/components/lists
    ${SdkRootDirPath}/CMSIS/Core/Include
    ${SdkRootDirPath}/components/pca6416a
    ${SdkRootDirPath}/components/lsm6dso
    ${SdkRootDirPath}/components/srtm/port
    ${SdkRootDirPath}/components/srtm
    ${SdkRootDirPath}/components/pwm
    ${SdkRootDirPath}/components/rtc
    ${SdkRootDirPath}/components/codec/wm8960
)

# include modules
include(component_codec_i2c_MIMX8UD7_cm33)

include(driver_common_MIMX8UD7_cm33)

include(component_lpi2c_adapter_MIMX8UD7_cm33)

include(driver_codec_MIMX8UD7_cm33)

include(component_wm8960_adapter_MIMX8UD7_cm33)

include(middleware_freertos-kernel_cm33_nonsecure_port_MIMX8UD7_cm33)

include(middleware_freertos-kernel_heap_4_MIMX8UD7_cm33)

include(driver_mu1_MIMX8UD7_cm33)

include(driver_rtd_cmc_MIMX8UD7_cm33)

include(driver_sentinel_MIMX8UD7_cm33)

include(middleware_multicore_rpmsg_lite_MIMX8UD7_cm33)

include(middleware_multicore_rpmsg_lite_freertos_MIMX8UD7_cm33)

include(driver_lptmr_MIMX8UD7_cm33)

include(driver_clock_MIMX8UD7_cm33)

include(device_MIMX8UD7_CMSIS_MIMX8UD7_cm33)

include(utility_debug_console_MIMX8UD7_cm33)

include(utility_assert_MIMX8UD7_cm33)

include(component_lpuart_adapter_MIMX8UD7_cm33)

include(driver_flexspi_MIMX8UD7_cm33)

include(component_serial_manager_MIMX8UD7_cm33)

include(component_lists_MIMX8UD7_cm33)

include(component_serial_manager_uart_MIMX8UD7_cm33)

include(driver_lpuart_MIMX8UD7_cm33)

include(device_MIMX8UD7_startup_MIMX8UD7_cm33)

include(driver_cache_cache64_MIMX8UD7_cm33)

include(driver_iomuxc_MIMX8UD7_cm33)

include(driver_reset_MIMX8UD7_cm33)

include(driver_upower_MIMX8UD7_cm33)

include(driver_rgpio_MIMX8UD7_cm33)

include(driver_fusion_MIMX8UD7_cm33)

include(driver_trdc_MIMX8UD7_cm33)

include(driver_dma3_MIMX8UD7_cm33)

include(driver_lpi2c_MIMX8UD7_cm33)

include(CMSIS_Include_core_cm_MIMX8UD7_cm33)

include(driver_lpi2c_freertos_MIMX8UD7_cm33)

include(driver_lpspi_MIMX8UD7_cm33)

include(driver_lpspi_freertos_MIMX8UD7_cm33)

include(middleware_freertos-kernel_MIMX8UD7_cm33)

include(middleware_freertos-kernel_extension_MIMX8UD7_cm33)

include(driver_sai_edma_MIMX8UD7_cm33)

include(driver_sai_MIMX8UD7_cm33)

include(driver_wuu_MIMX8UD7_cm33)

include(driver_pca6416a_MIMX8UD7_cm33)

include(driver_lsm6dso_MIMX8UD7_cm33)

include(driver_srtm_freertos_MIMX8UD7_cm33)

include(driver_srtm_MIMX8UD7_cm33)

include(middleware_multicore_rpmsg_lite_imx8ulp_m33_freertos_MIMX8UD7_cm33)

include(component_pwm_tpm_adapter_MIMX8UD7_cm33)

include(driver_tpm_MIMX8UD7_cm33)

include(component_rtc_bbnsm_adapter_MIMX8UD7_cm33)

include(driver_bbnsm_MIMX8UD7_cm33)

include(driver_wm8960_MIMX8UD7_cm33)

include(utilities_misc_utilities_MIMX8UD7_cm33)


IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
ENDIF()  

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/sdk20-app.bin)

