include_guard()
message("driver_iomuxc component is included.")


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)

include(driver_common_MIMX8UD7_cm33)
