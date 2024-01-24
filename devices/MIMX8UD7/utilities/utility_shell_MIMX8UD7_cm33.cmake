include_guard()
message("utility_shell component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_shell.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(utility_str_MIMX8UD7_cm33)

include(component_lists_MIMX8UD7_cm33)

include(driver_common_MIMX8UD7_cm33)

