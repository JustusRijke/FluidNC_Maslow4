FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/FluidNC/*.*)

list(FILTER app_sources EXCLUDE REGEX "/test/")
list(FILTER app_sources EXCLUDE REGEX "/tests/")
list(REMOVE_ITEM app_sources "${CMAKE_SOURCE_DIR}/FluidNC/esp32/i2s_engine.c")
list(REMOVE_ITEM app_sources "${CMAKE_SOURCE_DIR}/FluidNC/src/BTConfig.cpp")
list(REMOVE_ITEM app_sources "${CMAKE_SOURCE_DIR}/FluidNC/src/Machine/I2SOBus.cpp")
list(REMOVE_ITEM app_sources "${CMAKE_SOURCE_DIR}/FluidNC/src/OLED.cpp")
list(REMOVE_ITEM app_sources "${CMAKE_SOURCE_DIR}/FluidNC/src/Pins/I2SOPinDetail.cpp")

idf_component_register(SRCS ${app_sources})
