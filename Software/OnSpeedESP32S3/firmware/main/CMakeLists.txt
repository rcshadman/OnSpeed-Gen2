# ONSPEED_HW=1 builds the real-hardware variant (I2C sensors, ADC flap pot, I2S audio).
# Default is the simulator variant that runs in QEMU; the ADC driver is left out of it because
# its start-up calibration polls ADC hardware that QEMU does not model.
if(DEFINED ENV{ONSPEED_HW})
  set(reqs onspeed_core esp_driver_i2c esp_driver_i2s esp_adc)
else()
  set(reqs onspeed_core esp_driver_i2c esp_driver_i2s)
endif()
idf_component_register(SRCS "main.cpp" "flight_model.cpp" "sensors.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ${reqs})
if(DEFINED ENV{ONSPEED_HW})
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ONSPEED_SIM=0)
endif()
