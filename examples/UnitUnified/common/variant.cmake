# Map the Kconfig "Target unit" choice (common/Kconfig.variant) to the source-level macro that
# both the Arduino and ESP-IDF builds use. Include this from an example's main/CMakeLists.txt
# *after* idf_component_register() (it needs ${COMPONENT_LIB}).
set(M5UNIT_VARIANT "")
if(CONFIG_EXAMPLE_USING_UNIT_TOF)
    set(M5UNIT_VARIANT USING_UNIT_TOF)
elseif(CONFIG_EXAMPLE_USING_UNIT_TOF4M)
    set(M5UNIT_VARIANT USING_UNIT_TOF4M)
elseif(CONFIG_EXAMPLE_USING_UNIT_TOF90)
    set(M5UNIT_VARIANT USING_UNIT_TOF90)
elseif(CONFIG_EXAMPLE_USING_HAT_TOF)
    set(M5UNIT_VARIANT USING_HAT_TOF)
endif()
if(M5UNIT_VARIANT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${M5UNIT_VARIANT})
endif()
