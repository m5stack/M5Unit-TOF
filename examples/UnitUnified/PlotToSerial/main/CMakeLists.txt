# Dependencies (M5Unit-TOF, M5UnitUnified, M5Unified, ...) are pulled in via
# main/idf_component.yml and auto-added as requirements by the component manager.
idf_component_register(
    SRCS "PlotToSerial.cpp"
    INCLUDE_DIRS ".")

# The unit variant is selected via Kconfig (main/Kconfig.projbuild -> common/Kconfig.variant) with
# `idf.py menuconfig`. Include the CONFIG_* -> macro mapping after the register call (it needs
# ${COMPONENT_LIB}).
include("${CMAKE_CURRENT_LIST_DIR}/../../common/variant.cmake")
