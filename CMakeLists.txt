project(CFE_TEMPERATURE_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${temperature_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(temperature_app fsw/src/temperature_app.c)
