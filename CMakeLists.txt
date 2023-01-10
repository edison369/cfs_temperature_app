project(CFE_TEMP_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${temp_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(temp_app fsw/src/temp_app.c)
