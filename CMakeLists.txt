project(CFE_TEMP_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${temp_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(temp_app fsw/src/temp_app.c)

# Include the apps that send the temperature packets
include_directories(${altitude_app_MISSION_DIR}/fsw/platform_inc)
include_directories(${altitude_app_MISSION_DIR}/fsw/src)

# Include the public API from aht10, mpl3115a2 and mpu6050
# to call library-provided functions
add_cfe_app_dependency(temp_app aht10)
add_cfe_app_dependency(temp_app mpl3115a2)
add_cfe_app_dependency(temp_app mpu6050)
