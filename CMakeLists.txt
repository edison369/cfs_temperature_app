project(CFE_TEMP_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${temp_app_MISSION_DIR}/fsw/platform_inc)

aux_source_directory(fsw/src APP_SRC_FILES)

# Include the apps that send the temperature packets
include_directories(${altitude_app_MISSION_DIR}/fsw/platform_inc)
include_directories(${altitude_app_MISSION_DIR}/fsw/src)

include_directories(${imu_app_MISSION_DIR}/fsw/platform_inc)
include_directories(${imu_app_MISSION_DIR}/fsw/src)

# Create the app module
add_cfe_app(temp_app ${APP_SRC_FILES})
