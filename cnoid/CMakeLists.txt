find_package(PkgConfig)

# openhrp3.1
pkg_check_modules(OPENHRP3_1 REQUIRED openhrp3.1)
message("-- OPENHRP3_1_PREFIX: ${OPENHRP3_1_PREFIX}")

# hrpsys-base
pkg_check_modules(HRPSYSBASE REQUIRED hrpsys-base)
message("-- HRPSYSBASE_PREFIX: ${HRPSYSBASE_PREFIX}")

# choreonoid
find_package(choreonoid)
if(choreonoid_FOUND)
  set(CHOREONOID_FULL_SHARE_DIR "${CHOREONOID_ROOT_DIR}/${CHOREONOID_SHARE_SUBDIR}")
  message("-- CHOREONOID_FULL_SHARE_DIR ${CHOREONOID_FULL_SHARE_DIR}")
  install(DIRECTORY model/ DESTINATION ${CHOREONOID_FULL_SHARE_DIR}/model/environment/MCC)
endif()

add_subdirectory(project)
