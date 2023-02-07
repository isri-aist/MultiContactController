find_package(PkgConfig)

# openhrp3.1
if(NOT DEFINED OPENHRP3_1_PREFIX)
  pkg_check_modules(OPENHRP3_1 REQUIRED openhrp3.1)
endif()
message("-- OPENHRP3_1_PREFIX: ${OPENHRP3_1_PREFIX}")

# hrpsys-base
if(NOT DEFINED HRPSYSBASE_PREFIX)
  pkg_check_modules(HRPSYSBASE REQUIRED hrpsys-base)
endif()
message("-- HRPSYSBASE_PREFIX: ${HRPSYSBASE_PREFIX}")

add_subdirectory(project)
