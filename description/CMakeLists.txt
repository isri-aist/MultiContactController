# Copied from https://github.com/jrl-umi3218/mc_rtc/blob/b2fe81b4f418c8251d85d3ceee974c0ba7e0610a/src/mc_robots/CMakeLists.txt#L28-L45
macro(append_aliases PATH VAR NAME)
  file(GLOB URDF_FILES ${PATH}/urdf/*.urdf)
  foreach(URDF ${URDF_FILES})
    get_filename_component(URDF_NAME "${URDF}" NAME_WE)
    # Robot should be env (fixed base) or object (floating base) instead of ${NAME} because arbitrary name robot cannot be added
    list(APPEND ${VAR} "${NAME}/${URDF_NAME}: [env, \"${PATH}\", ${URDF_NAME}]")
  endforeach()
  string(REPLACE ";" "\n" ${VAR} "${${VAR}}")
endmacro()

append_aliases(${CMAKE_CURRENT_SOURCE_DIR} MCC_ALIASES MCC)

set(MCC_ROBOT_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/aliases/mcc_aliases.yml")
file(WRITE ${MCC_ROBOT_MODULE_PATH} "${MCC_ALIASES}")
message("-- MCC_ROBOT_MODULE_PATH: ${MCC_ROBOT_MODULE_PATH}")
install(FILES "${MCC_ROBOT_MODULE_PATH}" DESTINATION "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}/aliases/")
