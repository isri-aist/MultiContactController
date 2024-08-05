execute_process(
  COMMAND python -c "from distutils import sysconfig; print(sysconfig.get_config_var(\"VERSION\"))"
  OUTPUT_VARIABLE PYTHON_VERSION
  RESULT_VARIABLE PYTHON_VERSION_SUCCESS
  OUTPUT_STRIP_TRAILING_WHITESPACE)

set(MCC_CNOID_CONFIG_FILE_LIST
  rtc.conf.choreonoid
  robot.conf
  RobotHardware.conf
  PDcontroller.conf.choreonoid
  PDgains_sim.dat
)

foreach(CNOID_CONFIG_FILE IN LISTS MCC_CNOID_CONFIG_FILE_LIST)
  execute_process(COMMAND cmake -E create_symlink
    "${HRPSYSBASE_PREFIX}/share/hrpsys/samples/JVRC1/${CNOID_CONFIG_FILE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/${CNOID_CONFIG_FILE}"
  )
endforeach()

file(GLOB MCC_PROJECT_FILE_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.in.cnoid)
foreach(PROJECT_FILE_IN IN LISTS MCC_PROJECT_FILE_LIST)
  get_filename_component(PROJECT_FILE_IN ${PROJECT_FILE_IN} NAME)
  string(REPLACE ".in." "." PROJECT_FILE_OUT "${PROJECT_FILE_IN}")
  set(MCC_MODEL_DIR "${PROJECT_SOURCE_DIR}/cnoid/model")
  configure_file(${PROJECT_FILE_IN}
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_FILE_OUT}
    @ONLY)
  if(DEFINED CHOREONOID_FULL_SHARE_DIR)
    set(MCC_MODEL_DIR ${CHOREONOID_FULL_SHARE_DIR}/model/environment/MCC)
    configure_file(${PROJECT_FILE_IN}
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_FILE_OUT}
      @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_FILE_OUT} DESTINATION ${HRPSYSBASE_PREFIX}/share/hrpsys/samples/JVRC1/)
  endif()
endforeach()
