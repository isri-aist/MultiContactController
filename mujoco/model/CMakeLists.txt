set(MODEL_NAME_LIST
  SampleField
)

foreach(MODEL_NAME IN LISTS MODEL_NAME_LIST)
  file(WRITE
    ${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.yaml
    "xmlModelPath: ${MC_MUJOCO_MODEL_DIR}/${MODEL_NAME}.xml"
  )
  if(DEFINED CATKIN_DEVEL_PREFIX)
    configure_file(${MODEL_NAME}.xml
      ${MC_MUJOCO_MODEL_DIR}/${MODEL_NAME}.xml
      COPYONLY
    )
    configure_file(${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.yaml
      ${MC_MUJOCO_MODEL_DIR}/${MODEL_NAME}.yaml
      COPYONLY
    )
  else()
    install(FILES
      ${MODEL_NAME}.xml
      ${CMAKE_CURRENT_BINARY_DIR}/${MODEL_NAME}.yaml
      DESTINATION
      ${MC_MUJOCO_MODEL_DIR}
    )
  endif()
endforeach()
