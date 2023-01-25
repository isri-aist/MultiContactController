set(CONTROLLER_NAME MultiContactController)

add_library(${CONTROLLER_NAME} SHARED
  MultiContactController.cpp
  LimbTypes.cpp
  )
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)

if(DEFINED CATKIN_DEVEL_PREFIX)
  target_link_libraries(${CONTROLLER_NAME} PUBLIC ${catkin_LIBRARIES})
else()
  target_link_libraries(${CONTROLLER_NAME} PUBLIC
    trajectory_collection::TrajColl
    force_control_collection::ForceColl
    centroidal_control_collection::CCC
  )
endif()

install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR} EXPORT ${TARGETS_EXPORT_NAME})

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

add_subdirectory(states)
