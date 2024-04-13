add_fsm_state(InitialState InitialState.cpp)
target_link_libraries(InitialState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(ConfigMotionState ConfigMotionState.cpp)
target_link_libraries(ConfigMotionState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(InterruptState InterruptState.cpp)
target_link_libraries(InterruptState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(GuiWalkState GuiWalkState.cpp)
target_link_libraries(GuiWalkState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_state(GuiStepState GuiStepState.cpp)
target_link_libraries(GuiStepState PUBLIC
  ${CONTROLLER_NAME})

add_fsm_data_directory(data)
