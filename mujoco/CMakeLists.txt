# mc_mujoco
if(NOT DEFINED MC_MUJOCO_MODEL_DIR)
  find_package(mc_mujoco REQUIRED)
  set(MC_MUJOCO_MODEL_DIR ${MC_MUJOCO_USER_DESTINATION})
endif()
message("-- MC_MUJOCO_MODEL_DIR: ${MC_MUJOCO_MODEL_DIR}")

add_subdirectory(model)
