if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/jvrc_mj_description/CMakeLists.txt")
  message(FATAL_ERROR "No CMakeLists.txt in jvrc_mj_description folder, did you run `git submodule update --init`?")
endif()

set(HFIELD "castle_wall_1000" CACHE STRING "Define the hfield")
set(HFIELD_PATH "/home/jrluser/mc_mujoco/robots/Heighmap/${HFIELD}.png")

set(HFIELD_PATH1 "/home/jrluser/mc_mujoco/robots/Heighmap/${HFIELD1}.png")
set(HFIELD_PATH2 "/home/jrluser/mc_mujoco/robots/Heighmap/${HFIELD2}.png")
set(HFIELD_PATH3 "/home/jrluser/mc_mujoco/robots/Heighmap/${HFIELD3}.png")
set(HFIELD_PATH4 "/home/jrluser/mc_mujoco/robots/Heighmap/${HFIELD4}.png")
set(HFIELD_PATH5 "/home/jrluser/mc_mujoco/robots/Heighmap/${HFIELD5}.png")

add_subdirectory(jvrc_mj_description)
add_subdirectory(hrp4cr_mj_description)
add_subdirectory(hrp4j_mj_description)
add_subdirectory(hrp4j_soft_mj_description)

function(setup_env_object NAME ENV_OBJECT)
  set(YAML_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}.yaml")
  configure_file(${NAME}.in.yaml "${YAML_OUT}")
  install(FILES "${YAML_OUT}" DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}")
  install(FILES "${NAME}.xml" DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}/${ENV_OBJECT}")
  install(DIRECTORY assets DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}/${ENV_OBJECT}")
endfunction()

setup_env_object(ground env)
setup_env_object(box object)
setup_env_object(longtable object)
