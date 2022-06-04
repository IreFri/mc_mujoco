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

configure_file(ground.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/ground.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ground.yaml" DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}")
install(FILES ground.xml DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}/env")
