if(ENABLE_BULLET)
  add_executable(bullet bullet.c)
  target_link_libraries(bullet sim2b_bullet)

  add_custom_command(TARGET bullet
    POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/2dof.urdf" "${CMAKE_BINARY_DIR}/2dof.urdf"
  )
endif()
