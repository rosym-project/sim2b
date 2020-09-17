if(ENABLE_BULLET)
  find_package(PkgConfig)

  pkg_check_modules(UBX REQUIRED ubx0)

  pkg_get_variable(UBX_MODDIR ubx0 UBX_MODDIR)
  pkg_get_variable(UBX_EXEC_PREFIX ubx0 exec_prefix)

  file(RELATIVE_PATH UBX_MODDIR_REL ${UBX_EXEC_PREFIX} ${UBX_MODDIR})

  include_directories(${UBX_INCLUDE_DIRS})

  add_library(sim2b_bullet_ubx SHARED bullet.c)
  target_link_libraries(sim2b_bullet_ubx PRIVATE sim2b_bullet ${UBX_LIBRARIES})

  configure_file(ubx_env.sh.in ${CMAKE_BINARY_DIR}/ubx_env.sh)

  add_custom_command(TARGET sim2b_bullet_ubx
    POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_BINARY_DIR}/libsim2b_bullet_ubx.so"
        "${CMAKE_CURRENT_BINARY_DIR}/${UBX_MODDIR_REL}/sim2b_bullet.so")

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/libsim2b_bullet_ubx.so"
          DESTINATION "${CMAKE_INSTALL_DIRECTORY}${UBX_MODDIR_REL}"
          RENAME sim2b_bullet.so)
endif()
