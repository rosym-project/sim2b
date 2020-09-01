if(ENABLE_BULLET)
  find_package(PkgConfig)
  pkg_check_modules(BULLET_ROBOTICS REQUIRED bullet_robotics)

  include_directories(${BULLET_ROBOTICS_INCLUDE_DIRS})

  add_library(sim2b_bullet SHARED bullet.cpp)
  target_link_libraries(sim2b_bullet PRIVATE ${BULLET_ROBOTICS_LIBRARIES})


  install(
    TARGETS sim2b_bullet
    EXPORT sim2b_bullet-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  set(EXPORTED_LIBRARIES ${EXPORTED_LIBRARIES} sim2b_bullet PARENT_SCOPE)
endif()
