if(ENABLE_BULLET)
  find_package(PkgConfig)
  pkg_check_modules(BULLET REQUIRED bullet)
  pkg_check_modules(BULLET_ROBOTICS REQUIRED bullet_robotics)

  include_directories(
    ${BULLET_INCLUDEDIR}/SharedMemory
    ${BULLET_ROBOTICS_INCLUDE_DIRS}
  )

  link_directories(
    ${BULLET_LIBRARY_DIRS}
    ${BULLET_ROBOTICS_LIBRARY_DIRS}
  )

  add_library(sim2b_bullet SHARED bullet.cpp)
  target_link_libraries(sim2b_bullet PRIVATE
    ${BULLET_LIBRARIES}
    ${BULLET_ROBOTICS_LIBRARIES}
  )

  target_compile_options(sim2b_bullet PUBLIC
    ${BULLET_CFLAGS}
    ${BULLET_ROBOTICS_CFLAGS}
  )

  install(
    TARGETS sim2b_bullet
    EXPORT sim2b-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  set(EXPORTED_LIBRARIES ${EXPORTED_LIBRARIES} sim2b_bullet PARENT_SCOPE)
endif()
