# Optional helper; CMakeLists.txt also uses pkg_check_modules(libcamera).
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA QUIET libcamera)
endif()
if(LIBCAMERA_FOUND)
  set(LIBCAMERA_LIBRARIES ${LIBCAMERA_LIBRARIES})
  set(LIBCAMERA_INCLUDE_DIRS ${LIBCAMERA_INCLUDE_DIRS})
endif()
