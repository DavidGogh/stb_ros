# If the pyx file is a C++ file, we should specify that here.
set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/hapticsstb_rt.pyx
  PROPERTIES CYTHON_IS_CXX FALSE )
  
if(APPLE)
	include_directories(/System/Library/Frameworks/Python.framework/Versions/2.7/Extras/lib/python/numpy/core/include/)
endif(APPLE)

# Multi-file cython modules do not appear to be working at the moment.
cython_add_module(hapticsstb_rt hapticsstb_rt.pyx hapticsstb_rt.c)

# target_link_libraries(hapticsstb_rt ${PCL_LIBRARIES})

if(${hapticsstb_rt_ROS})
	set_target_properties(hapticsstb_rt PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
	install(TARGETS hapticsstb_rt
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
	)
endif(${hapticsstb_rt_ROS})
