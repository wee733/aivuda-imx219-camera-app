cmake_minimum_required(VERSION 3.20)
project(cuda_buffer_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Workaround for CMake's FindCUDAToolkit on Ubuntu's split nvidia-cuda-dev
# layout in the bloom build pipeline. Pre-populate the cache variable to
# skip the broken auto-detection.
# Related: https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=995122
find_path(_CUDA_RUNTIME_DIR
  NAMES cuda_runtime.h
  HINTS
    ENV CUDA_HOME
    ENV CUDA_PATH
    /usr/local/cuda/include
    /usr/include
)
if(_CUDA_RUNTIME_DIR AND NOT DEFINED CUDAToolkit_INCLUDE_DIRECTORIES)
  set(CUDAToolkit_INCLUDE_DIRECTORIES "${_CUDA_RUNTIME_DIR}" CACHE PATH
    "CUDA include dir (FindCUDAToolkit nvidia-cuda-dev workaround)")
endif()
find_package(CUDAToolkit REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/cuda_buffer_backend_plugin.cpp
)

# Imported-buffer deleters are queued on cuda_buffer's process-lifetime recycler.
# Keep their code mapped after pluginlib destroys the backend registry at shutdown.
# Otherwise a pending deleter can execute after dlclose (observed on JetPack 7.2).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(${PROJECT_NAME} PRIVATE "LINKER:-z,nodelete")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  ${cuda_buffer_backend_msgs_TARGETS}
  rosidl_buffer_backend::rosidl_buffer_backend
  rosidl_runtime_cpp::rosidl_runtime_cpp
  rosidl_buffer_backend_registry::rosidl_buffer_backend_registry
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  pluginlib::pluginlib
  cuda_buffer::cuda_buffer
  rcutils::rcutils
  CUDA::cudart
  CUDA::cuda_driver
)

# The plugin is dlopen'd by pluginlib at runtime; its install RPATH needs the
# CUDA toolkit library directory so libcudart.so.<abi> can be resolved without
# relying on the caller's LD_LIBRARY_PATH.
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

pluginlib_export_plugin_description_file(rosidl_buffer_backend cuda_buffer_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cuda_buffer_backend
    test/test_cuda_buffer_backend.cpp
  )
  if(TARGET test_cuda_buffer_backend)
    target_link_libraries(test_cuda_buffer_backend
      ${PROJECT_NAME}
      rosidl_buffer_backend::rosidl_buffer_backend
      rosidl_runtime_cpp::rosidl_runtime_cpp
      rosidl_buffer_backend_registry::rosidl_buffer_backend_registry
      ${cuda_buffer_backend_msgs_TARGETS}
      CUDA::cudart
    )
  endif()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
