cmake_minimum_required(VERSION 3.8)
project(uro_shared_memory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)

add_library(uro_shm SHARED src/CameraImage.cc src/RobotName.cc)
target_link_libraries(uro_shm)

pybind11_add_module(uro_shm_py src/uro_shm.cc)
set_target_properties(uro_shm_py PROPERTIES OUTPUT_NAME uro_shm)
target_link_libraries(uro_shm_py PRIVATE uro_shm ${Python3_LIBRARIES})

# Python の site-packages 先
set(PYTHON_SITE_PACKAGES
  lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)
# uro_shared_memory パッケージ下に配置したいので、さらにサブディレクトリを足す
set(PYTHON_PKG_DIR ${PYTHON_SITE_PACKAGES}/uro_shared_memory)

install(TARGETS
  uro_shm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# バイナリ拡張モジュール（.so）を site-packages/uro_shared_memory へ
install(TARGETS uro_shm_py
  LIBRARY DESTINATION ${PYTHON_PKG_DIR}
)

# __init__.py も同じ場所へ
install(FILES python/uro_shared_memory/__init__.py
  DESTINATION ${PYTHON_PKG_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
