cmake_minimum_required(VERSION 3.5)
project(nucleus_driver_meta)

find_package(ament_cmake REQUIRED)

# launch フォルダをインストールターゲットに含める
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

