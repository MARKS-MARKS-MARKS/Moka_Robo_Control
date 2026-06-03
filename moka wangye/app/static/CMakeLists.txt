cmake_minimum_required(VERSION 3.5)

project(mr07s)

# 导入ament包
find_package(ament_cmake REQUIRED)

# 设置包依赖
ament_package()

# 安装目录
install(DIRECTORY
  meshes
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

