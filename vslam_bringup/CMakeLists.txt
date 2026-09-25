cmake_minimum_required(VERSION 3.8)
project(vslam_bringup)

find_package(ament_cmake REQUIRED)

# 這個套件不編譯任何程式碼,只是把 launch/ 跟 config/ 安裝到 share/ 底下,
# 讓 `ros2 launch vslam_bringup ...` 找得到。實際的 SLAM 節點執行檔在 stella_vslam_ros 套件裡。
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
