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

# ORB 詞彙檔:colcon_build.sh 會在 build 前自動把它複製/下載到 vocab/(不進版控,見 .gitignore),
# 讓它跟著這個套件一起裝進 share/,launch 檔預設路徑才不用寫死這台機器的絕對路徑。
# 用 if(EXISTS) 包起來是因為有人可能繞過 colcon_build.sh 直接 `colcon build`,vocab/ 還沒被準備好
# 時不要讓整個套件安裝失敗——這種情況下 launch 檔的 vocab_file 預設路徑就會不存在,需要自己手動指定。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vocab")
  install(DIRECTORY
    vocab
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
