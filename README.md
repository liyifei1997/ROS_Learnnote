# ROS_Learnnote


ROS Workspace Structure:

workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n


WorkSpace --- 自定义的工作空间

    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。

    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件、动态&静态链接库、可执行文件等。

    |--- src: 源码

        |-- package：功能包(ROS基本单元)包含多个节点、库与配置文件，包名所有字母小写，只能由字母、数字与下划线组成

            |-- CMakeLists.txt 配置编译规则，比如源文件、依赖项、目标文件

            |-- package.xml 包信息，比如:包名、版本、作者、依赖项...(以前版本是 manifest.xml)

            |-- scripts 存储python文件

            |-- src 存储C++源文件

            |-- include 头文件

            |-- msg 消息通信格式文件

            |-- srv 服务通信格式文件

            |-- action 动作格式文件

            |-- launch 可一次性运行多个节点 

            |-- config 配置信息

        |-- CMakeLists.txt: 编译的基本配置




1. 创建工作空间
  mkdir -p 自定义空间名称/src
  cd 自定义空间名称
  catkin_make

2. 创建包
  cd src
  catkin_create_pkg 自定义ROS包名 std_msgs rospy roscpp 

3. 创建py（py在scripts文件夹下）
  mkdir scripts

4. 添加权限
  chmod +x *.py

5. 更改包下CamkeList.txt

6. 工作空间编译
  catkin_make

7. Terminal启动
  1. roscore
  2. cd 工作空间
      source ./devel/setup.bash
      rosrun 包名 自定义文件名.py