# shenlanxueyuan Homework 3
### grid_path_searcher
- 需要配合hw2中的另外两个包才能运行，这里的包只是RRT的实现部分

## rrt_global_planner
- ### 介绍
  - 自己写的替代move_base中现有global path planner的插件
  - 由ompl的rrt*实现
  - **开发过程中，发现RRT的bounds如果设置的非常大，在规定时间内经常会找不到可行解。在这里测试了30×30(米)大小的bounds，基本上每次都能求解出可行的全局路径了。**

- ### 使用流程
  - 创建stage环境
    > roslaunch rrt_global_planner world.launch
    
  - 开启导航包
    > roslaunch rrt_global_planner nav.launch
    
  - rviz中设置目标点即可
  
  - 如果想重新建图
    - 创建stage环境
    > roslaunch rrt_global_planner world.launch
    
    - 开启建图包
    > roslaunch rrt_global_planner mapping.launch
    
    - 开启遥控
    > roslaunch rrt_global_planner teleop.launch
    
    - 控制小车移动
    
    - 保存地图
    > rosrun map_server map_saver
  