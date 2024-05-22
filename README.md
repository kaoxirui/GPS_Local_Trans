# GPS_to_Local
* 把发过来的gps的经纬高坐标转换到局部坐标系（或局部坐标系转gps）。其中局部坐标系和东北天的姿态角比较特殊，yaw为x轴与北向的夹角（0-360，右为正），pitch向上为正，roll右滚为正

* msg_pub和msg_pub_local为测试用

* hover_ego:把转到local系的坐标发给ego planner

---

2024-5-21

* 封装了一下gps转local的屎山代码

* gps_to_local_node.cpp依赖于gps_to_local.cpp和头文件，所以要用配置源文件的方式配置CMakeLists

http://www.autolabor.com.cn/book/ROSTutorials/di-3-zhang-ros-tong-xin-ji-zhi-jin-jie/32-roszhong-de-tou-wen-jian-yu-yuan-wen-jian/322-zi-ding-yi-yuan-wen-jian-diao-yong.html

* 注意回调函数传入this指针，表示类的实例，这样才能访问成员变量

---
2024-5-22

对代码格式和命名进行了规范化，把local to gps的代码也进行了封装