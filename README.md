# 271201Project
ปิยะนันท์ ปิยะวรรณ์โณ 650610845

package name : forklift

```py
ros2 launch forklift launch_sim.launch.py
```

```py
ros2 run forklift forklift_control.py
```

# src
  forklift_control.py : สำหรับ bind ปุ่มที่ใช่ในการควบคุม

# launch
  launch_sim.launch.py : สำหรับ run robot.urdf.xacro พร้อมเปิด gazebo

# config
  controllers.yaml : สำหรับการ claim joint

# description
robot.urdf.xacro : สำหรับใช้ launch
robot_core.xacro : สำหรับประกอบหุ่น และใส่ inertia
lidar.xarco : สำหรับติด lidar
jointcontrol.xacro : สำหนับ set ค่า max/min velocity ของ joint
gazebo_control.xacro : สำหรับทำให้ขับเคลื่อน 4 ล้อ  และควบคุม joint
  
  
