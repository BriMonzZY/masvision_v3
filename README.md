# masvision_v3

![img](https://github.com/BriMonzZY/roborts-mas/blob/main/images/rmuc.png)

河北工业大学山海机甲战队RMUC2024哨兵代码

基于2023年RMUL哨兵代码 [roborts_mas](https://github.com/BriMonzZY/roborts-mas) 进行修改

详细的README见 [roborts_mas](https://github.com/BriMonzZY/roborts-mas) 仓库

哨兵结构为双头结构，相机采用海康威视的工业相机
</br>
</br>
执行：
```
./auto_start_double.sh
```
会启动双头的自瞄程序（启动了两个group，并分别启动了base节点、hikrobot_mvs节点和armor_detect节点），此时需要相机ID为1和2，注意双头的USB的ID的对应（还没有来得及添加ID固定），否则会出现错误。

执行：
```
./image_save.sh
```
会开始记录内录视频并保存为rosbag
