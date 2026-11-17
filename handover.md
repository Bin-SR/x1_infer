# x1_infer  
## 仿真  

### 启动仿真  

#### 1.启动  
先创建conda环境，`conda activate -n x1_infer python=3.10`, 根据`README.zh_CN.md`的步骤，完成编译，启动仿真`./run_sim.sh`  
注： 在编译或者启动仿真过程中，可能会有一些报错是版本或者缺少module的，只需要pip安装对应版本或者安装对于module即可 
#### 2.当前状态  
#### 2.1 说明  
目前的状态是：启动仿真后，通过数字键盘来控制机器人切换状态，启动后默认为`stand`状态   
```
3 : stand
4 : walk_leg
```
在4状态下，通过wasdqe控制运动方向，x清除所有速度，操作方法：  
`w前进，s后退，a左移，d右移，q逆时针转圈，e顺时针转圈`  

##### 2.2 未来方向与问题
目标，按8，播放动作模仿的模型，动作播放完毕后，自动回到`stand`状态  
问题：从3切换到8的衔接不好，机器人在切换过程中回倒下  

## 实物部署  
在部署之前，可以选择对各个关节电机进行调试检查，如通过上位机检查电机或者桌面联调，具体请查看官方文档  
`https://www.agibot.com.cn/DOCS/OS`, 其中的4. X1整机开发流程
### 启动实物  
在conda环境中，启动对应脚本  
```
cd x1_infer/build
conda activate x1_infer
./run.sh #  启动实物运行脚本，现在还有问题，还不能直接启动，机器人会抽搐
```
**问题**：IMU和DCU的数据不通问题   
解决：跳过DCU,将IMU直接与MiniPC通信，通过IMU官方提供的ROS2的接口  
目前在sim2sim中可以实现imu与机器人的通信：  
(1).在sim2sim中，sim_module发布`/imu/data` 和 `/joint_states`话题，control_module接收这两个话题  
本次改动，禁止sim_module发布`/imu/data`话题，改由IMU官方提供的ROS2接口，  
将原yesense_node.cpp中要发布的数据类型为sensor_msgs::msg::Imu的`/imu_data_ros`话题名改为`/imu/data`  
即代替了原sim_module中imu publisher的作用  
(2).此外，在x1_cfg.yaml中，默认sub订阅通道只有local, 需要增加一个ros2, 来订阅ros2发布的话题  
commit: pass dcu/sim_module, using ros2 interface to read real imu data directly  
关于yesense是IMU官方提供的SDK,后续再整理  
通过网盘分享的文件：
链接: https://pan.baidu.com/s/1Fsq4n-hHaCIW-kLv60YkNQ?pwd=1234 提取码: 1234 复制这段内容后打开百度网盘手机App，操作更方便哦

**问题**：策略模型输出actions，对应的关节电机旋转方向与真实电机方向不一致
解决：通过上位机逐一控制电机，记录每个电机的正反方向，在输出actions和x1_infer的control_module之间，加一层映射  
真实电机与仿真电机方向一致则乘1, 相反则乘-1  


关于编译：  
1.修改.cc文件之后，如何重新编译：  
cd ~/x1_infer  
cmake --build build   --config Release   --target install   --parallel $(nproc)  

2.何时使用./build.sh：  
修改CMakeLists.txt，新增源文件，修改消息类型等  

