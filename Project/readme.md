[^_^]: # (整个project的readme)

## 说明
### 如何run整个project

#### 1 run on raspi:
在Project/SensorData_Processing/下，run pi_read_sensor.py\
#### 
    pi_read_sensor.py
raspi开始读取sensor的值，并且将值通过 网线 传给NAO
##### 注意：这里要设置要raspi和NAO对应的IP，才能成功通信，在pi_read_sensor.py中以下位置设置：
    
    PI_IP_via_cable = "169.254.234.160"

    NAO_IP_via_cable = "169.254.92.177"

###### 如果出现NAO只连上了实验室内部的Wi-Fi，但是没有连上网线
###### 1 可以在raspi上打开浏览器，输入NAO的IP：
    192.168.0.110（这个是机器人在实验室局域网中的IP，已经被固定了，如果没有，自己固定一个然后记下来）
###### 2 点击图中NETWORK SETTING
<img width="500" height="400" src="https://github.com/heihuhuRay/MasterThesis/blob/master/Project/SensorData_Processing/NAO_ip_2.jpeg"/>

###### 然后，点击wired， 连接有线网，就会出现下面的NAO的通过网线连raspi的IP了

   <img width="500" height="400" src="https://github.com/heihuhuRay/MasterThesis/blob/master/Project/SensorData_Processing/NAO_ip_1.jpeg"/> 

#### 2 run on NAO:
在Project/control_NAO/下，run NAOReadDataintoMemory.py
####
    NAOReadDataintoMemory.py
    # 将raspi上传过来的data存在内存中
##### 如何读取NAO内存中存下来的raspi传来的sensor值
    sensor_data = memProxy.getData("WristForceSensor")
    
#### 3 run on your laptop:
    你自己写好的逻辑，记得windows上可以装naoqi接连NAO，remote运行你电脑上的code
------------------------------------------------------------------
