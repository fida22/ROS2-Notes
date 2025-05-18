# ROS2-Humble Hawksbill
## Index
- Installation
  - Set Locale
  - Setup sources
  - Install ROS2 packages
  - Environment Setup
- Ros2 Workspace Structure
  - creating workspace
  - creating packages
  -c rating ros2 nodes
- Publisher,Subscriber and Node
  - To make a Publisher node
  - To make a subscriber node
- Summary and working of Week1 task
  
<br><br>

# Installation

**4 steps:**

1. **Set Locale**
2. **Setup Sources**
3. **Install ROS2 Packages**
4. **Environment setup**

## 1. Set Locale

—Ensure locale supports UTF-8 

> **locale:** Set of Parameters that defines users language , region etc
> 

> **UTF-8:** character encoding
> 

```bash
local
```

## 2. Setup Sources

a. make sure Ubuntu Universe Respository is enabled<br>
   This allows installing software Packages from the Universal Repository

```bash
sudo apt install software-properties-common

sudo add-apt-repostory-universe
```

![command_Detils](https://github.com/fida22/ROS2-Notes/blob/049754ed702bad2ef61739493ab91ac8c57d0bec/images/command%20details.jpeg)

<br><br>

b. add ROS2 GPG key with apt

```bash
sudo apt update && sudo apt install curl -y
```


|command|Explanation|
|---|---|
|curl|A command line tool to download data from URL|
| -y | Automatically yes to prompts |


--This commands installs curl wthout asking conformation
<br><br>
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
|command|Explanation|
| --- | --- |
| -sSL  | -s silent mode(Dont show progress)  <br>-S show errors if -S is used  <br>-L follow redirects if URL points to another Location |
| https://raw…./ros.key | The URL of the GPG key file for ROS |
| -o/usr/….keyring.gpg | Output save in this location |



--This command: 

  - silently downloads the official ROS GPG key
  - saves it to a secure location so that future ROS package installations can be verified and safe.
  
<br>

c. Add repository to ur source list

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
|command|Explanation|
| --- | --- |
| echo "deb [ ... ] ... main” | This outputs a formatted APT source line for the ROS 2 repository. |
|  sudo tee /etc/apt/sources.list.d/ros2.list | sudo tee is used because /etc/apt/sources.list.d/ros2.list requires root access.<br>This file adds the ROS 2 repository as a new software source. |
|  > /dev/null | Hides the standard output of the tee command, so nothing prints to your terminal. |


<br><br>
## 3. Install ROS2 Packages

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```
<br><br>
## 4. Environment setup

- open bashrc

```bash
gedit ~/.bashrc
```

- add this command in the file

```bash
source /opt/ros/humble/setup.bash
```

— All the commands in bash rc will be executed when we open a terminal 

---

# Ros2 Workspace Structure

NODE —> PACKAGE —>WORKSPACE

<aside>


**Node** is a program , that  has acess to ros functionalities and communications

**Package**: A unit of code organization in ROS 2 that contains one or more nodes, along with configuration files, launch files, etc.

**Workspace**: A directory where ROS 2 packages are built, sourced, and organized.

</aside>

## Creating WorkSpace

```bash
mkdir <workspace_name>
cd <workspace_name>

mkdir src

colcon build
```

—colcon build will fetch the code inside the src and build and install

—inside install there will be a list of files

—we should source the setup.bash file inside the install folder 

 

```bash
source ~/<workspace_name>/install/setup.bash
```

— include it in ~/.bashrc

## Creating Packages

```bash
cd <workspace_name>/src
```

> Format: `ros2` `pkg` `create` `pkg_name` `- - build-type` `ament_python/cmake` `- -dependencies` `name_of_dependecies`   `- -license` `Apache-2.0`
> 

—This will create a folder in workspace with package name with files

- resource
- test
- **<pkg_name>**
- License
- **package.xml**
- setup.cfg
- **setup.py**

## Creating  ROS2 node

```bash
cd <workspace_name>/src/<pkg_name>/<pkg_name>
```

—create a file to write program for the node

Steps while Creating a node:

1. create and write the program using oop
2. spin the node in main
3. install the node
    1. go to setup.py
    2. At last in console_scripts add the node
    
    > “`executable_name` = `file_name`:`function name` ”
    > 
    
    dont forget to put commas inbetween
    
4. colcon build - - symlink-install (in workspace folder)

To run a ros2 node:

> `ros2` `run` `package` `node`
> 

# Publisher, Subscriber and Topic

- The publisher transmits data by publishing messages to a topic.

- The subscriber receives data by subscribing to the same topic.

 This model enables asynchronous and flexible data exchange between nodes in a ROS 2 system.

Example:

![talker&listener](https://github.com/fida22/ROS2-Notes/blob/049754ed702bad2ef61739493ab91ac8c57d0bec/images/Talker%20and%20listner.png)

![rqt_Graph](https://github.com/fida22/ROS2-Notes/blob/049754ed702bad2ef61739493ab91ac8c57d0bec/images/rqt_graph_talker_listner.png)

Here:

- /talker is Publishing to /chatter
- /listner is subscribed to /chatter
- /chatter is the topic

### To make a publisher Node:

inside **init** function:

`self.<publisher_ instance>= self.create_publisher(topic_type, 'topic_name', Queue size)` 

Use timer to call a callback funciton:

`self.timer=self.create_timer(<time in sec,self.<callback_func>)`

Inside the callback function:

store the value to be stored in msg

`self.<publisher_instance>.publish(msg)`

### To make a Subscriber Node:

inside **init** funciton:

`self.<subscriber_instance>=self.create_subscription(Topic_type,"Topic_name",callback_function,Queue size)`

in callback function:

- take msg as parameter
- msg  will contain the data

# Summary and Working of Week1 Task

![ss1.png](https://github.com/fida22/ROS2-Notes/blob/049754ed702bad2ef61739493ab91ac8c57d0bec/images/ss1.png)

![ss2.png](https://github.com/fida22/ROS2-Notes/blob/049754ed702bad2ef61739493ab91ac8c57d0bec/images/ss2.png)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.num_pub=self.create_publisher(Int32,"/number",10)
        self.num=1
        self.timer=self.create_timer(1.0,self.publish_num)
    
    def publish_num(self):
            msg=Int32()
            msg.data=self.num
            self.num_pub.publish(msg)
            self.get_logger().info(f"Number: {self.num}")
            self.num+=1
      

def main(args=None):
    rclpy.init(args=args)
    node=PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':   
    main()
```

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class SubscriberNode(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        self.num_sub=self.create_subscription(Int32,"/number",self.square,10)
    
    def square(self,msg):
        square=msg.data*msg.data
        self.get_logger().info(f'Squared Number: {square}')

def main(args=None):
    rclpy.init(args=args)
    node=SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':   
    main()
```

![ss3.png](https://github.com/fida22/ROS2-Notes/blob/049754ed702bad2ef61739493ab91ac8c57d0bec/images/ss3.png)

![ss4.png](https://github.com/fida22/ROS2-Notes/tree/049754ed702bad2ef61739493ab91ac8c57d0bec/images)
