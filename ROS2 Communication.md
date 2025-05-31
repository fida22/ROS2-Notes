# ROS2 Communication

# **What DDS and peer to peer communication are?**

DDS (Data Distribution Service) is the middleware used in ROS 2 to manage the discovery of nodes and the transport of data between them.

When a node joins the network, it announces itself, the topics it publishes or subscribes to, and its QoS settings. This process is known as peer-to-peer discovery, meaning nodes directly discover each other without a central server.

Once matching publishers and subscribers are found, DDS establishes direct communication and handles the efficient transportation of data, typically using UDP or TCP, depending on configuration and QoS policies

---

# **Why ROS2 dropped ROS master?**

In ROS 1, whenever a node joined the network, it had to register with the ROS Master. The master acted like a phone book, keeping track of which nodes were publishing or subscribing to which topics, and connecting them accordingly.

However, this centralized approach came with several limitations, which led to ROS 2 adopting DDS (Data Distribution Service) and removing the need for a master node.

Here are the key reasons:

### **1. Single Point of Failure (SPOF)**

In ROS 1, the ROS Master was critical for the whole system to function. If the Master crashed, the entire system would go down, and new nodes wouldn’t be able to join. That’s a major risk for any robot system. In ROS 2, with **peer-to-peer discovery**, nodes find each other directly, so there’s no single point of failure. The system becomes much more robust.

### **2. Scalability Issues in Large Networks**

.As the number of nodes in your robot system grows, the ROS Master starts becoming a bottleneck. The Master has to handle all the registration and connection requests, which can lead to traffic congestion. In contrast, with **DDS** in ROS 2, nodes communicate directly with each other. This peer-to-peer setup means adding more nodes doesn’t slow everything down, and the system can scale up more efficiently.

### **3. Real-Time Communication**

Robots often operate in real-world environments where timing is critical.

ROS 1 used TCP by default and lacked real-time guarantees.

DDS in ROS 2, however, supports:

- Real-time communication
- Low-latency transport
- Priority-based messaging
- Deadline QoS policies

These features enable ROS 2 to be suitable for real-time robotic applications, where missed deadlines could mean failed operations.