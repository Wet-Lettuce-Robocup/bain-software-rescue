# Introduction
The algorithm for the project is designed for the RoboCup junior line rescue division, specifically, the rescue section of the course. The following project is based only around this section of the course and takes a more complex approach compared to algorithms our team has used in the past. 
Due to the simple shape of the area (a rectangle with triangles in some corners)
The components of the task at a high level consist of the following: 
 - Detecting the entry of the rescue zone
 - Searching and recognising a rescue capsule
 - Determining the type of rescue capsule
 - Aligning the robot to the position of the rescue capsule
 - Picking up and storing the rescue capsule

# Preliminaries 10MAY
## ROS2
This project required me to learn ROS2, as it is core to the functionality of the robot.
### Messaging
**Topics**: 
- A 'publisher' node which outputs data (usually constantly and indefinitely) 
- One or more 'subscriber' node(s) which 'subscribe' to the 'publisher' to read the data
**Services**
- A 'response' node which provides data (usually retrieved on command) to requesting nodes
- One or more 'request' nodes which request the data from a specific node
**Actions**
- Similar to services with the added 'feedback' feature
- After the 'request' is received by the 'response' node, the 'response' node has the ability to provide additional data before the response (eg. while a task is being completed)
### Packages
- Contains one or more nodes which are arranged into a package
- Packages are built using provided instructions and can be used in anyone's code
- Can be built into different languages enabling multi-language projects
- Primary use is code reuse
### Tools
**Examples**
- Recording — Messages are recorded during execution and stored in a ROS bag
- Logging — Logs can easily be created to view during or after execution
- RViz — Mapping to visualise the robot's position
- Transformation — Automated trigonometry can calculate the position of an object in different frames, provided necessary fixed position data 





# Sources
https://www.youtube.com/watch?v=8aoFndU7jos&t=24s