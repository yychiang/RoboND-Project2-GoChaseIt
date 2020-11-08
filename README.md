# RoboND-Project2-GoChaseIt

###### tags: `ROS`
# Project 2: Go Chase It!

## 1. Introduction
[**Video**](https://www.youtube.com/watch?time_continue=60&v=pzZKvUSFkgs&feature=emb_logo)


### Project Preview
Here’s a preview of the final outcome of this project. Note that your world and the robot will look different than mine as you will be implementing your own world and robot.

[**Video**](https://www.youtube.com/watch?time_continue=1&v=HxYGmwMp2uw&feature=emb_logo)

## 2. ROS in the Workspace

## 3. Setting up my_robot
The first task in this project is to create the `my_robot` ROS package. Inside `my_robot`, you will store and launch an empty Gazebo world file. As you proceed with the project, you will model and store a robot, as well as replace the empty world file with the world you created in the **Build My World** project. For now, follow these steps to set up `my_robot`.

Note: Do not have more than one `my_robot` instance in the Gazebo world otherwise it would not be able to launch.

### 3.1 Create the `my_robot` Package

**3.1.1- Create and initialize a catkin_ws**

Feel free to skip if you already have a `catkin_ws`.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```

**3.1.2- Navigate to the `src` directory of your `catkin_ws` and create the `my_robot` package:**

```
$ cd ~/catkin_ws/src/
$ catkin_create_pkg my_robot
```
**3.1.3- Next, create a` worlds` directory and a `launch` directory, that will further define the structure of your package:**

```
$ cd ~/catkin_ws/src/my_robot/
$ mkdir launch
$ mkdir worlds
```
### 3.2 Create and Store an Empty Gazebo World File
Inside the `worlds` directory, create and store an empty Gazebo world file. As a reminder, in Gazebo a **world** is a collection of models, such as your robot, together with a specific environment. You can also define several other physical properties specific to this world.

**3.2.1- Create an empty Gazebo world**
An empty world in Gazebo is a simple world, with no objects or models.
```bash
$ cd ~/catkin_ws/src/my_robot/worlds/
$ gedit empty.world
```
**3.2.2- Add the following to `empty.world`**

```xml=
<?xml version="1.0" ?>

<sdf version="1.4">

  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```
The `.world` file uses the XML file format to describe all the elements with respect to the Gazebo environment. The simple world that you are creating here has the following elements:

* `<sdf>`: The base element which encapsulates the entire file structure and content.
* `<world>`: The world element defines the world description and several properties pertaining to that world. In this example, you are adding a ground plane, a light source, and a camera to your world. Each model or property can have further elements that add detail. For example, the `camera` has a `pose` element which defines its position and orientation.
* `<include>`: The include element, along with the `<uri>` element, provide a path to a particular model. In Gazebo there are several models that are available by default.













### 3.3 Create a Launch File
Launch files in ROS allow us to execute more than one node simultaneously, which helps avoid the potentially tedious task of defining and launching several nodes in separate shells or terminals.

**3.3.1- Create the `world.launch` file**

```
$ cd ~/catkin_ws/src/my_robot/launch/
$ gedit world.launch
```

**3.3.2- Add the following to `world.launch`**

```ㄌㄩㄠ=
<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <!-- World File -->
  <arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>

  <!-- Launch Gazebo World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="true" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>

</launch>
```


As in the case of the `.world` file, the `.launch` files are also based on XML. The structure for the launch files has two parts:

* First, define arguments using the `<arg>` element. Each such element will have a `name` attribute and a `default` value.
* Second, include the `world.launch` file from the `gazebo_ros` package. The [**empty_world**](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/launch/empty_world.launch) file includes a set of important definitions that are inherited by the world that we create. Using the world_name argument and the path to your .world file passed as the value to that argument, you will be able to launch your world in Gazebo.


### 3.4 Launch `empty.world`
```bash
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
![](https://video.udacity-data.com/topher/2018/November/5be4b23a_gazebo-empty-world/gazebo-empty-world.png)

It does look a bit bland, but don't worry, there will soon be a different world for your robot to explore!

Follow these steps to set up the my_robot package

![](https://i.imgur.com/jCp6YBP.png)

## 4. Understanding Unified Robot Description Format (URDF)
In the Build My World project, you used the Model Editor tool in Gazebo to model a robot with the Simulation Description Format, or SDF. Now that you are working with ROS, you have to model a robot with the Unified Robot Description Format, or URDF. Both of these formats use XML markup language. We can use a URDF file to define a robot model, its kinodynamic properties, visual elements and even model sensors for the robot. URDF can only describe a robot with rigid links connected by joints in a chain or tree structure. It cannot describe a robot with flexible or parallel links.

A simple robot with two links and a joint can be described using URDF as follows:
```xml=
<?xml version="1.0"?>
<robot name="two_link_robot">
  <!--Links-->
  <link name="link_1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.2"/>
      </geometry>
    </visual>
  </link>
  <link name="link_2">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>
  <!--Joints-->
  <joint name="joint_1" type="continuous">
    <parent link="link_1"/>
    <child link="link_2"/>
  </joint>
</robot>
```
Since we use URDF files to describe several robot and environmental properties, the files tend to be long and tedious. This is why we use Xacro (XML Macros) to divide our single URDF file into multiple Xacro files. While the syntax remains the same, we can now divide our robot description into smaller subsystems.

Since URDF (and Xacro) files are basically XML, they use tags to define robot geometry and properties. The most important and commonly used tags with their elements are described below:

### `<robot>` `</robot>`
This is a top level tag that contains all the other tags related to a given robot.

-----------------------------------------------------------------------------



### `<link>` `</link>`
Each rigid link in a robot must have this tag associated with it.

#### Attributes
**name:** Requires a unique link name attribute.

#### Elements
#### `<visual>` `</visual>`
This element specifies the appearance of the object for visualization purposes.

|Name |Description|
|-----|--------|
|`<origin>`|The reference frame of the visual element with respect to the reference frame of the link.|
|`<geometry>`  |The shape of the visual object.|
|`<material>`|The material of the visual element.|

#### `<collision>` `</collision>`
The collision properties of a link. Note that this can be different from the visual properties of a link, for example, simpler collision models are often used to reduce computation time.

|Name |Description|
|-----|--------|
|`<origin>`|The reference frame of the collision element, relative to the reference frame of the link.|
|`<geometry>`  |See the geometry description in the above visual element.|

#### `<inertial>` `</inertial>`
The inertial properties of the link are described within this tag.
|Name |Description|
|-----|--------|
|`<origin>`|This is the pose of the inertial reference frame, relative to the link reference frame. The origin of the inertial reference frame needs to be at the center of gravity.|
|`<mass>`  |The mass of the link is represented by the value attribute of this element.|
|`<inertia>`|The 3x3 rotational inertia matrix, represented in the inertia frame. Because the rotational inertia matrix is symmetric, only 6 above-diagonal elements of this matrix are specified here, using the attributes ixx, ixy, ixz, iyy, iyz, izz.|

Example snippet for `<link>` tag with important elements:
```xml=
<link name="link_1">
    <inertial>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <mass value="${mass1}"/>
      <inertia ixx="30" ixy="0" ixz="0" iyy="50" iyz="0" izz="50"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://kuka_arm/meshes/kr210l150/visual/link_1.dae"/>
      </geometry>
      <material name="">
        <color rgba="0.75294 0.75294 0.75294 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://kuka_arm/meshes/kr210l150/collision/link_1.stl"/>
      </geometry>
    </collision>
  </link>
```

The `<link>` tag has many more optional elements that can be used to define other properties like color, material, texture, etc. Refer [**this link**](http://wiki.ros.org/urdf/XML/link) for details on those tags.


---------------------------------------------------------------------------

### `<joint>` `</joint>`
This tag typically defines a single joint between two links in a robot. The type of joints you can define using this tag include:
|Name |Description|
|-----|--------|
|Fixed|Rigid joint with no degrees of freedom. Used to weld links together.|
|Revolute|A range-limited joint that rotates about an axis.|
|Prismatic|A range-limited joint that slides along an axis|
|Continuous|Similar to Revolute joint but has no limits. It can rotate continuously about an axis.|
|Planar|A 2D **Prismatic** joint that allows motion in a plane perpendicular to an axis.|
|Floating|A joint with 6 degrees of freedom, generally used for Quadrotors and UAVs|

#### Attributes
**name** Unique joint name

**type** Type of joint


#### Elements
To define a joint, we need to declare the axis of rotation/translation and the relationship between the two links that form the joint.

|Name|Description|
|----|-----------|
|`<origin>`|This is the transform from the parent link to the child link. The joint is located at the origin of the child link.|
|`<parent>`|Name of the Parent link for the respective joint.|
|`<child>`|Name of the child link for the respective joint.|
|`<axis>`|Defines the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints. Fixed and floating joints do not use the axis field.|



Example snippet for `<joint>` tag with important elements:


```xml=
<joint name="joint_2" type="revolute">
  <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
  <parent link="link_1"/>
  <child link="link_2"/>
  <axis xyz="0 1 0"/>
</joint>
```

Other optional elements under the `<joint>` tag can be [**found here**](http://wiki.ros.org/urdf/XML/joint).


There are many more optional tags and attributes that help to define various dynamic and kinematic properties of a robot, along with sensors and actuators. For a full list, refer to the [**ROS documentation on URDF**](http://wiki.ros.org/urdf).


## 5. Robot Basic Setup


Let’s build a basic mobile robot model by creating a URDF file and launch it inside an empty Gazebo world.

We can break the effort down into smaller components - **a robot base**, **wheels**, and **sensors**.

For this model, we will create a cuboidal base with two caster wheels. The caster wheels will help stabilize this model. They aren't always required, but they can help with weight distribution, preventing the robot from tilting along the z-axis.

![](https://video.udacity-data.com/topher/2018/November/5be4b28c_gazebo-robot-base-castors/gazebo-robot-base-castors.png)
[Robot base with two castor wheels](https://video.udacity-data.com/topher/2018/November/5be4b28c_gazebo-robot-base-castors/gazebo-robot-base-castors.png)

### 5.1 Create the URDF File
#### 5.1.1 Create a `urdf` directory in the `my_robot` package

```bash
$ cd ~/catkin_ws/src/my_robot/
$ mkdir urdf
```


#### 5.1.2 Create the robot’s `xacro` file inside the `urdf` directory
```bash
$ cd ~/catkin_ws/src/my_robot/urdf/
$ gedit my_robot.xacro
```
#### 5.1.3 Copy the following code into `my_robot.xacro` file
```xml=
<?xml version='1.0'?>

<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>

    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/> 
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>


    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

  </link>

</robot>

```

We have a single link, with the `name` defined as "chassis", encompassing the base as well as the caster wheels. Every link has specific elements, such as the `inertial` or the `collision` elements. You can quickly review the details of these elements covered in the previous section. The chassis is a cube, whereas the casters are spherical, as denoted by their `<geometry>` tags. Each link (or joint) has an origin (or pose), as well. Every element of that link or joint will have its own origin, which will be relative to the link's frame of reference.

For this base, the casters are included as part of the link for stability. There is no need for any additional links to define the casters, and therefore no joints to connect them. The casters do, however, have friction coefficients defined for them. These `friction` coefficients are set to 0, to allow for free motion while moving.


### 5.2 Launch the Robot
Now that you’ve built the basic robot model, let’s create a launch file to load it inside an empty Gazebo world.

#### 5.2.1 Create a new launch file to load the `URDF` model file
```bash
$ cd ~/catkin_ws/src/my_robot/launch/
$ gedit robot_description.launch
```
#### 5.2.2 Copy the following code into `robot_description.launch` file
```xml
<?xml version="1.0"?>
<launch>

  <!-- send urdf to param server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'" />

</launch>
```

To generate the URDF file from the Xacro file, you must first define a parameter, `robot_description`. This parameter will set a single command to use the [**xacro package**](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File) to generate the URDF from the xacro file.


#### 5.2.3 Update the `world.launch` file created earlier so that Gazebo can load the robot `URDF` model

Add the following to the launch file (after `<launch>`):
```xml
<!-- Robot pose -->
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="0"/>

  <!-- Launch other relevant files-->
  <include file="$(find my_robot)/launch/robot_description.launch"/>

```
**Note: If you have copied your gazebo world from Project 1 then you could skip this step, since you already have `my_robot` in your Gazebo world.**

Add the following to the launch file (before `</launch>`):

```xml=
<!-- Find my robot Description-->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'"/>

  <!-- Spawn My Robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" 
        args="-urdf -param robot_description -model my_robot 
              -x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"/>

```

The [**gazebo_ros package**](http://wiki.ros.org/gazebo_ros) spawns the model from the URDF that `robot_description` helps generate.


### 5.3 Launch

```bash
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
這時，您可以看到Gazebo打開，有這個畫面：
![](https://i.imgur.com/VUkRqaG.png)


**Note**: Launching Gazebo for the first time with a model can take time for everything to load up.


![](https://i.imgur.com/2CdaFCu.png)


## 6. Robot Enhancements

Now that you’ve built a basic model of your robot, enhance it and add wheels. Each wheel is represented as a `link` and is connected to the base link (the chassis) with a `joint`.

![](https://video.udacity-data.com/topher/2018/November/5be4b2be_gazebo-robot/gazebo-robot.png)

### 6.1 Create Wheel `Links`
You will first create the links for each wheel using the specifications given below and add that to your Xacro file. For each wheel, you will have a `collision`, `inertial`, and `visual` element, along with the following properties:
* `link name` - "SIDE_wheel", where the SIDE is either left or right.
* `geometry` - "cylinder" with radius 0.1 and length 0.05.
* `origin` for each element - [0, 0, 0, 0, 1.5707, 1.5707]
* `mass` of each wheel - "5".
* You can use the same inertia values as the ones for the chassis for simplicity:

```C
ixx="0.1" ixy="0" ixz="0"
iyy="0.1" iyz="0"
izz="0.1"
```
(如果不會改，請參考附在第6節最後的程式碼。)
### 6.2 Create `Joints` for the two wheels
Once define the links, you need to create the corresponding joints. The following elements will create a joint between your left wheel (the child link) and the robot chassis (the parent link):
```xml
<joint type="continuous" name="left_wheel_hinge">
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <child link="left_wheel"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
</joint>
```
The `joint type` is set to "continuous" and is similar to a revolute joint but has no limits on its rotation. This means that the joint can rotate continuously. The joint will have its own `axis` of rotation. Also, the joint will have certain `limits` to enforce the maximum "effort" and "velocity" for that joint. The limits are useful constraints in for a real robot and can help in simulation as well. ROS has [good documentation on safety limits.](http://wiki.ros.org/pr2_controller_manager/safety_limits) In addition, the joint will have specific joint `dynamics` that correspond to the physical properties of the joint like "damping" and “friction”.

Add the left wheel joint to your Xacro file. Then use it as a template to create the joint between the right wheel and the chassis.

![](https://i.imgur.com/SnVPbGm.png)

### 6.3 Launch
Excellent work! You can now launch the `empty.world` file to visualize your enhanced robot model in Gazebo.

![](https://i.imgur.com/RLRhAbB.png)

到目前為止，您的URDF檔，也就是`my_robot.xacro`，看起來要像這樣：
```xml=
<?xml version='1.0'?>

<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>

    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/> 
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>


    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

  </link>

<!--Link (left_wheel)-->  
  <link name='left_wheel'>    
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>
    
    <collision name='left_wheel_collision'>
      <origin  xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    
    <visual name='left_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>  
    </visual>
    
  </link> 
<!--Link (right_wheel)-->  
  <link name='right_wheel'>    
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>
    
    <collision name='right_wheel_collision'>
      <origin  xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    
    <visual name='right_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>  
    </visual>
    
  </link>

<!--Joint (left_wheel_hinge)-->
  <joint name="left_wheel_hinge" type="continuous">
    <origin xyz="0 0.125 0" rpy="0 0 0"/>
    <child link="left_wheel" />
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0" />
    <limit effort="10000" velocity="1000"/>
    <joint_properties damping="1.0" friction="1.0" />
  </joint>
<!--Joint (right_wheel_hinge)-->
  <joint name="right_wheel_hinge" type="continuous">
    <origin xyz="0 -0.125 0" rpy="0 0 0"/>
    <child link="right_wheel" />
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0" />
    <limit effort="10000" velocity="1000"/>
    <joint_properties damping="1.0" friction="1.0" />
  </joint>
</robot>
```

再Launch一次：
```bash
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
結果如下：
![](https://i.imgur.com/sA7cGxR.png)

## 7. Robot Sensors

Earlier, you built a very basic robot model by creating its own URDF file. Then, you enhanced the model and added a wheel and a joint on each side of the chassis. Now, it’s time to add sensors to our robot so it can perceive its surroundings. You’ll add two sensors - a **camera** and a **lidar**.

### 7.1 Sensors
**Camera:** Cameras are one of the most common sensors in Robotics. They capture information that is easily interpreted by humans at a high resolution compared to other sensors. Every image captures millions of pixels. To extract depth information from camera images, people have started using stereo cameras. These work like your eyes do and are able to estimate distances to objects.

***Lidar:*** Lidar stands for *Light Detection and Ranging*. It uses arrays of lasers to sense “**point cloud**” models of the environment. By measuring thousands of millions of times per second, lidar builds an accurate model of the world. However, the resolution is not nearly as high as that of a camera.

![](https://video.udacity-data.com/topher/2018/November/5be4b2dd_robot-enhanced-with-sensors/robot-enhanced-with-sensors.png)

### 7.2 Add a Camera
First, add the camera link and a corresponding joint. Open the `my_robot.xacro` file and add a camera sensor based on the following specifications:

* `link name` - "camera"
* `link origin` - "[0, 0, 0, 0, 0, 0]"
* `geometry` - box with size "[0.05, 0.05, 0.05]"
* `mass` - "0.1"
* `box_inertia` - m="0.1" x="0.05" y="0.05" z="0.05"
* `inertia` - ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"
* `joint name` - "camera_joint"
* `joint type` - "fixed"
* `joint axis` - "[0, 1, 0]"
* `joint origin` - "[0.2, 0, 0, 0, 0, 0]"
* `joint parent link` - "chassis"
* `joint child link` - "camera"

As we covered in the previous section, each link should have its own `visual`, `collision` and `inertial` elements.

在`my_robot.xacro`加上以下程式碼：
```xml
<!--Link (camera)-->
  <link name='camera'>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <box_inertia m="0.1" x="0.05" y="0.05" z="0.05"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual name='camera_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </visual>

    <collision name='camera_collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </collision>
  </link>

  <!--Joint (camera)-->
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.225 0 0" rpy="0 0 0"/>
    <child link="camera"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0"/>
  </joint>
```

### 7.3 Add a Lidar
Now, let's add the lidar sensor. ROS supports many different types of [**sensors**](http://wiki.ros.org/Sensors#A2D_range_finders). One of them, that you will use for this robot and for the project, is the Hokuyo rangefinder sensor.
![](https://video.udacity-data.com/topher/2018/November/5be4b2ed_hokuyo-lidar-sensor/hokuyo-lidar-sensor.jpg)

The Hokuyo sensor can be added to your robot model just like the camera sensor, except that you first need to add a mesh file to your robot model. Mesh files define the shape of the object or model you are working with. There are some basic shapes, like the box or cylinder, that do not require a mesh file. However, for more advanced designs, mesh files are necessary. The mesh file should be located in a directory called `meshes` that you can create in your package folder, `my_robot`.

#### 7.3.1 Create `meshes` directory
Let’s create a meshes directory in `my_robot` to hold sensor mesh files:
```bash
$ cd ~/catkin_ws/src/my_robot/
$ mkdir meshes
```
#### 7.3.2 Now, download this [**hokuyo.dae**](https://s3-us-west-1.amazonaws.com/udacity-robotics/hokuyo.dae) file and place it under the meshes directory you just created.

Wondering where I got the mesh file for the Hokuyo sensor? Gazebo shares the mesh files for its [**entire library of models**](http://models.gazebosim.org/).

#### 7.3.3 Add the Hokuyo sensor to `my_robot.xacro`
Here are the Hokuyo lidar sensor specifications:

* `link name` - "hokuyo"
* `link origin` - "[0, 0, 0, 0, 0, 0]"
* `geometry` for `<collision>` - box with size "[0.1, 0.1, 0.1]"
* `geometry` for `<visual>` - filename = “package://my_robot/meshes/hokuyo.dae”
* `mass` - "1e-5"
* `inertia` - ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"
* `joint name` - "hokuyo_joint"
* `joint type` - "fixed"
* `joint axis` - "[0 1 0]"
* `joint origin` - "[0.15, 0, .1, 0, 0, 0]"
* `joint parent link` - "chassis"
* `joint child link` - "hokuyo"

As we covered in the previous section, each link should have its own `visual`, `collision` and `inertial` elements.

在`my_robot.xacro`加上以下程式碼：
```xml
<!--Links (hokuyo)-->
  <link name='hokuyo'>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-5"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual name='hokuyo_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot/meshes/hokuyo.dae"/>
      </geometry>
    </visual>

    <collision name='hokuyo_collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".1 .1 .1"/>
      </geometry>
    </collision>
  </link>

  <!--Joint (hokuyo)-->
  <joint type="fixed" name="hokuyo_joint">
    <origin xyz="0.175 0 .085" rpy="0 0 0"/>
    <child link="hokuyo"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0"/>
  </joint>
```
整個`my_robot.xacro`會是這樣：

```xml=
<?xml version='1.0'?>

<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>

    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/> 
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>


    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

  </link>

<!--Link (left_wheel)-->  
  <link name='left_wheel'>    
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>
    
    <collision name='left_wheel_collision'>
      <origin  xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    
    <visual name='left_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>  
    </visual>
    
  </link> 
<!--Link (right_wheel)-->  
  <link name='right_wheel'>    
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>
    
    <collision name='right_wheel_collision'>
      <origin  xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    
    <visual name='right_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>  
    </visual>
    
  </link>

<!--Joint (left_wheel_hinge)-->
  <joint name="left_wheel_hinge" type="continuous">
    <origin xyz="0 0.125 0" rpy="0 0 0"/>
    <child link="left_wheel" />
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0" />
    <limit effort="10000" velocity="1000"/>
    <joint_properties damping="1.0" friction="1.0" />
  </joint>
<!--Joint (right_wheel_hinge)-->
  <joint name="right_wheel_hinge" type="continuous">
    <origin xyz="0 -0.125 0" rpy="0 0 0"/>
    <child link="right_wheel" />
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0" />
    <limit effort="10000" velocity="1000"/>
    <joint_properties damping="1.0" friction="1.0" />
  </joint>


<!--Link (camera)-->
  <link name='camera'>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <box_inertia m="0.1" x="0.05" y="0.05" z="0.05"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual name='camera_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </visual>

    <collision name='camera_collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </collision>
  </link>

  <!--Joint (camera)-->
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.225 0 0" rpy="0 0 0"/>
    <child link="camera"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0"/>
  </joint>

<!--Links (hokuyo)-->
  <link name='hokuyo'>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-5"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual name='hokuyo_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot/meshes/hokuyo.dae"/>
      </geometry>
    </visual>

    <collision name='hokuyo_collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".1 .1 .1"/>
      </geometry>
    </collision>
  </link>

  <!--Joint (hokuyo)-->
  <joint type="fixed" name="hokuyo_joint">
    <origin xyz="0.175 0 .085" rpy="0 0 0"/>
    <child link="hokuyo"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0"/>
  </joint>


</robot>
```
### 7.4 Launch
Excellent work! You created a robot model and added sensors to it. Now you can test your updated model in Gazebo:
```bash
cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
Now, let's see what your model looks like!
![](https://i.imgur.com/yEBdwBq.png)

要怎麼上色才能變成這樣呢?

![](https://video.udacity-data.com/topher/2018/November/5be4b308_gazebo-robot-color/gazebo-robot-color.png)

The `<material>` element inside a `<gazebo>` element can add color to your robot. For example, to change the left wheel color to green you have to add this to your URDF file:
```ㄌㄩㄠ
<gazebo reference="left_wheel">
    <material>Gazebo/Green</material>
</gazebo>
```
所有的Links都要上色。將以下程式片段放在最後面的`</robot>`之前：
```xml
<gazebo reference="left_wheel">
    <material>Gazebo/Green</material>
</gazebo>

<gazebo reference="right_wheel">
    <material>Gazebo/Green</material>
</gazebo>

<gazebo reference="camera">
    <material>Gazebo/Red</material>
</gazebo>

<gazebo reference="chassis">
    <material>Gazebo/Blue</material>
</gazebo>
```
終於完成了：
![](https://i.imgur.com/sadynmc.png)

**Quiz:**

![](https://i.imgur.com/4eNg4od.png)
![](https://i.imgur.com/UPC2q2A.png)

![](https://i.imgur.com/8IePPJj.png)

## 8. Gazebo Plugins
You added sensors to your robot, allowing it to visualize the world around it! But how exactly does the **camera** sensor takes those images in simulation? How does a **lidar** sensor take laser measurements in simulation? How exactly does your robot **move** in a simulated environment?

URDF in itself can't help with that. However, Gazebo allows for plugins that implement specific use-cases.

### 8.1 Sensor and Actuators Plugins
We will cover the use of three such plugins:
* A plugin for the **camera** sensor.
* A plugin for the **Hokuyo lidar** sensor.
* A plugin for the **wheel joints** actuator.
### 8.2 Add Plugins
Download the [**my_robot.gazebo**](https://s3-us-west-1.amazonaws.com/udacity-robotics/my_robot.gazebo) file, which includes the 3 plugins mentioned above, and place it inside the `urdf` directory of `my_robot`.
### 8.3 Gazebo Plugin Files
Since we have a two-wheeled mobile robot, we will use a plugin that implements a differential drive controller. Let's take a look at how the plugin is defined in the `my_robot.gazebo` file.

```xml
<?xml version="1.0"?>
<robot>

  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <legacyMode>false</legacyMode>
      <alwaysOn>true</alwaysOn>
      <updateRate>10</updateRate>
      <leftJoint>left_wheel_hinge</leftJoint>
      <rightJoint>right_wheel_hinge</rightJoint>
      <wheelSeparation>0.4</wheelSeparation>
      <wheelDiameter>0.2</wheelDiameter>
      <torque>10</torque>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>robot_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishWheelJointState>false</publishWheelJointState>
      <rosDebugLevel>na</rosDebugLevel>
      <wheelAcceleration>0</wheelAcceleration>
      <wheelTorque>5</wheelTorque>
      <odometrySource>world</odometrySource>
      <publishTf>1</publishTf>
      <publishOdomTF>true</publishOdomTF>
    </plugin>
  </gazebo>

  <!-- camera -->
  <gazebo reference="camera">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- hokuyo -->
  <gazebo reference="hokuyo">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
               reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>/scan</topicName>
        <frameName>hokuyo</frameName>
      </plugin>
    </sensor>
  </gazebo>


</robot>

```
libgazebo_ros_diff_drive.so is the shared object file created from compiling the C++ source code. The plugin accepts information specific to your robot's model, such as wheel separation, joint names, and more. Then it calculates and publishes the robot's odometry information to the topics that you specified, like `odom`. In an upcoming section, you will send velocity commands to your robot to move it in a specific direction. This controller helps achieve that result.

If you'd like to understand how this plugin was created, you can refer to its C++ [**source code**](https://bitbucket.org/osrf/gazebo/src/afe08834571835008fa7419f1feba5b7f89b9d62/plugins/DiffDrivePlugin.cc?at=gazebo7&fileviewer=file-view-default).

Gazebo already has several such plugins publicly available. We will utilize the preexisting plugins for the [**camera sensor**](http://gazebosim.org/tutorials?tut=ros_gzplugins#Camera) and the preexisting plugins for the Hokuyo lidar sensor. Both of these are already included in the `my_robot.gazebo` file linked previously.

### 8.4 ROS Communication
You need to define the topics to which each sensor publishes.

For the wheel joints, it's the `cmd_vel` topic.
```xml
<commandTopic>cmd_vel</commandTopic>
```
For the camera, it's the `rgb/image_raw` topic.
```xml
<imageTopicName>rgb/image_raw</imageTopicName>
```
And for the lidar, it's the `scan` topic
```xml
<topicName>/scan</topicName>
```


### 8.5 Import Plugins
Before we proceed to test these sensors and actuators with ROS, you need to make sure that your plugins are imported by your URDF `my_robot.xacro` file as well. Import the sensors plugins by adding the following code to the top of the file (immediately before you define the `robot_footprint` link):
```xml
<xacro:include filename="$(find my_robot)/urdf/my_robot.gazebo" />
```

`my_robot.xacro`修正後如下：
```xml=
<?xml version='1.0'?>

<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

<xacro:include filename="$(find my_robot)/urdf/my_robot.gazebo" />

  <link name="robot_footprint"></link>
  ...
  ...
  ...
```

### 8.6 Next
Now, you’re ready to test these sensors with ROS!
![](https://i.imgur.com/7AYTjYB.png)

### 8.7 補充：TurtleBot3的PlugIn

TurtleBot3的PlugIn，定義在：
```
~catkin_ws/src/turtlebot3/turtlebot3_description/urdf/
```
列出`turtlebot3_waffle_pi.gazebo.xacro`供參考：

```xml=
<?xml version="1.0"?>
<robot name="turtlebot3_waffle_pi_sim" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="laser_visual"  default="false"/>
  <xacro:arg name="camera_visual" default="false"/>
  <xacro:arg name="imu_visual"    default="false"/>

  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>

  <gazebo reference="wheel_left_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="wheel_right_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>500000.0</kp>
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
    <fdir1>1 0 0</fdir1>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="caster_back_right_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="caster_back_left_link">
    <mu1>0.1</mu1>
    <mu2>0.1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <minDepth>0.001</minDepth>
    <maxVel>1.0</maxVel>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor type="imu" name="imu">
      <always_on>true</always_on>
      <visualize>$(arg imu_visual)</visualize>
    </sensor>
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo>
    <plugin name="turtlebot3_waffle_pi_controller" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometrySource>world</odometrySource>
      <publishOdomTF>true</publishOdomTF>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishTf>true</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <leftJoint>wheel_left_joint</leftJoint>
      <rightJoint>wheel_right_joint</rightJoint>
      <wheelSeparation>0.287</wheelSeparation>
      <wheelDiameter>0.066</wheelDiameter>
      <wheelAcceleration>1</wheelAcceleration>
      <wheelTorque>10</wheelTorque>
      <rosDebugLevel>na</rosDebugLevel>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <alwaysOn>true</alwaysOn>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <topicName>imu</topicName>
      <serviceName>imu_service</serviceName>
      <gaussianNoise>0.0</gaussianNoise>
      <updateRate>200</updateRate>
      <imu>
        <noise>
          <type>gaussian</type>
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
    </plugin>
  </gazebo>

  <gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor type="ray" name="lds_lfcd_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>$(arg laser_visual)</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120</min>
          <max>3.5</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_lds_lfcd_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>

<!--link : https://www.raspberrypi.org/documentation/hardware/camera/-->
  <gazebo reference="camera_rgb_frame">
    <sensor type="camera" name="Pi Camera">
      <always_on>true</always_on>
      <visualize>$(arg camera_visual)</visualize>
      <camera>
          <horizontal_fov>1.085595</horizontal_fov>
          <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
          </image>
          <clip>
              <near>0.03</near>
              <far>100</far>
          </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>camera</cameraName>
        <frameName>camera_rgb_optical_frame</frameName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## 9. RViz Basics
![](https://video.udacity-data.com/topher/2018/November/5be4b324_rviz/rviz.png)

### 9.1 RViz
RViz stands for ROS Visualization. RViz is our one-stop tool to visualize all three core aspects of a robot: ***perception***, ***decision-making***, and ***actuation***.

While Gazebo is a physics simulator, RViz can visualize any type of sensor data being published over a ROS topic: *camera images, point clouds, ultrasonic measurements, lidar data, inertial measurements*, and more. This data can be a live stream directly from the sensor or pre-recorded data stored as a ***bagfile***.

You can also visualize live joint angle values from a robot and hence construct a real-time 3D representation of any robot. Having said that, RViz is not a simulator and does not interface with a physics engine. So RViz models neither collisions nor dynamics. RViz is not an alternative to Gazebo, but rather a complementary tool to keep an eye on every single process under the hood of a robotic system.
### 9.2 Running RViz
Since RViz is a ROS package, it requires roscore. In a terminal spin up roscore:
```bash
$ roscore
```
In another terminal, run rviz:
```bash
$ rosrun rviz rviz
```
“rviz” is a node located in an “rviz” package. Once launched, RViz should look something like this:
![](https://video.udacity-data.com/topher/2018/November/5be4b341_rviz-anonotated/rviz-anonotated.png)
The empty window in the center is called **3D view**. This is where you will spend most of your time observing the robot model, the sensors, and other meta-data.

The panel on the left is a list of loaded **displays**, while the one on the right shows different **views** available.

At the top we have a number of useful **tools**. The bottom bar displays information like time elapsed, frames per second, and some handy instructions for the selected tool.
### 9.3 Displays
For anything to appear in the **3D view**, you first need to load a proper display. A display could be as simple as a basic 3D shape or a complex robot model.

Displays can also be used to visualize sensor data streams like 3D point clouds, lidar scans, or depth images.

RViz by default starts with two fixed property fields that cannot be removed: **Global Options** and **Global Status**. One governs simple global settings, while the other detects and displays useful status notifications.

For more information on RViz, check out their official guide [**here**](http://wiki.ros.org/rviz/UserGuide).


## 10. RViz Integration
In this section, you will display your model into RViz and visualize data from the **camera** and **lidar** sensors. You will also **actuate** your robot and drive it around!

### 10.1 Modify robot_description
Start by modifying the `robot_description.launch` file. Open it and add these lines after the first `param` definition.

```xml
<!-- Send fake joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

  <!-- Send robot states to tf -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
```

修改之後如下。8~14行是新加入的。
```xml=
<?xml version="1.0"?>
<launch>

  <!-- send urdf to param server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'" />


  <!-- Send fake joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

  <!-- Send robot states to tf -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>

</launch>
```
Those elements add two nodes - the `joint_state_publisher` and the `robot_state_publisher`.
* `joint_state_publisher`: Publishes joint state messages for the robot, such as the angles for the non-fixed joints.
* `robot_state_publisher`: Publishes the robot's state to tf (transform tree). Your robot model has several frames corresponding to each link/joint. The robot_state_publisher publishes the 3D poses of all of these links. This offers a convenient and efficient advantage, especially for more complicated robots.


### 10.2 Modify `world.launch`
Next, you need to launch RViz along with Gazebo. Open the `world.launch` file and add these elements after the `urdf_spawner` node definition:
```xml
<!--launch rviz-->
<node name="rviz" pkg="rviz" type="rviz" respawn="false"/>
```
This will create a node that launches the package rviz. Let's launch it 🚀

### 10.3 Launch!
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
This time both Gazebo and RViz should launch!

### 10.4 RViz Setup
Setup RViz to visualize the sensor readings. On the left side of RViz, under `Displays`:

* Select `odom` for **fixed frame**
![](https://i.imgur.com/2zvqsrv.png)

* Click the **Add** button and
    * add `RobotModel` and your robot model should load up in RViz.
    ![](https://i.imgur.com/KsRCeAn.png)

    * add `Camera` and select the **Image topic** that was defined in the camera Gazebo plugin (若camera抓不到image，建議參考[附錄B.1](https://hackmd.io/WOFNrsstQOKn9CU6oJXswA?view#%E9%99%84%E9%8C%84B%EF%BC%9A%E8%AE%80%E8%80%85%E5%9B%9E%E6%87%89)升級Gazebo )
![](https://i.imgur.com/p5ypjit.png)

    * add `LaserScan` and select the **topic** that was defined in the Hokuyo Gazebo plugin
    ![](https://i.imgur.com/6eDxHaV.png)

### 10.5 Add Objects
In Gazebo, add a box, sphere or cylinder object in front of your robot. Your robot’s sensors should be able to visualize it. You can check the `Camera` viewer on the bottom left side of RViz to see a picture of the new object. Also, you can see a red `Laser` scan inside the scene, reflecting from your object.
![](https://video.udacity-data.com/topher/2018/November/5be4b362_robot-in-rviz/robot-in-rviz.png)

#### 10.5.1 光達範圍
![](https://i.imgur.com/KaKPKXT.png)
![](https://i.imgur.com/VMaZrPD.png)


### 10.6 Drive Around
While everything above is still running, test the robot’s actuators and drive it around. Open a new terminal window and publish velocity commands to the robot’s wheel actuators:
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" 
```

This command publishes messages to `cmd_vel`, a topic which was defined in the drive controller plugin. The values set for `linear.x` and `angular.z` will enable the robot to start moving in a circle!


Follow these steps to test the robot’s sensors and actuators with RViz:
![](https://i.imgur.com/kcjbCm6.png)

## 11. House your Robot
So far, you created a robot model from scratch, added sensors to it to visualize its surroundings, and developed a package to launch the robot in a simulated environment. That's a real accomplishment!

But you haven’t yet placed the robot in an environment. Let’s house it inside the world you built in **Build My World** project.
### 11.1 Adding the World File
Copy the `<yourname>.world` file from the `world` directory of the **Build My World** project and paste it in the `worlds` directory of `my_robot`.

Inside your package’s `worlds` directory you should now see two files - the `empty.world` that we created earlier and the `<yourname>.world` file that you just added.

Feel free to delete the `empty.world` file. We won’t need it anymore.
### 11.2 Launch the World
Edit the `world.launch` file and add a reference to the `<yourname>.world` file that you just added. To do so, open the `world.launch` file and edit this line:

```xml
<arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>
```
Replace it with this:
```xml
<arg name="world_file" default="$(find my_robot)/worlds/<yourname>.world"/>
```
### 11.3 Launch!
Now, that you’ve added your world file to the my_robot package, let’s launch and visualize our robot inside our home.
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
![](https://video.udacity-data.com/topher/2018/November/5bfc9382_screen-shot-2018-11-26-at-4.33.30-pm-2/screen-shot-2018-11-26-at-4.33.30-pm-2.png)

### 11.4 Initialize the Robot’s Position and Orientation
As you can see, my robot’s initial position is outside of my world! You might face the same problem. I have to change my robot’s initial pose: its position and orientation. This can be done through editing the `world.launch` file:

```xml
 <!-- Robot pose -->
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="0"/>
```

The best way to figure out these numbers is to change the robot’s position and orientation within Gazebo, record its pose, and then update the launch file.

![](https://video.udacity-data.com/topher/2018/November/5bfc93a2_screen-shot-2018-11-26-at-4.29.06-pm-2/screen-shot-2018-11-26-at-4.29.06-pm-2.png)

![](https://i.imgur.com/A2iYVVp.png)


## 12. Setting up ball_chaser
The second major task in this project is to create the `ball_chaser` ROS package. Within this package, you'll analyze the image captured by the camera to determine the position of a white ball. Then you’ll drive the robot toward the ball. The nodes in `ball_chaser` will communicate with the `my_robot` package by subscribing to the robot camera sensor and publishing to the robot’s wheel joints.

### 12.1 Package Nodes
The `ball_chaser` package will have two C++ nodes: the `drive_bot` and `process_image`
* `drive_bot`: This server node will provide a `ball_chaser/command_robot` **service** to drive the robot around by controlling its linear x and angular z velocities. The service will **publish** a message containing the velocities to the wheel joints.
* `process_image`: This client node will **subscribe** to the robot’s camera images and analyze each image to determine the position of the white ball. Once ball position is determined, the client node will request a service to drive the robot either left, right or forward.

Now, follow along with the steps to set up `ball_chaser`.
### 12.2 Create the `ball_chaser` Package
#### 12.2.1 Navigate to the `src` directory of your `catkin_ws` and create the `ball_chaser` package:
We will be writing nodes in C++. Since we already know in advance that this package will contain C++ source code and messages, let’s create the package with those dependencies:
```bash
$ cd ~/catkin_ws/src/
$ catkin_create_pkg ball_chaser roscpp std_msgs message_generation
```
#### 12.2.2 Next, create an `srv` and a `launch` folder, which will further define the structure of your package:
```bash
$ cd ~/catkin_ws/src/ball_chaser/
$ mkdir srv
$ mkdir launch
```
Remember, `srv` is the directory where you store **service** files and `launch` is the directory where you store **launch files**. The `src` directory where you will store C++ programs is created by default.

### 12.3 Build the Package
```
$ cd ~/catkin_ws/
$ catkin_make
```
Now you should be ready to write some code!
![](https://i.imgur.com/oWWpCmR.png)

## 13. ROS Node: drive_bot

This server node provides a `ball_chaser/command_robot` **service** to drive the robot around by setting its linear x and angular z velocities. The service server **publishes** messages containing the velocities for the wheel joints.

After writing this node, you will be able to request the `ball_chaser/command_robot` service, either from the terminal or from a client node, to drive the robot by controlling its linear x and angular z velocities.

### 13.1 Reference
The `drive_bot.cpp` node is similar to the `arm_mover.cpp` node that you already wrote. Both nodes contain a ROS **publisher** and **service**. But this time, instead of publishing messages to the arm joint angles, you have to publish messages to the wheels joint angles. Please refer to the `arm_mover.cpp` node before you begin coding the `drive_bot.cpp` node.

### 13.2 ROS Service File
#### 13.2.1 Write the `DriveToTarget.srv` file
Create a `DriveToTarget.srv` file under the `srv` directory of `ball_chaser`. Then, define the `DriveToTarget.srv` file as follows:
**Request**:

* `linear_x` type **float64**
* `angular_z` type **float64**

**Response**:
* `msg_feedback` type **string**

```bash
$ cd ~/catkin_ws/src/ball_chaser/srv
$ gedit DriveToTarget.srv
```

`DriveToTarget.srv`如下：
```
float64 linear_x
float64 angular_z
---
string msg_feedback
```

#### 13.2.2 Test `DriveToTarget.srv`
After writing the service file, test it with ROS. Open a new terminal and execute the following:
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rossrv show DriveToTarget
```
You should receive this response:
```
[ball_chaser/DriveToTarget]:
float64 linear_x
float64 angular_z
---
string msg_feedback
```
### 13.3 `drive_bot.cpp` Node
Now it’s time to write the `drive_bot.cpp` server node that will provide the `ball_chaser/command_robot` service. Create the script under the src directory of your `ball_chaser` package. It might be a bit challenging to write this script from scratch, thus I am providing you with some hints.

Attached below is a program that will continuously publish to the robot `/cmd_vel` topic. This code will drive your robot forward:
```C=
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    while (ros::ok()) {
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        // Set wheel velocities, forward [0.5, 0.0]
        motor_command.linear.x = 0.5;
        motor_command.angular.z = 0.0;
        // Publish angles to drive the robot
        motor_command_publisher.publish(motor_command);
    }

    // TODO: Handle ROS communication events
    //ros::spin();

    return 0;
}
```
Take a look at this program and try to understand what is happening. Then, copy it to `drive_bot.cpp`, and make the necessary changes to define a `ball_chaser/command_robot` service.


修改過後，`drive_bot.cpp`變成：
```c=
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Publish angles to drive the robot
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Velocities set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
    /*
    while (ros::ok()) {
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        // Set wheel velocities, forward [0.5, 0.0]
        motor_command.linear.x = 0.5;
        motor_command.angular.z = 0.0;
        // Publish angles to drive the robot
        motor_command_publisher.publish(motor_command);
    }
    */

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
```
### 13.4 Edit CMakeLists.txt
After you write the server node in C++, you’ll have to add the following dependencies:
* Add the `add_compile_options` for C++ 11 dependency, this step is optional and depends on your code **(Line 6)**
* Add the `add_service_files` dependency which defines the DriveToTarget.srv file **(Line 64~67)**
* Add the `generate_messages` dependency **(Line 83~86)**
* Add `include_directories` dependency **(Line 132~135)**
* Add the `add_executable` **(Line 153)**, `target_link_libraries` **(Line 171)**, and `add_dependencies` **(Line 164)** dependency for your `drive_bot.cpp` script


```cmake=
cmake_minimum_required(VERSION 3.0.2)
project(ball_chaser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_service_files(
   FILES
   DriveToTarget.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

generate_messages(
   DEPENDENCIES
   std_msgs
)



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ball_chaser
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ball_chaser.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ball_chaser_node.cpp)
add_executable(drive_bot src/drive_bot.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drive_bot ball_chaser_generate_messages_cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(drive_bot ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ball_chaser.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```
### 13.5 Build Package
Now that you’ve included specific instructions for your drive_bot.cpp code in CMakeLists.txt file, compile it with:
```bash
$ cd ~/catkin_ws/
$ catkin_make
```

### 13.6 Test `drive_bot.cpp`
To test if the service you wrote is working, first launch the robot inside your world. Then call the `/ball_chaser/command_robot` service to drive the robot forward, left, and then right.

#### 13.6.1 Launch the robot inside your world
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

#### 13.6.2 Run the drive_bot node
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun ball_chaser drive_bot
```
#### 13.6.3 Request a ball_chaser/command_robot service
Test the service by requesting different sets of velocities from the terminal.

Open a new terminal while all the nodes are running and type:
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash

$ rosservice call /ball_chaser/command_robot "linear_x: 0.5
angular_z: 0.0"  # This request should drive your robot forward

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.5"  # This request should drive your robot left

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: -0.5"  # This request should drive your robot right

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.0"  # This request should bring your robot to a complete stop

```
### 13.7 Launch Files
Let’s add the drive_bot node to a launch file. Create a `ball_chaser.launch` file under the launch directory of your ball_chaser package and then copy this code to it.

```bash
$ cd ~/catkin_ws/src/ball_chaser/launch/
$ gedit ball_chaser.launch
```

```xml
<launch>

 <!-- The drive_bot node -->
  <node name="drive_bot" type="drive_bot" pkg="ball_chaser" output="screen">
  </node>

</launch>
```
This code will launch your `drive_bot` node, which is contained in the `ball_chaser` package. The server node outputs all the logs to the terminal window.

![](https://i.imgur.com/LLztCEK.png)


## 14. Model a White Ball
Before you proceed to code the `process_image` client node, you have to model a white ball and place it in your Gazebo World scene.

After modeling the white ball, you'll control its position in Gazebo by placing it at different positions in front of the robot’s camera. The `process_image` client node will be responsible for analyzing the robot’s image and requesting services from the server `drive_bot` node to drive the robot towards it.

Now, let’s go ahead and model the white ball using the Model Editor tool in Gazebo!

### 14.1 Model Editor
Here’s a reminder of how to open the model editor:
```bash
$ gazebo # then Edit-> Model Editor
```
### 14.2 Insert Sphere
Under the **simple shapes** menu of the Model Editor tool, click on a sphere and insert it anywhere in the scene.
### 14.3 Edit Size
Double click on the sphere, and change its radius to **0.1** both in `Visual` and `Collision`.
### 14.4 Change Color
To change the ball’s color to white, set its **Visual** Ambient, Diffuse, Specular, and Emissive `RGBA` values to 1.
![](https://video.udacity-data.com/topher/2018/November/5be4b3cf_model-a-ball/model-a-ball.png)

### 14.5 Save
Save the white ball model as `my_ball` under the `~` directory (home directory). Then exit the Model Editor tool and go back to the Gazebo main world.
### 14.6 Insert Ball
Now that you are back in the Gazebo main world, you can click on “Insert” and drop the white ball anywhere in the scene.
![](https://video.udacity-data.com/topher/2018/November/5be4b3e1_white-ball-empty-scene/white-ball-empty-scene.png)

### 14.7 Relaunch Nodes
Now that you modeled the white ball, relaunch the nodes inside `world.launch`. Then verify that you can insert a `my_ball` anywhere inside your world.

```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
![](https://video.udacity-data.com/topher/2018/November/5bfc92d1_screen-shot-2018-11-26-at-4.35.40-pm-2/screen-shot-2018-11-26-at-4.35.40-pm-2.png)

### 14.8 Save
Place the white ball anywhere outside of your building structure, so that the robot would not see it. Then, save a copy of this new world under `~/catkin_ws/src/my_robot/worlds` by replacing your old `<yourname>.world` file. Whenever you launch this newly saved world you should be able to see your building environment, in addition, the white ball.

![](https://video.udacity-data.com/topher/2018/November/5bfc9318_screen-shot-2018-11-26-at-4.35.52-pm-2/screen-shot-2018-11-26-at-4.35.52-pm-2.png)

![](https://i.imgur.com/Qioaq2M.png)

## 15. ROS Node: process_image
The second node that you’ll write in this project is the `process_image` node. This client node will **subscribe** to the robot’s camera images and analyze them to determine the position of the white ball. Once the ball position is determined, the client node will request a service from the `drive_bot` server node to drive the robot toward the ball. The robot can drive either left, right or forward, depending on the robot position inside the image.

After you write this node, place the white ball in front of the robot’s camera. If everything works, your node should analyze the image, detect the ball’s position, and then request a `ball_chaser/command_robot` service to drive the robot towards the ball!
### 15.1 Reference
The `process_image.cpp` client node is similar to the `look_away.cpp` client node that you wrote in this lesson. Both nodes contain a ROS subscriber and client. Please review the `look_away.cpp` node before you start coding the `process_image.cpp` node.
### 15.2 Analyzing the Images
To identify the ball’s **presence** and **position** inside the image, you will use a simple approach. First, search for white pixels inside the array image. Since the ball is the only object in the world that is white, white pixels indicate the ball’s presence. Then, once you find that the ball, identify its position with respect to the camera - either the left, middle, or right side of the image.
![](https://video.udacity-data.com/topher/2018/November/5be4b428_analyze-image/analyze-image.png)

You’ll have to subscribe to the `/camera/rgb/image_raw` topic to get instantaneous images from the robot’s camera. Inside the callback function, retrieve the image by reading the image data message. The image message contains many fields, such as the image height, data and more. Check out the complete [ROS`sensor_msgs/Image` documentation](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html). Now that you have the image messages, you have to loop through the image data. For each pixel compare it to a value of 255 indicating a bright white pixel, if this pixel is found try to identify in which section of the image it fall either left, mid, or right. Then, request a service to drive toward that direction.
### 15.3 Write `process_image.cpp`
Now it’s time to write the `process_image.cpp` client node. This node will analyze the image and request services to drive the robot. Create the source code file within the `src` directory of your `ball_chaser` package. It might be a bit challenging to write this program from scratch, thus I am providing you with some hints. Attached below is a piece of the complete code with multiple hints to help you finish the implementation.

```c=
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
```
Copy this code to `process_image.cpp`, and make the necessary changes.

**解答：**
```c=
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
	
    // Call the command_robot service and pass the requested joint angles
    if (!client.call(srv)){
        ROS_ERROR("Failed to call service command_robot");
    }	
}

void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_counter = 0;
    int front_counter = 0;
    int right_counter = 0;
	
    // TODO: 
    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i += 3) {
        int position_index = i % (img.width * 3) / 3;
	
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            if(position_index <= 265) {
		left_counter += 1;                
            }
            if(position_index > 265 && position_index <= 533) {
		front_counter += 1;               
            }
            if(position_index > 533) {
		right_counter += 1;                
            }
	}
    }
		
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    vector<int> position_counter{left_counter, front_counter, right_counter};
    int where_to_move = *max_element(position_counter.begin(), position_counter.end());

    // Depending on the white ball position, call the drive_bot function and pass velocities to it.
    // Request a stop when there's no white ball seen by the camera.
    if (where_to_move == 0){
        drive_robot(0.0, 0.0); // This request brings my_robot to a complete stop
    }
    else if (where_to_move == left_counter) {
	drive_robot(0.0, 0.5);  // This request should drive my_robot left
    }
    else if (where_to_move == front_counter) {
        drive_robot(0.5, 0.0);  // This request drives my_robot robot forward
    }
    else if (where_to_move == right_counter) {
        drive_robot(0.0, -0.5); // This request drives my_robot right
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
```
### 15.4 Edit CMakeLists.txt
In addition to all the dependencies you added earlier for `drive_bot.cpp`, these are the dependencies that you should add for `process_image.cpp`:

* Add `add_executable`
* Add `target_link_libraries`
* Add `add_dependencies`


```cmake
add_executable(process_image src/process_image.cpp)
add_dependencies(process_image ball_chaser_generate_messages_cpp)
target_link_libraries(process_image ${catkin_LIBRARIES})
```

### 15.5 Build Package
Now that you’ve included specific instructions for your `process_image.cpp` code in `CMakeLists.txt`, compile it with:
```bash
$ cd ~/catkin_ws/
$ catkin_make
```
### 15.6 Launch File
Edit the `ball_chaser.launch` file saved under `~/catkin_ws/src/ball_chaser/launch` and add the `process_image` node to it.

```xml
<!-- The process_image node -->
  <node name="process_image" type="process_image" pkg="ball_chaser" output="screen">
  </node>
```
Now, launching this file should run the `drive_bot` and `process_image`!
### 15.7 Test `process_image`
To test if the code you just wrote is working as expected, first launch the robot inside your world and then run both the `drive_bot` and `process_image` nodes.
#### 15.7.1 Launch the robot inside your world
This can be done by launching the `world.launch` file:
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
#### 15.7.2 Run `drive_bot` and `process_image`
This can be done by executing `ball_chaser.launch`:
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
#### 15.7.3 Visualize
To visualize the robot’s camera images, you can subscribe to camera RGB image topic from RViz. Or you can run the rqt_image_view node:
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view  
```
Now place the white ball at different positions in front of the robot and see if the robot is capable of chasing the ball!
![](https://video.udacity-data.com/topher/2018/November/5bfc9289_screen-shot-2018-11-26-at-4.38.15-pm-2/screen-shot-2018-11-26-at-4.38.15-pm-2.png)

![](https://i.imgur.com/BUVWHEd.png)

#### 15.7.4 簡化程式開啟的步驟
以上的步驟，開三個視窗下命令，非常麻煩。我們修改`.bashrc`，下次開啟新視窗之後只需要輸入數字，不用再記憶及輸入冗長的命令。

```bash
$ gedit ~/.bashrc
```
在`.bashrc`最後面加入以下命令：
```bash
unset selection
echo "1) launching the world.launch"
echo "2) Run drive_bot and process_image"
echo "3) Run rqt_image_view"
echo "4) Do nothing"
echo "Please choose: " 
read selection


if [ "$selection" '==' "1" ];
then
    cd ~/catkin_ws/
    source devel/setup.bash
    roslaunch my_robot world.launch
elif [ "$selection" '==' "2" ];
then
    cd ~/catkin_ws/
    source devel/setup.bash
    roslaunch ball_chaser ball_chaser.launch
elif [ "$selection" '==' "3" ];
then
    cd ~/catkin_ws/
    source devel/setup.bash
    rosrun rqt_image_view rqt_image_view  
else
    # Do nothing #
    echo "Don't do anything ..."
fi
```
## 16. Project Description
### 16.1 Summary of Tasks
In this project, you should create two ROS packages inside your `catkin_ws/src`: the `drive_bot` and the `ball_chaser`. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:
#### 16.1.1 `drive_bot`:
* Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
* Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
* House your robot inside the world you built in the **Build My World** project.
* Add a white-colored ball to your Gazebo world and save a new copy of this world.
* The `world.launch` file should launch your world with the white-colored ball and your robot.

#### 16.1.2 `ball_chaser`:
* Create a `ball_chaser` ROS package to hold your C++ nodes.
* Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
* Write a `process_image` C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
* The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.

The robot you design in this project will be used as a base model for all your upcoming projects in this Robotics Software Engineer Nanodegree Program.
### 16.2 Evaluation
Once you finish designing your robot and building the nodes, check the [Project Rubric](https://review.udacity.com/#!/rubrics/2397/view) to see if it meets the specifications. If you meet the specifications, then you are ready to submit!

If you do not meet specifications, keep working and discussing with your fellow students and mentors.

### 16.3 Submission Folder
Your submission should follow the directory structure and contain all the files listed here:
```
 .Project2                          # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── <yourworld>.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──                     
```

### 16.4 Ready to submit your project?
Click on the "Submit Project" button and follow the instructions to submit!
### 16.5 Project Submission Checklist
**Before submitting your project, please review and confirm the following items.**

*  I am confident all rubric items have been met and my project will pass as submitted.

*  Project builds correctly without errors and runs.

*  All required functionality exists and my project behaves as expected per the project's specifications.

**Once you have checked all these items, you are ready to submit!**

## [模擬結果](https://youtu.be/gax-skPuk1E)



