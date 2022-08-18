---
title: Lab 1
subtitle: Setting up the development environment and basic ROS
layout: page
---

# The Robotic Operating System

In this class, we are going to be using the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Introduction)). ROS is a sophisticated and open source framework of software libraries, simulators, and development tools that help us to more quickly and effectively build robot systems. Why are we learning ROS? Well, ROS has become the de-facto standard for the robotics community and is one of the enablers of the current robotic revolution, so what we learn in this class (a small portion of ROS) will be valuable regardless of what direction your future takes you. Other reason to use ROS is that you will join an active community that constantly invents new robots (in the hundreds), provides new packages to cover the latest emerging algorithms and paradigms, and give us tons of documentation, answers, and support (thousands of contributors!). Take a look at the wide range of robots that use ROS below (or the [whole list](https://robots.ros.org)):

<div class="columns is-centered">
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_a.jpg">
        </figure>
        <p class='has-text-centered'>Prosthetic Robot</p>
    </div>
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_b.jpg">
        </figure>
        <p class='has-text-centered'>Ground Robot</p>
    </div>
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_c.jpg">
        </figure>
        <p class='has-text-centered'>Aerial Robot</p>
    </div>
    <div class="column is-3">
        <figure class="image is-square">
            <img src="../images/lab1/robot_d.png">
        </figure>
        <p class='has-text-centered'>Underwater Robot</p>
    </div>
</div>
---- 

# Learning Objectives
This lab is structured into two sections. The first shows you how to set up the Docker container that we will be using for all labs in this course. This section aims to teach how to:
* Install a Docker container that includes ROS.
* Run basic ROS commands in the Docker container.

The second part of the lab aims at teaching you the basics of building ROS-based systems and tooling. More specifically it aims to teach you how to:
* Compile ROS code.
* Launch ROS files.
* Use ROS tools.

---- 

# Getting a Machine with ROS
ROS is mainly developed on Linux-Ubuntu. Therefore, you first need to install a Docker container that will allow you to start developing in ROS. A *Docker container* is a package that contains all dependencies required to run an application. The set-up process is outlined below:

## Docker Setup:
Open a new terminal (Terminal 1) and enter the following commands:
```bash
$ cd ~
$ git clone https://github.com/adwiii/cs4501-robotics-docker.git
$ cd cs4501-robotics-docker
```

Next, clone the class repository and build the container:
```bash
$ git clone https://github.com/less-lab-uva/CS4501-Labs.git
$ docker-compose up --build
```

If successful, the output should look like this:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-16by9">
        <img src="../images/lab1/install1.png">
        </figure>
    </div>
</div>

To access GUI applications, follow [this link](http://localhost:8080/vnc.html) and click "Connect".

Keep Terminal 1 running. Now, open a **new terminal** (Terminal 2) and enter the following commands to connect to your Docker container:
```bash
$ cd ~/cs4501-robotics-docker
$ docker-compose exec ros bash
```

## Test your Docker Container:

<p> </p>

To test that the Docker container and ROS are working properly, enter the following command into Terminal 2:

```bash
$ roscore
```

The terminal should output something similar to what you see in the picture below:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-16by9">
        <img src="../images/lab1/install3.png">
        </figure>
    </div>
</div>


---- 
# Checkpoint 1

For this checkpoint, you should be able to do the following:
1. Launch the Docker container
2. Run `roscore` in the terminal

{% include notification.html
message="Extra: Here is a bit more on the [history](https://www.ros.org/history/) of ROS, and a short interesting [video](https://www.youtube.com/watch?v=Dm7HnQb8n9Y) on who uses ROS."
icon="false"
status="is-success" %}
---- 

# Our First Code in ROS

Robots are complex systems, made of many software and electro-mechanical components, all interacting as the robot senses and acts on the world. 
ROS helps us to manage that complexity through abstractions, infrastructure, and tools. 
To start getting a sense of what ROS can do for us and how it operates, in this lab, we will build, run, and inspect some simple ROS code to control a simple rocket operating in simulation. 

## Rocket Ship

Let's assume we have been given the software used to fly a rocket to the moon. The rocket's software is written using ROS. Our job is to use the ROS tools to compile, launch, and then monitor this rocket on its journey to the moon.

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-5by4">
        <img src="../images/lab1/rocket1.png">
        </figure>
    </div>
</div>

## Downloading the Code

To start the lab, we need to make sure the lab code is up to date. In your Docker container (Terminal 2), enter the following:

```bash
# Change to labs directory
$ cd ~/Desktop/CS4501-Labs
# Pull the latest code
$ git pull
```

## Structuring and Building ROS Code

To start, we first need to compile the rocket's ROS code. ROS code is organized into workspaces. 
A ROS workspace is a folder where you modify, build, and install ROS code. 
Workspaces are one way to address the complexity in the development of large systems 
by providing a standard structure where developers and tools know where things are located.
ROS even has community [conventions](https://www.ros.org/reps/rep-0128.html) on such structures.

Let's now build the code in the workspace. In your Docker container (Terminal 2), run the following command:

```bash
# Change the directory to the ROS workspace
$ cd ~/CS4501-Labs/lab1_ws/
# Source and compile the ROS code
$ source /opt/ros/melodic/setup.bash
$ catkin build
```

```catkin``` is the name of ROS build system, the system that gathers source code and generates a target, similar to CMake but with extra support to handle the dependencies between heterogenous packages that usually coexist in robotic systems. Check [catkin design](http://wiki.ros.org/catkin/conceptual_overview) for more information.

If you get no errors, you have successfully build your rocket's ROS code.  ** When you change your code and want to run it, you should build it first.**

Next, let's run our rocket software. 
The rocket software consists of three software components. Let's start each component by opening  **three new terminals**, then enter the following:

### Terminal 2:
We need to start the ROS core, a key piece of ROS infrastructure  which provides  rich standarized mechanisms for processes to communicate.
```bash
# Start ROS in the background
$ roscore
```

### Terminal 3:

```bash
# Start the Docker container
$ docker-compose exec ros bash
# Update your environment variables so that ROS knows where your workspace is
$ source ~/CS4501-Labs/lab1_ws/devel/setup.bash
# Run the rocket-ship source code
$ rosrun rocketship rocket_engine.py
```

The `rocket_engine` software  controlling the power to the rocket engine is now running.  You will see no further messages printed in this terminal. 

### Terminal 4:

```bash
# Start the Docker container
$ docker-compose exec ros bash
# Update your environment variables so that ROS knows where your workspace is
$ source ~/CS4501-Labs/lab1_ws/devel/setup.bash
# Run the rocket-ship source code
$ rosrun rocketship main_controller
```

The `main_controller` which commands the `rocket_engine` to go at certain velocity is now running. The main controller should respond with the following message:

```bash
>>> [ INFO] [...]: Rocket Ready for Countdown
```

Now let's get ready to launch the rocket! Start the countdown using terminal 5.

### Terminal 5:

```bash
# Start the Docker container
$ docker-compose exec ros bash
# Update your environment variables so that ROS knows where your workspace is
$ source ~/CS4501-Labs/lab1_ws/devel/setup.bash
# Run the rocket-ship source code
$ rosrun rocketship countdown
```

The `countdown` component gives the command to the main_controller to initiate the launch sequence.



If everything went correctly in terminal 4, you should see a countdown (after 5 seconds) and subsequent velocity commands as shown below:

```bash
>>> [ INFO] [...]: Countdown: 10s
>>> [ INFO] [...]: Countdown: 9s
>>> ...
>>> [ INFO] [...]: Requested Velocity: 0.076450m/s
>>> [ INFO] [...]: Requested Velocity: 0.077994m/s
>>> ...
```

That's awesome. We just launched the rocketship using ROS code! 

You can stop execution of your code using the key-combination:
```bash
$ CTRL-C
```

But let's dive a bit deeper so that we start to understand what is going on behind the scenes. To launch this rocket, we needed four terminals. 
to launch the `roscore`, the rocket_engine, the main_controller software, and the countdown software. 
Using this information, we can start to understand some of the system organization as shown below: 

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-16by9">
        <img src="../images/lab1/rocket2.png">
        </figure>
    </div>
</div>

## Roslaunch

Having to run many different software components is common in robotics, but doing it manually by setting terminals as we have done is not practical, especially when deploying a system requires hundreds of processes to be run in specific ways.  Another way that ROS helps to address system complexity is by making it easier to deploy multiple software components through **launch files**.  Another benefit of a ROS launch file is that it automatically checks if a `roscore` is running, and if not, it starts one. 

Let's create a launch file to run all the software components of our rocket. First, open the base rocket.launch file provided to you. To do this, open the `~/CS4501-Labs/lab1_ws/src` directory in your preferred IDE.

Next, navigate to the `rocketship/launch/` directory and open the `rocket.launch` file. You will find an empty launch file with xml code as shown below:

```xml
<?xml version="1.0"?>
<launch>
   <!-- Add your code here -->
</launch>
```

Let's add each of the different software components into the launch file. ROS calls each software component a node. Add the following lines between `<launch>`and `</launch>`.

```xml
<node pkg="rocketship" type="rocket_engine.py" name="RocketEngineNode"></node>
<node pkg="rocketship" type="main_controller" name="MainControllerNode" output="screen"></node>
<node pkg="rocketship" type="countdown" name="CountDownNode"></node>
```

Take a second to explore what is inside the launch file. By using a launch file, ROS knows to start `roscore` automatically and how to start three software modules (nodes) `rocket_engine`, `main_controller`, and `countdown`. Each of the nodes belongs to the rocketship package (pkg). We want the `main_controller` to output to screen. Save the file and go back to your terminal. Let's launch the rocket. Close all terminals (except for Terminal 1) and open a new one. To launch the rocket we need to run:

```bash
# Start the Docker container
$ docker-compose exec ros bash
# Update your environment variables so that ROS knows where your workspace is
$ source ~/CS4501-Labs/lab1_ws/devel/setup.bash
# Run the countdown code
$ roslaunch rocketship rocket.launch
```

This time you should see the rocket state that it is ready for launch. The countdown should automatically begin, and the rocket should launch! 

## Using ROS Tools

Next, let's use some of the built-in ROS tools to get a better understanding of how our rocketship works. Let's start by finding what kind of communication is occurring between the software modules in this rocketship. 

Leave the rocketship running and open another terminal. In the new terminal, run the following command. (Note we will discuss ROS nodes and topics in more detail in the next lecture)

```bash
# Start the Docker container
$ docker-compose exec ros bash
# List all the communication channels (ROS topics)
$ rostopic list
```

You will see the following output: 
```bash
>>> /cmd_vel
>>> /countdown
>>> /launch_abort
>>> ...
```

This shows us the names of all the created communication channels (topics) that exist between processes. 

The next thing we might want to do is to display the data (messages) being sent on any one of these communication channels. Lets, for instance, display the rockets commanded velocity `/cmd_vel`:

```bash
# Display the data on the ROS communication channel (ROS messages)
$ rostopic echo /cmd_vel
>>> data: 1684.0
>>> ---
>>> data: 1684.0
>>> ...
```

Notice how these numbers match the numbers being printed by the rocketship software:
```
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s›
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
```

## RQT Graph

Our rocketship code is reasonably simple, and keeping track of what is going on in our heads is manageable. 
However, when you get to work on larger systems, it sometimes helps to **visualize the software modules and their communication** altogether. One way to do that is to use `rqt_graph`. `rqt_graph` provides a GUI plugin for visualizing the ROS computation graph made of topics and nodes. 

While your rocketship is running, run the following command in a separate Docker container:

```bash
$ rqt
```

You will be presented with a blank GUI in your VNC browser (if not open, click [here](http://localhost:8080/vnc.html) and connect). In the GUI window, click `Plugins->Introspection->Node Graph`. Then change the first dropdown to Nodes/Topics (all) and uncheck the **hide deadsinks** and **hide leaf topics** as shown in the figure below:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-4by1">
        <img src="../images/lab1/rqt1.png">
        </figure>
    </div>
</div>

This gives us a much clearer picture of the software we are running. 
The software modules are the nodes and they are represented as  circles. 
The communication channels are the topics and they are represented as rectangles. 
 

----

# Checkpoint 2

Take a screenshot of the ROS communication graph. 

1. How many software modules (nodes) are there? 
2. How many communication channels (topics) are there?

----

## RQT Plot

Previously we displayed data on the terminal using `rostopic echo`. For identifying data patterns or trends, sometimes if helps to visualize the data through plots.

With the rocket running, start `rqt_plot` in a separate Docker container and check your VNC browser:

```bash
$ rqt
```

To open `rqt_plot` click `Plugins->Visualization->Plot`. You will see output similar to the figure on the left. In the topic edit-box, type in `/cmd_vel` and hit the green + button. You will see a blue line, as shown in the right-hand figure. Use the magnifying glass to set the scale correctly and the arrows to locate the data being plotted.


<p> </p>

<div class="columns is-centered">
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot1.png">
        </figure>
        <p class='has-text-centered' markdown="1">After running the `rqt_plot` command, a GUI similar to this will appear on your screen.</p>
    </div>
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot2.png">
        </figure>
        <p class='has-text-centered' markdown="1">Enter the `cmd_vel` topic and hit the green + button. You can use the **magnifying glass to zoom in and out** (right-click and drag to zoom out).</p>
    </div>
</div>

----
# Checkpoint 3

You now have all you need to launch the rocket and plot the data passed between the different software components. Now plot both the command velocity (`/cmd_vel`) and the velocity sensor data (`/sensor_vel`) using `rqt_plot`. Relaunch the rocket's software so that you can see the velocity from the count down until it reaches maximum velocity. **Take a screenshot of your graph**.

<div class="columns is-centered">
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot3.gif">
        </figure>
        <p class='has-text-centered' markdown="1">The graph is continually updated and should   move as the rocket launches. **This is what you should see.**</p>
    </div>
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/plot4.png">
        </figure>
        <p class='has-text-centered' markdown="1">Take a screenshot, **including the entire GUI.** The graph should look similar to this. **This is what your screenshot should look like.**</p>
    </div>
</div>

Answer the following question using the plot:

1. Why do the sensor readings (red line) not match the command velocity (blue line) exactly? (zoom-in as needed to detect discrepancies)

---- 
# RQT Publisher

The final tool we will inspect in this lab is `rqt_publisher`. `rqt_publisher` provides a GUI plugin for **sending arbitrary ROS messages to a channel** to any node that may be listening to that channel. 
We will use this tool to control the abort function that is already implemented in our rocketship. You would have noticed (in the listed topics or in the rqt graph) that one of the communication channels (topics) the rocketship accepts is called `/launch_abort`. Let's use `rqt_publisher` to abort one of our launches. While your rocketship is running type the following into a separate Docker container:

```bash
$ rqt
```

Click `Plugins->Topics->Message Publisher`. You will see a GUI appear, similar to the one below:

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-4by3">
        <img src="../images/lab1/graph1.png">
        </figure>
    </div>
</div>

We were interested in the communication channel (topic) `/launch_abort`. 
We are going to send (publish) messages on that communication channel (topic) and try to **abort** the rocketship from launching. To do that, do the following:

<div class="columns is-centered">
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/graph2.png">
        </figure>
        <p class='has-text-centered' markdown="1">First: Select `/launch_abort` from the dropdown. Then change the frequency to 5. **Then click the green + button.**</p>
    </div>
    <div class="column is-6">
        <figure class="image is-4by3">
            <img src="../images/lab1/graph3.png">
        </figure>
        <p class='has-text-centered' markdown="1">Second: Open the message using the side arrows. Then select the checkmark. Selecting the checkmark sends messages.</p>
    </div>
</div>

You will notice that **nothing happens.** Let's investigate why. Open a new Docker container and display the content (ROS messages) of the communication channel (topic) using the following code:

```bash
# Start the Docker container
$ docker-compose exec ros bash
$ rostopic echo /launch_abort
>>> data: False
>>> ---
>>> data: False
>>> ...
```

This explains why it did nothing! We were sending `/launch_abort` message set to **false**. So we are **telling the robot not to abort!** To send true abort messages, check the checkbox next to the word false. Once you have done that, you should see the following appear on your rocketship terminal:

```bash
>>> [ INFO] [...]: Abort Failed
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
>>> [ INFO] [...]: Abort Failed
>>> [ INFO] [...]: Requested Velocity: 1684.000000m/s
```

The abort fails because the rocketship is already flying and so you cant abort the launch.

---- 

# Checkpoint 4:

Rerun the abort test before the rocket takes off. **Take a screenshot of the terminal, during an abort**.

1. What does the rocket print to the terminal when you abort before launch? 
2. After the successful abort, try and echo the command velocity (`/cmd_vel`). What message do you receive, and does this make sense?

----

# Final Check:

At the end of this lab, you should have the following:

1. Have installed Docker and the necessary container
    1. Launch the Docker container
    2. Run `roscore` in the terminal
2. A screenshot of the ROS communication graph. Use this graph to answer:
    1. How many software modules (nodes) are there?
    2. How many communication channels (topics) are there?
3. A screenshot of the velocity and sensor readings. Using the knowledge learned in class answer:
    1. Why do the sensor and velocity readings not match?
4. A screenshot of the rocketship software during an abort during the countdown. Using the screenshot to answer:
    1. What is printed to screen when you abort?
    2. What is printed to screen when you try and echo /cmd_vel? Does this make sense?

{% include notification.html
message="Extra: Here is more information on [roscore](http://wiki.ros.org/roscore). Here is more information on understanding [nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes). Here is more information on understanding [topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)."
icon="false"
status="is-success" %}
