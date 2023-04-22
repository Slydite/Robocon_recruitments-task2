# Robocon Recruitments- Task 3.2

This task is divided into two subtasks: "Mastering PID Control" and "Traversing a map guided by aruco tags".

## Subtask A: Mastering PID Control

In this subtask, you'll learn how to use a PID controller to take control of the Atom bot and guide it to any coordinate you choose. 

- A PID controller is a powerful tool that uses proportional, integral, and derivative terms to control a system. By mastering this tool, you'll be able to control the robot with precision and accuracy.
    - You can learn more about PID controllers from these resources: https://youtu.be/wkfEZmsQqiA and any other content you find for pid controllers. 

- To get started:
    - You'll need to use the Atom bot package again. (You may use another bot, but this guide shall follow the atom bot).
    - In this repository, there's a file called `pid.py`. This file needs to be placed in `Robotics_ws/src/atom/script`.
    - Once the file is in place, you need to run `chmod +x` on it to make it executable.
    - Then, in the original folder (`Robotics_ws`), you need to run `catkin_make` and `source devel/setup.bash`.
    - Finally, you can run `rosrun atom pid.py` for the script to start working. Make sure you first launch the world using the command from Task 1 (`roslaunch atom world.launch`).

- The `pid.py` script has some boilerplate code for implementing a PID controller. It asks for you to enter an input velocity every 2 seconds. We have already done the heavy lifting of writing code to publish and subscribe as well as setting up the variables needed for controller, so now you just need to tune your PID controller and write your code properly. **You need to modify this file so that it asks for coordinates instead and the bot must reach those coordinates using PID control.**

- To control the bot's movement, you can use the `odom` topic to get its current status and the `cmd_vel` topic to give it new velocities. Just be careful not to give it large velocities or it might behave unpredictably.

- Try to understand the code and how exactly the various topics are being published and subscribed from so you can write your own code yourself.

- The task is not difficult at all since we have abstracted away much of the implementation part and only left you with the logic implementation. You can finish this task quite easily if you are attentive and **READ** the documentation we have provided.

- The deadline for subtask 1 would be 27th May while subtask 2 will be done later keeping your midsems in mind. 


## Subtask B: Exploring Aruco Tag Detection

In this subtask, you'll learn how to detect Aruco tags, calculate their distance from the bot, and navigate the map using guidance from aruco tags. It's an opportunity to explore new techniques and expand your knowledge in robotics. The details of this subtask will be updated later.

## As always, feel free to ask any doubts you may have about the task.
