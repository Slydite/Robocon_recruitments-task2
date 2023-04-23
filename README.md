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

- The `pid.py` script has some boilerplate code for implementing a PID controller. Currently, it asks you to enter an input velocity every 2 seconds. **You need to modify this file so that it asks for coordinates instead and the bot must reach those coordinates using PID control.** 

- To control the bot's movement, you can use the `odom` topic to get its current status and the `cmd_vel` topic to give it new velocities. Just be careful not to give it large velocities or it might behave unpredictably.

- Try to understand the code and how exactly the various topics are being published and subscribed. 

- We have abstracted away much of the ROS node interactions and only left you with the logic implementation. Read the documentation provided carefully as it will solve almost 80% of any doubts you might have.

- A soft deadline for this subtask is 27th Apr while the next subtask will be done later keeping your midsems in mind, although we wouldn't mind if you submit the entire task by the deadline of subtask B, but it would be better if you split it into two parts the way we have done. 

- As always, feel free to ask any doubts you may have about the task. Good luck!

## Subtask B: Traversing a map guided by aruco tags

In this subtask, you'll learn how to detect Aruco tags, calculate their distance from the bot, and navigate the map using guidance from aruco tags. It's an opportunity to explore new techniques and expand your knowledge in robotics. The details of this subtask will be updated later. It would be very easy to do once you are done with subtask A


