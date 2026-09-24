# RoboCup.dk 2026 – Webots Simulation

This repository contains a Webots simulation of the **RoboCup.dk 2026** competition track.

![RoboCup.dk 2026 Simulation](worlds/.robocup.dk_2026.jpg)

## Running the Simulation

1. Download and install Webots from https://cyberbotics.com/.  
   The simulation was developed using **Webots R2025a**, but newer versions should also work.
2. Build `WildWilly.Webots.Controller` from the WildWillyCore solution. Its post-build step copies the controller executable and dependencies to `controllers/wild_willy_r5/`.
3. Open `worlds/robocup.dk_2026.wbt` in Webots and press **Play**. The controller requires the .NET 10 runtime.

The world uses the `WildWillyR5` robot and its `wild_willy_r5` C# controller. The PROTO's robot frame is X forward, Y left, Z up; its original Y-forward CAD and devices are rotated inside the PROTO, without modifying the DAE files. The old `controllers/boebot_controller/` example remains in the repository but is not selected by the world.

## Track Rules and Setup

During the competition, several obstacles on the track will be manually randomized.  
These include elements such as the **seesaw angle**, **free gates**, and **ball placement**.

In the simulator, this randomization occurs automatically when the robot passes the **start line**. At the same time, the **timed door**, **fitness obstacle**, and **locked door obstacle** are reset.

You can also manually trigger this randomization process by pressing the **`R`** key.

This allows you to reset and randomize the track without needing to drive the robot back to the start line.

The track behavior is controlled by the `controllers/controlled_obstacles/controlled_obstacles.py` script. You can experiment with or disable parts of this script to make it easier to test your own robot algorithms.
