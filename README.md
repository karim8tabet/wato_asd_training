# WATonomous ASD Admissions Assignment

# ASD Admission Assignment – My Solution

## What I built
- **Costmap:** turns each lidar scan into a 40 m × 40 m grid (0.1 m cells) around the robot, marks obstacles, and inflates them with a linear cost falloff over 1.5 m.
- **Map Memory:** keeps a 60 m × 60 m global map in `sim_world`. Every 1.5 m of travel, it transforms the latest costmap into the world frame using odometry and merges it, keeping the highest cost per cell so remembered obstacles aren't erased.
- **Planner:** A* on the global map with 8-connected movement. Cells with cost 50 or more are treated as walls, and lower costs add a penalty so paths stay away from obstacles. Replans every 0.5 s and stops when the goal is within 0.5 m or after a 120 s timeout.
- **Control:** pure pursuit with a 0.8 m lookahead at 0.5 m/s. Turns in place when the target is more than ~45° off-heading and stops within 0.4 m of the goal.

## Tuning
The robot initially clipped obstacles because the planner treats it as a point. Increasing the inflation radius (1.0 -> 1.5 m), lowering the obstacle threshold (80 -> 50), and shortening the lookahead (1.0 -> 0.8 m) fixed it.


## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/
