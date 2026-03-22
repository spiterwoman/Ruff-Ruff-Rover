Chrystel Geno, Diego Quinones, Mohammed Mamdouh, Victor Acuna   
Dr. Rakhshan  
EGN4060C  
9 February 2026  
**Project Proposal**  
**Pitch:**   
Rover that acts based on voice based commands from its operator, with the main sell being the robot ability to locate and move towards a person in their vicinity who whistles for the rover.

**MVP/Stretch Goals:** 

- *MVP:* Robot waits for a whistle, locates where the whistle noise came from, looks for a person in the whistle area, then moves to the whistler.  
- *Stretch Goals:*   
* Robot can take in voice commands to move to an object within its vicinity “Go to the apple”.  
* Robot can look at the operator pointing somewhere, and go to that point.  
* Robot can fetch items using an externally attached simple clasping manipulator.

**Technical Breakdown:**   
At its simplest, this robot will consist of a simple 2/4 wheel rover kit attached with a microphone and camera setup. The microphone allows the robot to pick up, classify, and localize an audio stream to allow for reactive work with real time audio. The camera setup will feed into a backend processing system to allow for object classification and rudimentary path planning needs. Depending on the complexity of stretch goals implemented, more technology may be implemented to better fit our usage, not limited to: 2D LiDAR/Ultrasonics for more sophisticated costmap generation for local path planning and object localization, more cameras for triangulation for pointing command gestures, and an external clasping manipulator to “fetch” items.

**Software/Hardware Makeup:**

- *Software*: [ROS2](https://github.com/ros2) (path planning, sound localization, general driver logic), [YOLOv11](https://docs.ultralytics.com/models/yolo26/) (person/object classification), [Whisper](https://github.com/openai/whisper) (audio transcriber if voice command stretch goal is attempted).  
- *Hardware*: Generic 2/4 wheel rover kit, camera(s), microphone. Optionally: 2D LiDAR, Ultrasonic, Manipulator.

**Project Inspirations/Links:**

- [Deep-learning-focused robotics SSL Review](https://www.mdpi.com/2076-3417/15/17/9354?utm_source=) (Sound-source localization)  
- [Open embedded Audition System](https://github.com/introlab/odas?tab=readme-ov-file) and [corresponding paper](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.854444/full) (Open-source ‘robot hearing’ stack that uses mic array audio to provide usable outputs)  
- [Whistle detection research](https://www.diva-portal.org/smash/get/diva2%3A836227/FULLTEXT01.pdf), [ROS2 Nav2 Concepts](https://docs.nav2.org/concepts/index.html) (Navigation)