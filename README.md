# Autonomous Delivery Robot – ROS + Gazebo Simulation

A fully simulated autonomous delivery robot developed as part of **SENG 691 – AI Agent Computing**.  
This project integrates **sensing, planning, and action** inside a physics‑based environment, transitioning the robot from symbolic behavior (Phase‑1) to realistic autonomous navigation driven by LiDAR and differential‑drive control (Phase‑3).

---

## 🚀 Project Overview

The Autonomous Delivery Robot (ADR) simulates a campus delivery agent capable of:
- Perceiving obstacles using a **Gazebo LiDAR plugin**
- Planning routes using a **grid‑based A\* planner**
- Executing motion using a **differential‑drive controller**
- Adjusting its movement through **sensor‑driven obstacle avoidance**
- Navigating realistically in a **Gazebo physics environment**

Phase‑3 completes the full **sense → plan → act → evaluate** loop.

---

## 🧭 Phase Progress Summary

### **Phase‑1 (Symbolic Prototype)**
- URDF model creation and RViz visualization  
- Placeholder LaserScan + hardcoded obstacles  
- Symbolic navigation (no Gazebo physics)  
- No real sensing, no real motion  

### **Phase‑2 (Simulation Foundation)**
- Robot successfully spawns in Gazebo  
- Hardware dependencies removed  
- A\* planner implemented and validated  
- Architecture restructured into perception–planning–control layers  
- Still **no real motion** and **no real LiDAR perception**

### **Phase‑3 (Full Autonomous Navigation ✓)**
- Differential‑drive controller added → robot **moves physically**  
- Gazebo LiDAR integrated → **real LaserScan** data  
- Obstacle‑aware navigation using sector‑based analysis  
- A\* waypoints now drive real movement  
- Completed perception → planning → control loop  

---

## 🏗 System Architecture (Phase‑3)

```
User/Task → Delivery Agent → A* Planner → Navigation Controller
                   ↓              ↑
 Gazebo LiDAR → Perception Layer → Obstacle Safety Node → cmd_vel → Diff‑Drive Plugin → Gazebo World
```

### Architecture Diagram  
*(Add diagram from your PDF page 10 here)*

### Workflow Diagram  
*(Add sequence diagram from PDF page 11 here)*

---

## 🔍 Key Phase‑3 Enhancements

### ✅ 1. Differential‑Drive Motion Integration
- Robot now moves using real `cmd_vel` commands  
- Wheel rotation, friction, collisions handled by Gazebo  
- Publishes `/odom` for waypoint tracking  
- Enables continuous forward/turn behavior  

### ✅ 2. Real LiDAR Perception
- Gazebo ray‑based LiDAR publishes real `/scan`  
- Environment obstacles reflect accurately  
- Sector‑based (left/front/right) analysis enables reactive behavior  

### ✅ 3. Navigation Pipeline Completion
- A\* global planner still generates waypoints  
- Navigation controller performs:
  - Heading correction  
  - Velocity generation  
  - Obstacle‑based redirection  
- Robot reaches goals meaningfully, not symbolically  

### ✅ 4. Updated Control Architecture
- Odometry → Pose2D conversion for planner  
- Safety node overrides unsafe velocities  
- Modular and maintainable code structure  

---

## 📂 Repository Structure

```
autonomous-delivery-robot/
├── src/
│   ├── main.cpp              # Navigation + differential-drive motion
│   ├── delivery.cpp          # Task handling, setpoints, agent logic
│   ├── obs_main.cpp          # LiDAR-based obstacle avoidance
│   ├── gps.py                # Simulated GPS / pose feed
│   ├── access_database.py    # (Optional) external DB integration
│
├── launch/
│   ├── gazebo_nav.launch     # Full Phase-3 launch
│   ├── gazebo_delivery.launch
│   ├── delivery.launch
│
├── worlds/                   # Custom Gazebo world
├── urdf/                     # Robot model w/ diff-drive + LiDAR
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🧪 Running the Project

### **1. Build**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **2. Run Full Phase‑3 System**
```bash
roslaunch autonomous-delivery-robot gazebo_nav.launch
```

This loads:
- Gazebo world  
- URDF robot  
- LiDAR plugin  
- Differential‑drive controller  
- A\* planner  
- Navigation controller  
- RViz visualization  

### What You Should See
- Robot spawns in Gazebo  
- `/scan` shows real LiDAR rays  
- A\* path appears in RViz  
- Robot moves toward goal  
- Robot slows/turns when obstacles appear  
- Robot completes navigation task  

---

## 📡 Important ROS Topics

| Topic | Description |
|-------|-------------|
| `/scan` | Real LaserScan from Gazebo LiDAR |
| `/odom` | Odometry from differential-drive |
| `/cmd_vel` | Final velocity commands |
| `/cmd_vel_nav` | Raw navigation controller output |
| `/cmd_vel_safe` | Obstacle-filtered safe commands |
| `/next_goal` | A\* waypoint publisher |

---

## 📈 Evidence of Improvements (Phase‑2 → Phase‑3)

### Motion
- **Phase‑2:** Robot spawned but could not move  
- **Phase‑3:** Robot moves realistically using diff‑drive plugin  

### Perception
- **Phase‑2:** Fake/symbolic LaserScan  
- **Phase‑3:** Real LiDAR perception from environment  

### Navigation
- **Phase‑2:** Planner disconnected from motion  
- **Phase‑3:** Robot physically follows A\* path  

### Obstacle Handling
- **Phase‑2:** None  
- **Phase‑3:** Sector‑based detection & avoidance  

### Integration
- **Phase‑2:** Modules symbolic, disconnected  
- **Phase‑3:** Full integrated sense → plan → act loop  

---

## ⚠️ Known Limitations

Even with Phase‑3 improvements, some constraints remain:

- No SLAM / AMCL → odometry drift possible  
- A\* global planner does not dynamically replan  
- Obstacle avoidance is reactive, not predictive  
- No multi‑robot coordination  
- System is simulation‑only (no real hardware yet)  

These limitations do not affect Phase‑3 correctness.

---

## 🔮 Future Scope

- Add SLAM/AMCL for stable localization  
- Introduce dynamic global re‑planning (costmaps)  
- Expand to multi‑robot coordination  
- Integrate machine‑learning-based motion control  
- Deploy on real hardware with LiDAR  

---

## 🎥 Phase‑3 Demonstration Summary

Your demo video should showcase:

1. Gazebo world + robot spawning  
2. RViz showing LiDAR rays, A\* path, TF frames  
3. Robot moving using diff‑drive  
4. Obstacle appearing and being avoided  
5. Robot reaching destination successfully  

This validates the complete perception → planning → control system.

---

## 📎 Resources

- GitHub Repo: https://github.com/Mounika2526/autonomous-delivery-robot  
- Demo Video Folder: (Add your Drive link)

---

## 📄 License
MIT License

---

This README reflects your **actual Phase‑3 academic achievements** and is suitable for GitHub, portfolios, and resume showcasing.
