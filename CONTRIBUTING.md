# Contributing to robotiq_2f85_ros2

Thank you for your interest in contributing!  
This project brings the Robotiq 2F-85 gripper into the ROS 2 Jazzy + Gazebo Harmonic ecosystem, and contributions are welcome.

---

## 🔧 How to Contribute

### 1. Fork the Repository
Click **Fork** at the top right of the GitHub page.

### 2. Clone Your Fork
```bash
git clone https://github.com/<your-username>/robotiq_2f85_ros2.git
cd robotiq_2f85_ros2
```

### 3. Create a New Branch
```bash
git checkout -b feature/my-improvement
```

---

## 🧹 Code Standards

### ROS 2 Best Practices
- Follow ROS 2 style conventions  
- Use `ament_lint_auto` and `ament_cmake`  
- Keep `launch`, `config`, `urdf`, and `worlds` well‑organized  
- Keep parameters and controllers modular  

### Python
- Use snake_case  
- Avoid unused imports  
- Keep launch files readable and deterministic  

### URDF/Xacro
- Avoid hardcoding absolute paths  
- Maintain ROS 2 + Gazebo Harmonic compatibility  

---

## 🧪 Testing

### Build the workspace:
```bash
colcon build --symlink-install
```

### Source it:
```bash
source install/setup.bash
```

### Launch simulation:
```bash
ros2 launch robotiq_gripper_gazebo launch_gripper_gazebo.py
```

### Test controllers:
Open:
```bash
ros2 topic pub --once /robotiq_gripper_controller/joint_trajectory ...
```

Ensure:
- No controller warnings  
- No missing joints  
- Smooth trajectory execution  

---

## 📝 Submitting a Pull Request

1. Ensure your branch is up to date:
```bash
git fetch origin
git rebase origin/main
```

2. Push your branch:
```bash
git push origin feature/my-improvement
```

3. Open a Pull Request:
   - Clear description of the change  
   - Include screenshots/logs if needed  
   - Mention related issues  

---

## 🐛 Reporting Issues

Open an issue with:
- Full description  
- Steps to reproduce  
- ROS 2 + Gazebo versions  
- Logs if controller or physics errors occur  

---

## 📦 Release Workflow

Contributors should **not** add tags directly.  
Open a PR and maintainers will:

- Update CHANGELOG  
- Update version in `package.xml`  
- Create GitHub release  

---

## 💬 Questions?

Open an issue or start a discussion on GitHub.

We appreciate every contribution made to improve this simulation package!
