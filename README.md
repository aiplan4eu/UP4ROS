```bash
echo "deb [trusted=yes] https://raw.githubusercontent.com/aiplan4eu/UP4ROS/focal-noetic/ ./" | sudo tee /etc/apt/sources.list.d/aiplan4eu_UP4ROS.list
echo "yaml https://raw.githubusercontent.com/aiplan4eu/UP4ROS/focal-noetic/local.yaml noetic" | sudo tee /etc/ros/rosdep/sources.list.d/1-aiplan4eu_UP4ROS.list
```
