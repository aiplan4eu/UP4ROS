Usage
=====

.. _installation:

Install through apt
-------------------

Here we assume that the unified-planning package is already installed on your machine. Else, see below.
Installation commands:

.. code-block:: console

   echo "deb [trusted=yes] https://raw.githubusercontent.com/aiplan4eu/UP4ROS/focal-noetic/ ./" | sudo tee /etc/apt/sources.list.d/aiplan4eu_UP4ROS.list
   echo "yaml https://raw.githubusercontent.com/aiplan4eu/UP4ROS/focal-noetic/local.yaml noetic" | sudo tee /etc/ros/rosdep/sources.list.d/1-aiplan4eu_UP4ROS.list
   sudo apt update
   sudo apt install ros-noetic-up4ros

Install from source
-------------------

This package is a `ROS <https://www.ros.org/>`_ package. It has been developed and tested against `ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_.
To use up4ros, first install its dependencies (we suggest in a `virtual environment <https://docs.python.org/3/library/venv.html>`_):

.. code-block:: console

   (.venv) $ pip install unified-planning 
   (.venv) $ pip instal unified-planning[tamer]
   (.venv) $ pip instal unified-planning[pyperplan]

At this point, you are ready to install the package in your catkin workspace.
Assuming that the path of such a worspace is ~/catkin_ws and your ROS distro is noetic:

.. code-block:: console

   export CATKIN_WORKSPACE=~/catkin_ws
   export ROS_DISTRO=noetic
   rosdep update
   rosdep install --from-paths $CATKIN_WORKSPACE/src/ --ignore-src --rosdistro=$ROS_DISTRO -y
   cd $CATKIN_WS
   bash -c "source /opt/ros/noetic/setup.bash && catkin build up_msgs up4ros --no-status --no-notify"
   cd $CATKIN_WORKSPACE/src
   bash -c "source ${{steps.prepare-catkin-ws.outputs.catkin_ws}}/install/setup.bash && python3 -m pytest"

Launching and solving our first planning problem
------------------------------------------------

To launch our freshly installed up4ros node, run the following command:

.. code-block:: console

   roslaunch up4ros up4ros.launch

At this point, the action servers and services are up and can be queried.


Example query
-------------

Two example clients are provided in the package. 
To see how this wrapper can be used, while the up4ros node is running make sure to run:

.. code-block:: console

   rosrun up4ros example_client.py

If everything goes well, in the node terminal you should see:

.. code-block:: console

   Tamer returned: [move(l1, l2)]

which can be also found in the result received by the client.