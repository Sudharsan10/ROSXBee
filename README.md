<!-- Update Meta -->
<meta name="google-site-verification" content="anaQPdlDO5QzFwQQ6mpEFvfJXLj2Ue8-EFylgHd7JlU" />

[comment]: <> ([![social-header]&#40;.\img\project_social_card01.png&#41;]&#40;https://github.com/Sudharsan10/ROSXBee&#41;)


<p align="center">
  ROSXBee  
  <br>
    <a href=""><strong>Explore ROSXBee docs »</strong></a>
    <br>
    <br>
    <a href="https://github.com/Sudharsan10/ROSXBee/issues/new">Report bug</a>
    ·
    <a href="https://github.com/Sudharsan10/ROSXBee/issues/new">Request feature</a>
</p>

ROSXBee is a ROS2 package to interface and expose XBee Device using ROS2 packaging as a wrapper around the Digi-XBee python API library.

---

# *Table of contents*

- [*Table of contents*](#table-of-contents)
- [*Work in progress*](#work-in-progress)
- [*Status*](#status)
- [*What's included*](#whats-included)
- [*Pre-requisites*](#pre-requisites)
  - [*1. ROS2 Foxy Fitzroy Installation*](#1-ros2-foxy-fitzroy-installation)
  - [*2. Pip Installation*](#2-pip-installation)
  - [*3. Digi-Xbee library Installation*](#3-digi-xbee-library-installation)
  - [*4. Miscellaneous Py3 dependencies*](#4-miscellaneous-py3-dependencies)
- [*Documentation*](#documentation)
- [*Bugs and feature requests*](#bugs-and-feature-requests)
- [*Creators*](#creators)

# *Work in progress*
You can find latest work done commits in the development branch and completed features will be pulled into features branch.
- [X] Create Basic XBee Interface in python
- [X] Feat: Add transmit func.
- [X] Use parameters to set ports and Baud rate
- [ ] Feat: Add Receive func.
- [X] Add Readme.md to main branch
- [ ] Update Readme and Wiki

---

# *Status*

[![ROS-version](https://img.shields.io/badge/ROS-2-darkorange)](https://docs.ros.org/en/foxy/index.html)
[![ROS-Distro](https://img.shields.io/badge/Distro-Foxy%20Fitzroy-darkorange)](https://docs.ros.org/en/foxy/Releases.html)
[![made-with-python](https://img.shields.io/badge/Python-3.8.5-brightgreen)](https://www.python.org/)
[![pip-version](https://img.shields.io/badge/Pip-21.0.1-brightgreen)](https://pip.pypa.io/en/stable/installing/)
[![Digi-Xbee-version](https://img.shields.io/badge/Digi%20Xbee-1.3.0-darkgreen)](https://xbplib.readthedocs.io/en/latest/#)
[![Cbor2-version](https://img.shields.io/badge/Cbor2-5.2.0-darkgreen)](https://cbor2.readthedocs.io/en/latest/index.html)
[![Flow-charts](https://img.shields.io/badge/Flowcharts-MS%20Power%20Point-20639B.svg)](https://www.microsoft.com/en-us/microsoft-365/powerpoint?SilentAuth=1&wa=wsignin1.0)
[![contributors](https://img.shields.io/badge/Contributors-01-0366d6)](https://github.com/Sudharsan10/ROSXBee/graphs/contributors)
[![Documentation Status](https://img.shields.io/badge/Documentation-yes-e01563)](https://github.com/Sudharsan10/ROSXBee/tree/master/README.md)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-e01563.svg)](https://github.com/Sudharsan10/ROSXBee/graphs/commit-activity)

# *What's included*

Within the download you'll find the following directories and files, logically grouping the modules in its own packages.You'll see something like this:

```text
ROSXBee/
├── code/
|   └── src
|       ├── rosxbeecpp/...  # ROS2 c++ package
|       └── rosxbeepy/..    # ROS2 Python package
├── .gitignore
└── setup.py
```

# *Pre-requisites*

**ROSXBee** is a ROS2 package hence it requires `ROS2 Foxy Fitzroy` as main pre-requisite and it also has few python3 dependencies, and you can get them set up by below instructions,

## *1. ROS2 Foxy Fitzroy Installation*

Go to this link [ROS Foxy Fitzroy installation](https://docs.ros.org/en/foxy/Installation.html)
and follow the official guide to install ROS in ubuntu or follow the below steps for linux users.

- *Set local*

  ```bash
  locale  # check for UTF-8

  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  locale  # verify settings
  ```

- *Setup your source.list*

  You will need to add the ROS 2 apt repositories to your system. To do so, first authorize our GPG key with apt like this:

  ```bash
  sudo apt update && sudo apt install curl gnupg2 lsb-release
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  ```

  And then add the repository to your sources list:

  ```bash
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
  ```

- *Install ROS 2 Foxy Fitzroy Desktop*
  - First, make sure your Debian package index is up-to-date,
  
    ```bash
    sudo apt update
    ```
  
  - Install Ros2 Desktop full:
  
    ```bash
    sudo apt install ros-foxy-desktop
    ```

- *Environment Setup*
  - To use ROS2 in your terminal source the ROS2 installation,

    ```bash
    source /opt/ros/foxy/setup.bash
    ```
  
  - Install Argcomplete:
  ROS 2 command line tools use argcomplete to autocompletion. So if you want autocompletion, installing argcomplete is necessary.

    ```bash
    sudo apt install -y python3-pip
    pip3 install -U argcomplete
    ```

- *Install Colcon*

  ```bash
  sudo apt-get install python3-colcon-common-extensions
  ```

## *2. Pip Installation*

```bash
sudo apt install python3-pip
```

## *3. Digi-Xbee library Installation*

```bash
pip3 install digi-xbee
```

## *4. Miscellaneous Py3 dependencies*

```bash
pip3 install cbor2 pyyaml 
```

[comment]: <> (# *Documentation*)
  
[comment]: <> (1. [How to use]&#40;#howtouse&#41;)

[comment]: <> (2. [Architecture]&#40;#architecture&#41;)

[comment]: <> (3. [Solver.py]&#40;#solver_py&#41;)

[comment]: <> (4. [Node obj Data structure]&#40;#node&#41;)

# *Bugs and feature requests*

Have a bug or a feature request? Search for existing and closed issues, if your problem or idea is not addressed yet, [please open a new issue](https://github.com/Sudharsan10/ROSXBee/issues/new).

# *Creators*

**@Sudharsan** : [www.iamsudharsan.com](https://www.iamsudharsan.com)

*Thank you for visiting my Repo, have a great day!*
