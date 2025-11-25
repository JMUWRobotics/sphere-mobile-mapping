# Ground Finder

This ROS package is to find the normal vector of theground from the current laserscan of the DAEDALUS spherical robot.

This package has been created as part of Caro's master thesis.


## Installation
Make sure to have the following installed:
```bash
sudo apt-get install ros-noetic-pcl-conversions ros-noetic-pcl-ros libpcl-dev
```

## Launch ground finder
Using roslaunch file: `main.launch`. Choose the wanted configurations with the parameters as follows.
```mermaid
flowchart TD
    A(Parameters)
    A --> B(filter)
    B -->|:=geo| C(Geometrical)
    B -->|:=kdt| D(KD-Tree)
    B -->|:=none| E(None)
    A --> F(subcloud)
    F -->|:=geo| G(Geometrical)
    F -->|:=kdt| H(KD-Tree)
    A --> I(plane)
    I -->|:=lsf| J(LSF)
    I -->|:=pca| K(PCA)
    I -->|:=ran| L(RANSAC)
    I -->|:=rht| M(RHT)
    I -->|:=rht2| R(RHT2)
    A --> O(quiet)
    O -->|:=true| P(No Output)
    O -->|:=false| Q(Regular Output)
```

## Test scripts
List of test scripts:
- Reading in point cloud from topic + test processing           (`src/test_files/test.cpp`)
- ikd-tree from hku mars (build + knn ranged search - runtime)  (`src/test_files/ikd_tree_test.cpp`)
- PCL kd-tree (build + knn search - runtime)                    (`src/test_files/pcl_kd_tree_test.cpp`)
- Ground plane detection (lsf/pca/ransac/rht) with n calculation(`src/test_files/plane_test.cpp`)

**HOW TO RUN TEST SCRIPTS:**
Using roslaunch file: `test.launch`. Choose the wanted test file with the parameters as follows.

```mermaid
flowchart TD
    A(test.launch)
    A -->|default| B(test.cpp)
    A -->|test:=kdt| C(KD-Tree)
    C -->|type:=ikd| D(ikd_tree_test.cpp)
    C -->|type:=pcl| E(pcl_kd_tree_test.cpp)
    A -->|test:=plane| F(plane_test.cpp)
    F -->|type:=lsf| G(LSF)
    F -->|type:=pca| H(PCA)
    F -->|type:=ran| I(RANSAC)
    F -->|type:=rht| J(RHT)
    A -->|test:=cb| K(cropbox_test.cpp)
```

Note: [Online mermaid editor](https://mermaid.live/edit#pako:eNpdj0FrwzAMhf-K0CmFZj_A0MGWrJddBsttHkXYyhzqOMa1GaXuf59KV-imi8R733ugE5rFMioc_fJtHKUMQw-gA8g8NZkP-cFTCcatbiK07WO1PFLxucLzlTExru79i6g2eytE17z27ZCYbw3dlThGVptpbyv0jaxdFmT3r-wejcZXeGmmN7cE_uMPTqIVth8jqZFaQwk6Sp-4xpnTTJOV_06XgMbseGaNSs7fFzTqcBaUSl7ej8GgyqnwGku0lLmf6CvRjNLrD3z-AfPCYKI)

## Authors
Carolin Bösch

# TODOs
[ ] Create subcloud mit cropbox filter pcl
[ ] sliding window
