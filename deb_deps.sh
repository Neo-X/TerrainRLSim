#! /bin/bash
#
# Install the system dependencies required to build TerrainRL
#
### For rendering the simulation using OpenGL
apt-get -y install freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libglew1.6-dev mesa-utils 
### dependancies for caffe....
apt-get install -y libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
### Not sure what needs this
apt-get install -y --no-install-recommends libboost-all-dev
### more dependancies of caffe
apt-get install -y libgflags-dev libgoogle-glog-dev liblmdb-dev
### more dependancies of caffe
apt-get install -y libatlas-base-dev
### Make sure you have a good compiler
# ./setupgcc6.sh

apt-get install -y libf2c2-dev 
### need for shader rendering
apt-get install -y libglew-dev
### Needed for Python wrappers 
apt-get install -y swig3.0 premake4
### For openGL ES support
apt-get install -y libglfw3-dev libgles2-mesa-dev
