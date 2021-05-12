#!/bin/bash

echo "Building shark..."

mkdir -p shark
cd shark
`svn co https://svn.code.sf.net/p/shark-project/code/trunk/Shark`
cd Shark
cmake -DOPT_ENABLE_ATLAS=ON -DOPT_ENABLE_OPENMP=ON
make
sudo make install

echo "Building shark... DONE!"