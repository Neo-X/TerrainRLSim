cd external/caffe
make clean
make
cd ../../
cp -r external/caffe/build/lib .
cp external/caffe/build/lib/libcaffe.* lib/
cp external/Bullet/bin/*.so lib/
cp external/jsoncpp/build/debug/src/lib_json/*.so* lib/
cd simAdapter/
./gen_swig.sh
cd ../
./premake4_linux clean
./premake4_linux gmake
cd gmake
make config=release64
cd ../