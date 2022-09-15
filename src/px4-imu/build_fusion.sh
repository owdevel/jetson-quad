echo "Configuring and building Fusion ..."

cd Fusion
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j3
