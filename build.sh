
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "compile done"

./main ../vitamine.yaml
