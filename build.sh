#cd thirdParty/g2o

#echo "Configuring and building Thirdparty/g2o ..."

#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Debug
#make -j

#cd ../../../


#rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "compile done"

./main ../../../1SLAM/SLAM_dataset/TUM/freg3.yaml ../../../1SLAM/SLAM_dataset/TUM/rgbd_dataset_freiburg3_long_office_household

#../vitamine.yaml #rgbd_dataset_freiburg3_long_office_household #rgbd_dataset_freiburg2_desk