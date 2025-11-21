```
>> apt-get update && apt install sudo build-essential libboost-all-dev ccache libopencv-dev

# install upgraded cmake>24.0 manually
>> cd /opt/ && wget https://github.com/Kitware/CMake/releases/download/v3.31.4/cmake-3.31.4-linux-x86_64.sh
>> sudo bash ./cmake-3.31.4-linux-x86_64.sh
>> sudo ln -s /opt/cmake-3.31.4-linux-x86_64/bin/* /usr/bin
>> cmake --version


>> git clone https://github.com/AbhiKhoyani/slambook.git
>> cd slambook
>> git submodule init
>> git submodule update
>> cd 3rdparty

# install Sophus and dependencies
>> ./Sophus/scripts/install_ubuntu_deps_incl_ceres.sh   # Not recommended
>> cd Sophus && mkdir build && cd build && cmake .. && make install

# install Pangolin and dependencies
>> cd 3rdparty/Pangolin
>> ./scripts/install_prerequisites.sh recommended
>> mkdir build && cd build
>> cmake .. && make install

# install g2o and dependencies  # ch6
>> sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake
>> cd 3rdparty/g2o
>> mkdir build && cd build
>> cmake .. & make install

# install ceres-solver
>> sudo apt-get install libgtest-dev
>> cd 3rdparty/ceres-solver
>> git submodule init && git submodule update
>> cd 3rdparty/ceres-solver/third_party/abseil-cpp && cmake -B build && cd build && make -j8 install
>> cd 3rdparty/ceres-solver/third_party/googletest && cmake -B build && cd build && make -j8 install
>> cd 3rdparty/ceres-solver
>> mkdir build && cd build
>> cmake .. & make install 

# install BoW3  # ch11
>> cd 3rdparty/DBow3
>> mkdir build && cd build
>> cmake .. && make install # add #include <string> in DBoW.h if you get error of istream/ostream

```