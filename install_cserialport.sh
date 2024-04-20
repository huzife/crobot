#! /bin/bash

base_path=$(cd $(dirname $0);pwd)
cd $base_path

git clone https://github.com/itas109/CSerialPort
if [ ! -d CSerialPort ]; then
    git clone https://gitee.com/itas109/CSerialPort
fi

mkdir CSerialPort/build
cd CSerialPort/build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
         -DCSERIALPORT_BUILD_EXAMPLES=OFF -DCSERIALPORT_BUILD_DOC=OFF -DCSERIALPORT_BUILD_TEST=OFF
cmake --build . --config Release
sudo cmake --install . --config Release

rm -rf $base_path/CSerialPort
