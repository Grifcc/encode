mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE="Debug" -DCMAKE_TOOLCHAIN_FILE=../toolchains/orin_native.toolchain.cmake  -DCMAKE_INSTALL_PREFIX=../install 
make install -j$(nproc)
if [ $? != 0 ]; then
    echo "Error occurs in complaining."
    exit 1
fi
