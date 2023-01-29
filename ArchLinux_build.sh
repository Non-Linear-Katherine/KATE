mkdir -p build
cd build
cmake -S ../ -B .
make && make Shaders &&./Gravity_System_Test
cd ../