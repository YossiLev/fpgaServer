rm -rf build/
cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo -B build -S . 
cmake --build build 

