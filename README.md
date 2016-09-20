# dsa-seahorn
DSA fork for SeaHorn. 
Based on the DSA for in [SMACK](https://github.com/smackers/smack).
Originally from [PoolAlloc](https://llvm.org/svn/llvm-project/poolalloc/).

# Standalone installation #

    mkdir build && cd build
    let LLVM_OBJ_ROOT be the output of llvm-config --obj-root
    cmake -DCMAKE_BUILD_TYPE=Release -DLLVM_DIR=${LLVM_OBJ_ROOT}/share/llvm/cmake/ -DCMAKE_INSTALL_PREFIX=run ../
	cmake --build . --target install

The main output of the installation is two libraries located at
`CMAKE_INSTALL_PREFIX/run/lib`: `AssistDS` and `DSA`.

# Usage #

    opt -load ${CMAKE_INSTALL_PREFIX}/run/lib/libDSA.so -load ${CMAKE_INSTALL_PREFIX}/run/lib/libAssistDS.so  -dsa-steens  test.bc -disable-output -print-alias-sets
