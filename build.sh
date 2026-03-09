#!/bin/sh
catkin_make VERBOSE=1 -j$(nproc) \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native -mtune=native -flto -fno-math-errno -DEIGEN_MAX_ALIGN_BYTES=64 -DEIGEN_UNROLLING_LIMIT=256" \
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -mtune=native -flto -fno-math-errno -DEIGEN_MAX_ALIGN_BYTES=64 -DEIGEN_UNROLLING_LIMIT=256" \
  -DCMAKE_EXE_LINKER_FLAGS="-flto" \
  -DCMAKE_SHARED_LINKER_FLAGS="-flto"