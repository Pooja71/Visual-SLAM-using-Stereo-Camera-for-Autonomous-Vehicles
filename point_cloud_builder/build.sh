rm -r build
mkdir build
cd build
cmake .. -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF
make -j4
cp build/devel/lib/point_cloud_builder/generate_pc ..
