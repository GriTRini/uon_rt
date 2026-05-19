mkdir -p ../build ../bin

cmake -S ../example/cpp -B ../build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_RUNTIME_OUTPUT_DIRECTORY="$(pwd)/../bin"

cmake --build ../build -j

