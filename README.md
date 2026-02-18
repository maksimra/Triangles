# Triangles

Project of simulation 3D triangles intersection.

## Dependencies

Compiler c/c++, cmake, gtest

```shell
apt-get install build-essential clang make cmake
apt-get install libgtest-dev libgmock-dev libtbb-dev
```

## Building

```shell
cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build
```

Binaries are located in `build/source/` and `build/tests`

## Usage

Format of input (stdin): amount of triangles, next for each triangle pass 3 point, each point - 3 double

```
8
1 1 0 3 1 0 1 3 0
0 0 0 1 0 0 0 1 0
1 0.5 0 1 0.5 1 0 0 0.5
1 0 0 0 1 0 0 0 1
0 0 0 0 3 3 0 0 3
1 1 0 1 2 3 5 4 8
9 9 9 9 9 9 9 9 9
8 8 8 8 8 8 -10 8 8
```

Format of output: id's of triangles that has intersection (each on new line)

```
0
1
2
3
4
5
```

## Tests

For run end to end tests do:

```shell
./build/tests/e2eTests
```

or unit tests:

```
./build/tests/unitTests
```