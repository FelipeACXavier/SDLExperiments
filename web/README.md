# Port of the wave generator to webassembly

To compile, don't forget to clean the build directory and run

```
emcmake cmake .. -DCMAKE_TOOLCHAIN_FILE=<path to emsdk>/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
```