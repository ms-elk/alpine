# alpine
**apline** is a CPU path tracer, and is developed to experiment several rendering algorithms.

## Requirements
- Visual Studio 2022
- C++ 17
- CMake 3.26
- OpenGL 4.6 (optional for [viewer](#viewer))

## Build
1. Download the libraries below, and put them into the `ext` directory
    - [Intel Embree 4.3.3](https://www.embree.org/)
    - [Intel Open Image Denoise 2.0.1](https://www.openimagedenoise.org/)
    - [GLFW 3.3.8](https://www.glfw.org/) (optional for [viewer](#viewer))
2. Fetch git submodules
    ```
    git submodule update --init
    ```
3. Run CMake in the root directory (optional: add `-DALPINE_BUILD_APPS=ON` command to include the [sample applications](#sample-applications))
    ```
    cmake -S . -B build
    ```
4. Open the solution file `alpine.sln` generated in the `build` directory, and build it in Visual Studio

## Sample Applications
### simple
*simple* is an application which generates an image using path tracing. It takes an obj file as input, and outputs a ppm file.
```
simple spp input.obj output.ppm
```
`spp` specifies the number of samples per pixel.

### viewer
*viewer* is a progressive path tracer, and takes a glb file as input.
```
viewer input.glb
```
The camera can be manipulated by a mouse.  
`Left Drag`: Rotate  
`Middle Drag`: Pan  
`Middle Wheel`: Zoom  
