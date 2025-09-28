# alpine
**alpine** is a CPU path tracer, and is developed to experiment with various rendering algorithms.

## Requirements
- Visual Studio 2022
- C++ 20
- CMake 3.15
- OpenGL 4.6 (optional, required for [viewer](#viewer))

## Build Instructions
1. Fetch git submodules
    ```
    git submodule update --init
    ```

2. Generate build files using CMake in the root directory
    ```
    cmake -S . -B build
    ```
    You can enable optional features via the following CMake options.  
    For each option, make sure to manually download the required library and place it in the specified directory under `ext/`:  

    | CMake Option | Description | Required Library | Place in |
    | - | - | - | - |
    | `ALPINE_BUILD_APPS`    | Builds alpine [sample applications](#sample-applications). | [GLFW](https://www.glfw.org/) 3.4 (for [viewer](#viewer)) | `ext/glfw` |
    | `ALPINE_ENABLE_EMBREE` | Enables Embree as an accelerator backend option. | [Intel Embree](https://www.embree.org/) 4.3.3             | `ext/embree` |
    | `ALPINE_ENABLE_OIDN`   | Enables a denoising option via OIDN. | [Intel Open Image Denoise](https://www.openimagedenoise.org/) 2.3.0 | `ext/oidn`   |
    | `ALPINE_ENABLE_ISPC`   | Enables ISPC-based SIMD code generation. | [Intel ISPC](https://ispc.github.io/) 1.26.0              | `ext/ispc`   |


3. Open the solution file `alpine.sln` generated in the `build` directory, and build it in Visual Studio

## Sample Applications
### simple
*simple* is an application which generates an image using path tracing. It takes either an obj file or a glb file as input, and outputs a ppm file.

Options:  
`-i, --input`: Input file (.obj|.glb)  
`-o, --output`: Output image file (.ppm)  
`--spp`: Number of samples per pixel  
`--accelerator`: Accelerator type: "bvh", "wideBvh" or "embree"  
`--lightSampler`: Light sampler type: "uniform", "power", or "bvh"  
`--denoiser`: Enable denoiser  

Example:  
```
simple -i input.glb -o output.ppm --spp 64 --accelerator bvh --lightSampler bvh --denoiser
```

### viewer
*viewer* is a progressive path tracer, and takes a glb file as input.
```
viewer input.glb
```
<img src="images/viewer.png" width="30%">  

The camera can be manipulated by a mouse.  
`Left Drag`: Rotate  
`Middle Drag`: Pan  
`Middle Wheel`: Zoom  

## Images
![room](images/room.png)  
![galaxy](images/galaxy.png)  
