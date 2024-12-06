# Voxelization usage

## Blueprint

1. Right-click in any blueprint editor and add the node: "ExecuteBaseComputeShader"
2. This should give you a node with 2 inputs and 1 output. The output = A * B
3. Enjoy, and get digging into the code

## C++

```cpp
// Params struct used to pass args to our compute shader
FVoxelizationDispatchParams Params(1, 1, 1);

// Fill in your input parameters here
Params.XYZ = 123;

// These are defined/used in:
// 1. Private/Voxelization/Voxelization.cpp under BEGIN_SHADER_PARAMETER_STRUCT
// 2. Public/Voxelization/Voxelization.h under FVoxelizationDispatchParams
// 3. Private/Voxelization/Voxelization.cpp under FVoxelizationInterface::DispatchRenderThread

// Executes the compute shader and blocks until complete. You can place outputs in the params struct
FVoxelizationInterface::Dispatch(Params);
```

Feel free to delete this file
