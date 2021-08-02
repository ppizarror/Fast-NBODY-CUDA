#include "nbody_kernel.cu"
#include <cutil.h>
#include <cstdlib>
#include <cstdio>
#include <string.h>
#include "../general/external.h"
#include <cuda_gl_interop.h>

extern "C"
{

void checkCUDA()
{   
    CUT_CHECK_DEVICE();
}

void allocateNBodyArrays(float* vel[2], int numBodies)
{
    // 4 floats each for alignment reasons
    unsigned int memSize = sizeof( float) * 4 * numBodies;
    
    CUDA_SAFE_CALL(cudaMalloc((void**)&vel[0], memSize));
    CUDA_SAFE_CALL(cudaMalloc((void**)&vel[1], memSize));
}

void deleteNBodyArrays(float* vel[2])
{
    CUDA_SAFE_CALL(cudaFree((void**)vel[0]));
    CUDA_SAFE_CALL(cudaFree((void**)vel[1]));
}

void copyArrayFromDevice(float* host, const float* device, unsigned int pbo, int numBodies)
{   
    if (pbo)
        CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&device, pbo));
    CUDA_SAFE_CALL(cudaMemcpy(host, device, numBodies*4*sizeof(float), cudaMemcpyDeviceToHost));
    if (pbo)
        CUDA_SAFE_CALL(cudaGLUnmapBufferObject(pbo));
}

void copyArrayToDevice(float* device, const float* host, int numBodies)
{
    CUDA_SAFE_CALL(cudaMemcpy(device, host, numBodies*4*sizeof(float), cudaMemcpyHostToDevice));
}

void registerGLBufferObject(unsigned int pbo)
{
    CUDA_SAFE_CALL(cudaGLRegisterBufferObject(pbo));
}

void unregisterGLBufferObject(unsigned int pbo)
{
    CUDA_SAFE_CALL(cudaGLUnregisterBufferObject(pbo));
}

void integrateNbodySystem(
        float* newPos, float* newVel,
        float* oldPos, float* oldVel,
        unsigned int pboOldPos, unsigned int pboNewPos,
        float deltaTime, float damping,
        int numBodies, int p, int q, bool tiles
        ) {
	int sharedMemSize = p*q*sizeof(float4); // 4 floats for pos

	dim3 threads;
	if (tiles) threads = dim3(p,q,1);
    else threads = dim3(p,1,1);
    dim3 grid(numBodies/p, 1, 1);

    CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&oldPos, pboOldPos));
    CUDA_SAFE_CALL(cudaGLMapBufferObject((void**)&newPos, pboNewPos));

    // execute the kernel:

    if (tiles) integrateBodies<true><<< grid, threads, sharedMemSize >>>((float4*)newPos, (float4*)newVel,
                                                                   (float4*)oldPos, (float4*)oldVel,
                                                                   deltaTime, damping, numBodies);
    else integrateBodies<false><<< grid, threads, sharedMemSize >>>((float4*)newPos, (float4*)newVel,
                                                                   (float4*)oldPos, (float4*)oldVel,
                                                                   deltaTime, damping, numBodies);

    // check if kernel invocation generated an error
    CUT_CHECK_ERROR("Kernel execution failed");

    CUDA_SAFE_CALL(cudaGLUnmapBufferObject(pboNewPos));
    CUDA_SAFE_CALL(cudaGLUnmapBufferObject(pboOldPos));

    
}


}
