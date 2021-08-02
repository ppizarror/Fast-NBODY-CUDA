#pragma once
#include "../general/constants.h"
#include <stdio.h>

#define LOOP_UNROLL 4
#define SX(i) &sharedPos[i+blockDim.x*threadIdx.y]
#define SX_SUM(i,j) sharedPos[i+blockDim.x*j]

__constant__ float eps = EPS;

__device__ void bodyBodyInteractionCUDA(float3* ai, float4* bi, float4* bj) {
    float dx = bi->x - bj->x;
    float dy = bi->y - bj->y;
    float dz = bi->z - bj->z;

    float distSqr = dx*dx+dy*dy+dz*dz;
    if (distSqr<EPS) return;

    float distSixth = distSqr * distSqr * distSqr;
    float invDistCube = 1.0f/sqrtf(distSixth);

    float s = bi->w * invDistCube;

    ai->x += dx * s;
    ai->y += dy * s;
    ai->z += dz * s;
}


__device__ void gravitationTiles(float4* myPos, float3* accel) {
    extern __shared__ float4 sharedPos[];
    int i;

    for (i=0; i<blockDim.x;) {
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
#if LOOP_UNROLL > 1
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
#endif
#if LOOP_UNROLL > 2
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
#endif
#if LOOP_UNROLL > 4
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
        bodyBodyInteractionCUDA(accel, SX(i), myPos); i += 1;
#endif
    }
}

#define WRAP(x,m) (((x)<m)?(x):(x-m))

__device__ float3 computeAccelerationRows(float4* bj, float4* positions, int numBodies) {
    float3 acc = {0.0f, 0.0f, 0.0f};
    for (int i=0; i<numBodies; i++) bodyBodyInteractionCUDA(&acc, &positions[i], bj);
    return acc;
}

__device__ float3 computeAccelerationTiles(float4* bodyPos, float4* positions, int numBodies) {
    extern __shared__ float4 sharedPos[];

    float3 acc = {0.0f, 0.0f, 0.0f};

    int p = blockDim.x;
    int q = blockDim.y;
    int n = numBodies;

    int start = n/q * threadIdx.y;
    int tile0 = start/(n/q);
    int tile = tile0;
    int finish = start + n/q;

    for (int i = start; i < finish; i += p, tile++)
    {
        sharedPos[threadIdx.x+blockDim.x*threadIdx.y] =
                positions[(WRAP(blockIdx.x+tile, gridDim.x)*blockDim.y + threadIdx.y )* blockDim.x + threadIdx.x];

        __syncthreads();
        gravitationTiles(bodyPos, &acc);
        __syncthreads();
    }

    SX_SUM(threadIdx.x, threadIdx.y).x = acc.x;
    SX_SUM(threadIdx.x, threadIdx.y).y = acc.y;
    SX_SUM(threadIdx.x, threadIdx.y).z = acc.z;

    __syncthreads();

    if (threadIdx.y == 0) {
        for (int i=1; i<blockDim.y; i++) {
            acc.x += SX_SUM(threadIdx.x,i).x;
            acc.y += SX_SUM(threadIdx.x,i).y;
            acc.z += SX_SUM(threadIdx.x,i).z;
        }
    }

    return acc;
}

template<bool tiles> __global__ void integrateBodies(
        float4* newPos, float4* newVel,
        float4* oldPos, float4* oldVel,
        float deltaTime, float damping, int numBodies) {

    int index = __mul24(blockIdx.x,blockDim.x) + threadIdx.x;
	float4 bodyPos = oldPos[index];

	float3 accel;

    if (tiles) accel = computeAccelerationTiles(&bodyPos, oldPos, numBodies);
    else accel = computeAccelerationRows(&bodyPos, oldPos, numBodies);
    float4 vel = oldVel[index];

    vel.x += accel.x * deltaTime;
    vel.y += accel.y * deltaTime;
    vel.z += accel.z * deltaTime;

    vel.x *= damping;
    vel.y *= damping;
    vel.z *= damping;

    bodyPos.x += vel.x * deltaTime;
    bodyPos.y += vel.y * deltaTime;
    bodyPos.z += vel.z * deltaTime;

    newPos[index] = bodyPos;
    newVel[index] = vel;
}
