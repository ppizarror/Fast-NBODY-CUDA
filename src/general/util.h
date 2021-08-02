#pragma once

#include <math.h>
#include <stdlib.h>
#include <algorithm>

enum NBodyConfig {
    NBODY_CONFIG_RANDOM,
    NBODY_CONFIG_SHELL,
    NBODY_CONFIG_EXPAND,
    NBODY_NUM_CONFIGS
};


struct vec3 { float x, y, z; };

float normalize(vec3& vector) {
    float dist = sqrtf(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
    if (dist > 1e-6) {
        vector.x /= dist;
        vector.y /= dist;
        vector.z /= dist;
    }
    return dist;
}

float dot(vec3 v0, vec3 v1) {
    return v0.x*v1.x+v0.y*v1.y+v0.z*v1.z;
}

vec3 cross(vec3 v0, vec3 v1) {
    vec3 rt;
    rt.x = v0.y*v1.z-v0.z*v1.y;
    rt.y = v0.z*v1.x-v0.x*v1.z;
    rt.z = v0.x*v1.y-v0.y*v1.x;
    return rt;
}

const float m = 0.0f;
const float s = 1.0f;
const float alpha = 3.0f;
float getMass() {
    float y = rand()/(float) RAND_MAX;
    float w = -log(1-y);
    float x = pow(w, -1.f/alpha)*s + m;
    return x;
}

void configRandom(float* pos, float* vel, float clusterScale, float velocityScale, int numBodies) {
    float scale = clusterScale * std::max(1.0f, numBodies / (1024.f));
    float vscale = velocityScale * scale;

    int p = 0, v = 0;
    int i = 0;
    while (i<numBodies) {
        vec3 point;
        //const int scale = 16;
        point.x = rand() / (float) RAND_MAX * 2 - 1;
        point.y = rand() / (float) RAND_MAX * 2 - 1;
        point.z = rand() / (float) RAND_MAX * 2 - 1;
        float lenSqr = dot(point, point);
        if (lenSqr > 1) continue;

        vec3 velocity;
        velocity.x = rand() / (float) RAND_MAX * 2 - 1;
        velocity.y = rand() / (float) RAND_MAX * 2 - 1;
        velocity.z = rand() / (float) RAND_MAX * 2 - 1;
        lenSqr = dot(velocity, velocity);
        if (lenSqr > 1)
            continue;

        pos[p++] = point.x * scale; // pos.x
        pos[p++] = point.y * scale; // pos.y
        pos[p++] = point.z * scale; // pos.z
        pos[p++] = 1.0f; // mass

        vel[v++] = velocity.x * vscale; // pos.x
        vel[v++] = velocity.y * vscale; // pos.x
        vel[v++] = velocity.z * vscale; // pos.x
        vel[v++] = 1.0f; // inverse mass

        i++;
    }
}

void configShell(float* pos, float* vel, float clusterScale, float velocityScale, int numBodies) {
    float scale = clusterScale;
    float vscale = scale * velocityScale;
    float inner = 2.5f * scale;
    float outer = 4.0f * scale;

    int p = 0, v=0;
    int i = 0;
    while (i<numBodies) {
        float x, y, z;
        x = rand() / (float) RAND_MAX * 2 - 1;
        y = rand() / (float) RAND_MAX * 2 - 1;
        z = rand() / (float) RAND_MAX * 2 - 1;

        vec3 point = {x, y, z};
        float len = normalize(point);
        if (len > 1)
            continue;

        pos[p++] =  point.x * (inner + (outer - inner) * rand() / (float) RAND_MAX);
        pos[p++] =  point.y * (inner + (outer - inner) * rand() / (float) RAND_MAX);
        pos[p++] =  point.z * (inner + (outer - inner) * rand() / (float) RAND_MAX);
        pos[p++] = 1.0f;

        x = 0.0f; // * (rand() / (float) RAND_MAX * 2 - 1);
        y = 0.0f; // * (rand() / (float) RAND_MAX * 2 - 1);
        z = 1.0f; // * (rand() / (float) RAND_MAX * 2 - 1);
        vec3 axis = {x, y, z};
        normalize(axis);

        if (1 - dot(point, axis) < EPS) {
            axis.x = point.y;
            axis.y = point.x;
            normalize(axis);
        }
        vec3 vv = {pos[4*i], pos[4*i+1], pos[4*i+2]};
        vv = cross(vv, axis);
        vel[v++] = vv.x * vscale;
        vel[v++] = vv.y * vscale;
        vel[v++] = vv.z * vscale;
        vel[v++] = 1.0f;

        i++;
    }
}

void configExpand(float* pos, float* vel, float clusterScale, float velocityScale, int numBodies) {
    float scale = clusterScale * std::max(1.0f, numBodies / (1024.f));
    float vscale = scale * velocityScale;

    int p = 0, v = 0;
    for(int i=0; i < numBodies;)
    {
        vec3 point;

        point.x = rand() / (float) RAND_MAX * 2 - 1;
        point.y = rand() / (float) RAND_MAX * 2 - 1;
        point.z = rand() / (float) RAND_MAX * 2 - 1;

        float lenSqr = dot(point, point);
        if (lenSqr > 1)
            continue;

        pos[p++] = point.x * scale; // pos.x
        pos[p++] = point.y * scale; // pos.y
        pos[p++] = point.z * scale; // pos.z
        pos[p++] = 1.0f; // mass
        vel[v++] = point.x * vscale; // pos.x
        vel[v++] = point.y * vscale; // pos.x
        vel[v++] = point.z * vscale; // pos.x
        vel[v++] = 1.0f; // inverse mass

        i++;
    }
}

void randomizeBodies(NBodyConfig config, float* pos, float* vel, float* color, float clusterScale, float velocityScale, int numBodies) {
    switch(config) {
        default:
        case NBODY_CONFIG_RANDOM:
            configRandom(pos, vel, clusterScale, velocityScale, numBodies);
            break;
        case NBODY_CONFIG_SHELL:
            configShell(pos, vel, clusterScale, velocityScale, numBodies);
            break;
        case NBODY_CONFIG_EXPAND:
            configExpand(pos, vel, clusterScale, velocityScale, numBodies);
            break;
    }

    if (color) {
        int v = 0;
        for(int i=0; i < numBodies; i++) {
            //const int scale = 16;
            color[v++] = rand() / (float) RAND_MAX;
            color[v++] = rand() / (float) RAND_MAX;
            color[v++] = rand() / (float) RAND_MAX;
            color[v++] = 1.0f;
        }
    }
}
