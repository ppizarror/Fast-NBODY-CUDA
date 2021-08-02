#include "bodysystemcpu.h"
#include "../general/constants.h"

#include <assert.h>
#include <memory.h>
#include <cutil.h>
#include <math.h>
#include <algorithm>


const float eps = EPS;

void bodyBodyInteractionCPU(float* accel, const float posMass_i[4], const float posMass_j[4]) {
    float dx = posMass_i[0] - posMass_j[0];
    float dy = posMass_i[1] - posMass_j[1];
    float dz = posMass_i[2] - posMass_j[2];

    float distSqr = dx*dx+dy*dy+dz*dz;
    if (distSqr<EPS) return;

    float distSixth = distSqr * distSqr * distSqr;
    float invDistCube = 1.0f/sqrtf(distSixth);

    float s = posMass_i[3] * invDistCube;

    accel[0] += dx * s;
    accel[1] += dy * s;
    accel[2] += dz * s;
}

BodySystemCPU::BodySystemCPU(int numBodies)
: BodySystem(numBodies),
  m_force(0),
  m_damping(1.0f),
  m_currentRead(0),
  m_currentWrite(0),
  m_timer(0)
{
    m_pos[0] = m_pos[1] = 0;
    m_vel[0] = m_vel[1] = 0;

    _initialize(numBodies);
}

BodySystemCPU::~BodySystemCPU()
{
    _finalize();
    m_numBodies = 0;
}

void
BodySystemCPU::_initialize(int numBodies)
{
    assert(!m_bInitialized);

    m_numBodies = numBodies;

    m_pos[0] = new float[m_numBodies*4];
    m_pos[1] = new float[m_numBodies*4];
    m_vel[0] = new float[m_numBodies*4];
    m_vel[1] = new float[m_numBodies*4];
    m_force  = new float[m_numBodies*4];

    memset(m_pos[0], 0, m_numBodies*4*sizeof(float));
    memset(m_pos[1], 0, m_numBodies*4*sizeof(float));
    memset(m_vel[0], 0, m_numBodies*4*sizeof(float));
    memset(m_vel[1], 0, m_numBodies*4*sizeof(float));
    memset(m_force, 0, m_numBodies*4*sizeof(float));
    
    CUT_SAFE_CALL(cutCreateTimer(&m_timer));

    m_bInitialized = true;
}

void
BodySystemCPU::_finalize()
{
    assert(m_bInitialized);

    delete [] m_pos[0];
    delete [] m_pos[1];
    delete [] m_vel[0];
    delete [] m_vel[1];
    delete [] m_force;

    CUT_SAFE_CALL(cutDeleteTimer(m_timer));
}


void 
BodySystemCPU::update(float deltaTime)
{
    assert(m_bInitialized);

    CUT_SAFE_CALL( cutStartTimer(m_timer));
    _integrateNBodySystem(deltaTime);
    CUT_SAFE_CALL( cutStopTimer(m_timer));

    std::swap(m_currentRead, m_currentWrite);

    CUT_SAFE_CALL( cutResetTimer(m_timer));
}

float* 
BodySystemCPU::getArray(BodyArray array)
{
    assert(m_bInitialized);
 
    float* data = 0;
    switch (array) {
    default:
    case BODYSYSTEM_POSITION:
        data = m_pos[m_currentRead];
        break;
    case BODYSYSTEM_VELOCITY:
        data = m_vel[m_currentRead];
        break;
    }

    return data;
}

void 
BodySystemCPU::setArray(BodyArray array, const float* data)
{
    assert(m_bInitialized);

    float* target = 0;

    switch (array)
    {
    default:
    case BODYSYSTEM_POSITION:
        target = m_pos[m_currentRead];
        break;
    case BODYSYSTEM_VELOCITY:
        target = m_vel[m_currentRead];
        break;
    }

    memcpy(target, data, m_numBodies*4*sizeof(float));
}

void BodySystemCPU::_computeNBodyGravitation(int j_min, int j_max) {
    for(int j = j_min; j < j_max & j<m_numBodies; ++j) {
        m_force[j*4] = m_force[j*4+1] = m_force[j*4+2] = 0;
        float acc[3] = {0.0f, 0.0f, 0.0f};
        for(int i = 0; i < m_numBodies; ++i) {
            bodyBodyInteractionCPU(acc, &m_pos[m_currentRead][i*4], &m_pos[m_currentRead][j*4]);
        }
        for (int k=0; k<3; ++k) m_force[j*4+k] += acc[k];
    }
}

void BodySystemCPU::_computePosition(float deltaTime, int i_min, int i_max) {
    for(int i = i_min; i < i_max & i<m_numBodies; ++i) {
        int index = 4*i;
        float pos[3], vel[3], accel[3];

        pos[0] = m_pos[m_currentRead][index+0];
        pos[1] = m_pos[m_currentRead][index+1];
        pos[2] = m_pos[m_currentRead][index+2];
        float mass = m_pos[m_currentRead][index+3];

        vel[0] = m_vel[m_currentRead][index+0];
        vel[1] = m_vel[m_currentRead][index+1];
        vel[2] = m_vel[m_currentRead][index+2];
        float invMass = m_vel[m_currentRead][index+3];

        accel[0] = m_force[index+0];
        accel[1] = m_force[index+1];
        accel[2] = m_force[index+2];

        vel[0] += accel[0] * deltaTime;
        vel[1] += accel[1] * deltaTime;
        vel[2] += accel[2] * deltaTime;

        vel[0] *= m_damping;
        vel[1] *= m_damping;
        vel[2] *= m_damping;

        pos[0] += vel[0] * deltaTime;
        pos[1] += vel[1] * deltaTime;
        pos[2] += vel[2] * deltaTime;

        m_pos[m_currentWrite][index+0] = pos[0];
        m_pos[m_currentWrite][index+1] = pos[1];
        m_pos[m_currentWrite][index+2] = pos[2];
        m_pos[m_currentWrite][index+3] = mass;

        m_vel[m_currentWrite][index+0] = vel[0];
        m_vel[m_currentWrite][index+1] = vel[1];
        m_vel[m_currentWrite][index+2] = vel[2];
        m_vel[m_currentWrite][index+3] = invMass;
    }
}

#include <ppl.h>
const int divisions = 16;

void BodySystemCPU::_computeGravitationParallel() {
    int step = (m_numBodies+divisions-1)/divisions;
    auto computeGroup = [&] (size_t index) {
        int j_min = index*step;
        int j_max = (index+1)*step;
        _computeNBodyGravitation(j_min, j_max);
    };
    Concurrency::parallel_for<size_t>(0, divisions, 1, computeGroup);
}

void BodySystemCPU::_computePositionParallel(float deltaTime) {
    int step = (m_numBodies+divisions-1)/divisions;
    auto computeGroup = [&] (size_t index) {
        int j_min = index*step;
        int j_max = (index+1)*step;
        _computePosition(deltaTime, j_min, j_max);
    };
    Concurrency::parallel_for<size_t>(0, divisions, 1, computeGroup);
}


void BodySystemCPU::_integrateNBodySystem(float deltaTime)
{
//    _computeNBodyGravitation(0, m_numBodies);
//    _computePosition(0, m_numBodies);

    _computeGravitationParallel();
    _computePositionParallel(deltaTime);
}
