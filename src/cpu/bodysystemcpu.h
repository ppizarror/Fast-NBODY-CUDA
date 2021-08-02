#pragma once
#include "../general/bodysystem.h"

// CPU Body System
class BodySystemCPU : public BodySystem
{
public:
    BodySystemCPU(int numBodies);
    virtual ~BodySystemCPU();

    virtual void update(float deltaTime);

    virtual void setDamping(float damping)     { m_damping = damping; }

    virtual float* getArray(BodyArray array);
    virtual void   setArray(BodyArray array, const float* data);

    virtual unsigned int getCurrentReadBuffer() const { return m_currentRead; }

protected: // methods
    BodySystemCPU() {} // default constructor

    virtual void _initialize(int numBodies);
    virtual void _finalize();

    void _computePosition(float deltaTime, int i_min, int i_max);
    void _computeNBodyGravitation(int j_min, int j_max);

    void _computeGravitationParallel();
    void _computePositionParallel(float deltaTime);

    void _integrateNBodySystem(float deltaTime);
    
protected: // data
    float* m_pos[2];
    float* m_vel[2];
    float* m_force;

    float m_damping;

    unsigned int m_currentRead;
    unsigned int m_currentWrite;

    unsigned int m_timer;
};
