#pragma once
#include "../general/bodysystem.h"

class BodySystemCUDA : public BodySystem
{
public:
    BodySystemCUDA(int numBodies);
    BodySystemCUDA(int numBodies, unsigned int p, unsigned int q);
    virtual ~BodySystemCUDA();

    virtual void update(float deltaTime);

    virtual void setDamping(float damping);
    virtual void setTiles(bool tiles);

    virtual float* getArray(BodyArray array);
    virtual void   setArray(BodyArray array, const float* data);

    virtual unsigned int getCurrentReadBuffer() const { return m_pbo[m_currentRead]; }

protected:
    BodySystemCUDA() {}

    virtual void _initialize(int numBodies);
    virtual void _finalize();

    
    
protected:
    float* m_hPos;
    float* m_hVel;

    float* m_dPos[2];
    float* m_dVel[2];

    float m_damping;

    unsigned int m_pbo[2];
    unsigned int m_currentRead;
    unsigned int m_currentWrite;

    unsigned int m_timer;

    unsigned int m_p;
    unsigned int m_q;
    bool m_tiles;
};
