#include "bodysystemcuda.h"

#include <cutil.h>

#include <assert.h>
#include <memory.h>
#include <cstdio>
#include <algorithm>
#include "../general/external.h"

extern "C"
{
    void checkCUDA();
    void allocateNBodyArrays(float* vel[2], int numBodies);
    void deleteNBodyArrays(float* vel[2]);
    void integrateNbodySystem(float* newPos, float* newVel, 
                              float* oldPos, float* oldVel,
                              unsigned int pboOldPos, unsigned int pboNewPos,
                              float deltaTime, float damping, 
                              int numBodies, int p, int q, bool tiles);
    void copyArrayFromDevice(float* host, const float* device, unsigned int pbo, int numBodies);
    void copyArrayToDevice(float* device, const float* host, int numBodies);
    void registerGLBufferObject(unsigned int pbo);
    void unregisterGLBufferObject(unsigned int pbo);
}


BodySystemCUDA::BodySystemCUDA(int numBodies)
: BodySystem(numBodies),
  m_hPos(0),
  m_hVel(0),
  m_currentRead(0),
  m_currentWrite(1),
  m_timer(0),
  m_p(256),
  m_q(1)
{
    m_dPos[0] = m_dPos[1] = 0;
    m_dVel[0] = m_dVel[1] = 0;

    _initialize(numBodies);
    setDamping(1.0f);
}

BodySystemCUDA::BodySystemCUDA(int numBodies, unsigned int p, unsigned int q)
: BodySystem(numBodies),
  m_hPos(0),
  m_hVel(0),
  m_currentRead(0),
  m_currentWrite(1),
  m_timer(0),
  m_p(p),
  m_q(q),
  m_tiles(false)
{
    m_dPos[0] = m_dPos[1] = 0;
    m_dVel[0] = m_dVel[1] = 0;

    _initialize(numBodies);
    setDamping(1.0f);
}

BodySystemCUDA::~BodySystemCUDA()
{
    _finalize();
    m_numBodies = 0;
}

void
BodySystemCUDA::_initialize(int numBodies)
{
    assert(!m_bInitialized);
    checkCUDA();

    m_numBodies = numBodies;

    m_hPos = new float[m_numBodies*4];
    m_hVel = new float[m_numBodies*4];

    memset(m_hPos, 0, m_numBodies*4*sizeof(float));
    memset(m_hVel, 0, m_numBodies*4*sizeof(float));

    // create the position pixel buffer objects for rendering
    // we will actually compute directly from this memory in CUDA too
    glGenBuffers(2, m_pbo);   
    for (int i = 0; i < 2; ++i)
    {
        glBindBuffer(GL_ARRAY_BUFFER, m_pbo[i]);
        glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(float) * m_numBodies, 
                     m_hPos, GL_DYNAMIC_DRAW);

        int size = 0;
        glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &size); 
        if ((unsigned)size != 4 * (sizeof(float) * m_numBodies))
            fprintf(stderr, "WARNING: Pixel Buffer Object allocation failed!n");
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        registerGLBufferObject(m_pbo[i]);
    }

    allocateNBodyArrays(m_dVel, m_numBodies);

    CUT_SAFE_CALL(cutCreateTimer(&m_timer));

    m_bInitialized = true;
}

void
BodySystemCUDA::_finalize()
{
    assert(m_bInitialized);

    delete [] m_hPos;
    delete [] m_hVel;

    deleteNBodyArrays(m_dVel);
    unregisterGLBufferObject(m_pbo[0]);
    unregisterGLBufferObject(m_pbo[1]);
    glDeleteBuffers(2, m_pbo);
}

void
BodySystemCUDA::setDamping(float damping)
{
    m_damping = damping;
}

void BodySystemCUDA::setTiles(bool tiles) {m_tiles = tiles;}

void BodySystemCUDA::update(float deltaTime)
{
    assert(m_bInitialized);

    CUT_SAFE_CALL( cutStartTimer(m_timer));

    integrateNbodySystem(m_dPos[m_currentWrite], m_dVel[m_currentWrite], 
                         m_dPos[m_currentRead], m_dVel[m_currentRead],
                         m_pbo[m_currentRead], m_pbo[m_currentWrite],
                         deltaTime, m_damping, m_numBodies, m_p, m_q, m_tiles);
    CUT_SAFE_CALL( cutStopTimer(m_timer));

    std::swap(m_currentRead, m_currentWrite);

    CUT_SAFE_CALL( cutResetTimer(m_timer));
}

float* 
BodySystemCUDA::getArray(BodyArray array)
{
    assert(m_bInitialized);
 
    float* hdata = 0;
    float* ddata = 0;

    unsigned int pbo = 0;

    switch (array)
    {
    default:
    case BODYSYSTEM_POSITION:
        hdata = m_hPos;
        ddata = m_dPos[m_currentRead];
        pbo = m_pbo[m_currentRead];
        break;
    case BODYSYSTEM_VELOCITY:
        hdata = m_hVel;
        ddata = m_dVel[m_currentRead];
        break;
    }

    copyArrayFromDevice(hdata, ddata, pbo, m_numBodies);
    return hdata;
}

void
BodySystemCUDA::setArray(BodyArray array, const float* data)
{
    assert(m_bInitialized);
 
    switch (array)
    {
    default:
    case BODYSYSTEM_POSITION:
        {
            unregisterGLBufferObject(m_pbo[m_currentRead]);
            glBindBuffer(GL_ARRAY_BUFFER, m_pbo[m_currentRead]);
            glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(float) * m_numBodies, 
                        data, GL_DYNAMIC_DRAW);

            int size = 0;
            glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &size); 
            if ((unsigned)size != 4 * (sizeof(float) * m_numBodies))
                fprintf(stderr, "WARNING: Pixel Buffer Object download failed!n");
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            registerGLBufferObject(m_pbo[m_currentRead]);
        }
        break;
    case BODYSYSTEM_VELOCITY:
        copyArrayToDevice(m_dVel[m_currentRead], data, m_numBodies);
        break;
    }       
}
