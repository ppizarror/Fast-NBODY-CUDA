#include "general/external.h"

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <cutil.h>
#include <paramgl.h>

#include "cuda/bodysystemcuda.h"
#include "cpu/bodysystemcpu.h"
#include "render/render_particles.h"
#include "general/constants.h"
#include "general/util.h"

// view params
int ox = 0, oy = 0;
int buttonState = 0;
float camera_trans[] = {0, -2, -100};
float camera_rot[]   = {0, 0, 0};
float camera_trans_lag[] = {0, -2, -100};
float camera_rot_lag[] = {0, 0, 0};
const float inertia = 0.1;
ParticleRenderer::DisplayMode displayMode = ParticleRenderer::PARTICLE_SPRITES_COLOR;

bool displayEnabled = true;
bool bPause = false;

int numBodies = N_BODIES;
int numIterations = 0; // run until exit

float clusterScale = 1.54f;
float velocityScale = 8.f;
float damping = 1.0f; // no damping by default
float timestep = 0.016f;

BodySystemCUDA *nbodyCUDA = 0;
BodySystemCPU  *nbodyCPU = 0;

float* hPos = 0;
float* hVel = 0;
float* hColor = 0;

//float* hPos = 0;
//float* hVel = 0;

// The UI
ParamListGL *paramlist;  // parameter list
bool bShowSliders = true;

// fps
static int fpsCount = 0;
static int fpsLimit = 1;
unsigned int timer = 0;

ParticleRenderer *renderer = 0;

void reset(NBodyConfig config)
{
    randomizeBodies(config, hPos, hVel, hColor, clusterScale, velocityScale, numBodies);

    nbodyCUDA->setArray(BodySystem::BODYSYSTEM_POSITION, hPos);
    nbodyCUDA->setArray(BodySystem::BODYSYSTEM_VELOCITY, hVel);
    renderer->setColors(hColor, nbodyCUDA->getNumBodies());
}

void init(int numBodies, int p, int q, bool tiles)
{
    nbodyCUDA = new BodySystemCUDA(numBodies, p, q);
    nbodyCUDA->setTiles(tiles);

    // allocate host memory
    hPos = new float[numBodies*4];
    hVel = new float[numBodies*4];
    hColor = new float[numBodies*4];

    nbodyCUDA->setDamping(damping);

    CUT_SAFE_CALL(cutCreateTimer(&timer));
}

void compare(float* cudaPos, float* cpuPos) {
    for (int i=0; i<numBodies; i++) {
        float dx = cudaPos[4*i]-cpuPos[4*i];
        float dy = cudaPos[4*i+1]-cpuPos[4*i+1];
        float dz = cudaPos[4*i+2]-cpuPos[4*i+2];
        float norm = dx*dx+dy*dy+dz*dz;

        if (norm>EPS)
        printf("%d: (%f %f %f)\t(%f %f %f)\n", i,
               cudaPos[4*i], cudaPos[4*i+1], cudaPos[4*i+2],
               cpuPos[4*i], cpuPos[4*i+1], cpuPos[4*i+2]
        );
    }
}

void compareResults(int numBodies) {
    nbodyCPU = new BodySystemCPU(numBodies);
    nbodyCPU->setArray(BodySystem::BODYSYSTEM_POSITION, hPos);
    nbodyCPU->setArray(BodySystem::BODYSYSTEM_VELOCITY, hVel);

    for (int i=0; i<numIterations; i++) {
        nbodyCUDA->update(timestep);
        nbodyCPU->update(timestep);
    }

    float* cudaPos = nbodyCUDA->getArray(BodySystem::BODYSYSTEM_POSITION);
    float* cpuPos  = nbodyCPU->getArray(BodySystem::BODYSYSTEM_POSITION);
    CUTBoolean res = cutComparefe(cpuPos, cudaPos, numBodies, .01f);
    printf( "Test %s\n", (res==1) ? "PASSED" : "FAILED");
}

// check for OpenGL errors
void checkGLErrors(char *s)
{
    GLenum error;
    while ((error = glGetError()) != GL_NO_ERROR) {
        fprintf(stderr, "%s: error - %s\n", s, (char *) gluErrorString(error));
    }
}

void initGL()
{  
    glewInit();
    if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
        fprintf(stderr, "Required OpenGL extensions missing.");
        exit(-1);
    }

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 1.0);

    renderer = new ParticleRenderer;
    
    checkGLErrors("initGL");
}

void initParameters()
{
	// create a new parameter list
	paramlist = new ParamListGL("sliders");
	paramlist->bar_col_outer[0] = 0.8f;
	paramlist->bar_col_outer[1] = 0.8f;
	paramlist->bar_col_outer[2] = 0.0f;
	paramlist->bar_col_inner[0] = 0.8f;
	paramlist->bar_col_inner[1] = 0.8f;
	paramlist->bar_col_inner[2] = 0.0f;

	// add some parameters to the list

	// Velocity Damping
	paramlist->AddParam(new Param<float>("Velocity Damping", damping, 
		                0.95, 1, .0001, &damping));
    // Time step size
    paramlist->AddParam(new Param<float>("Time Step", timestep, 
		                0, 1, .0001, &timestep));
    // Cluster scale (only affects starting configuration
    paramlist->AddParam(new Param<float>("Cluster Scale", clusterScale, 
		                0, 10, 0.1, &clusterScale));
    
    // Velocity scale (only affects starting configuration)
    paramlist->AddParam(new Param<float>("Velocity Scale", velocityScale, 
		                0, 20, 0.1, &velocityScale));
}

void computePerfStats(double &interactionsPerSecond, double &gflops, float milliseconds, int iterations)
{
    const int flopsPerInteraction = 20;
    interactionsPerSecond = (float)numBodies * (float)numBodies;
    interactionsPerSecond *= 1e-9 * iterations * 1000 / milliseconds;
    gflops = interactionsPerSecond * (float)flopsPerInteraction;

}

void runBenchmark(int iterations, bool cpu)
{
    if (cpu) {
        nbodyCPU = new BodySystemCPU(numBodies);
        nbodyCPU->setArray(BodySystem::BODYSYSTEM_POSITION, hPos);
        nbodyCPU->setArray(BodySystem::BODYSYSTEM_VELOCITY, hVel);
    }

    // once without timing to prime the GPU
    nbodyCUDA->update(timestep);

    CUT_SAFE_CALL(cutStartTimer(timer));  
    for (int i = 0; i < iterations; ++i) nbodyCUDA->update(timestep);
    CUT_SAFE_CALL(cutStopTimer(timer));  

    float milliseconds = cutGetTimerValue(timer);
    double interactionsPerSecond = 0;
    double gflops = 0;
    computePerfStats(interactionsPerSecond, gflops, milliseconds, iterations);

    printf("CUDA:\n");
    printf("%d bodies, total time for %d iterations: %0.3f ms\n", numBodies, iterations, milliseconds);
    printf("= %0.3f billion interactions per second\n", interactionsPerSecond);
    printf("= %0.3f GFLOP/s at %d flops per interaction\n", gflops, 20);

    if (!cpu) return;

    printf("\nCPU:\n");
    CUT_SAFE_CALL(cutStartTimer(timer));
    for (int i = 0; i < iterations; ++i) nbodyCPU->update(timestep);
    CUT_SAFE_CALL(cutStopTimer(timer));

    milliseconds = cutGetTimerValue(timer);
    interactionsPerSecond = 0;
    gflops = 0;
    computePerfStats(interactionsPerSecond, gflops, milliseconds, iterations);

    printf("%d bodies, total time for %d iterations: %0.3f ms\n", numBodies, iterations, milliseconds);
    printf("= %0.3f billion interactions per second\n", interactionsPerSecond);
    printf("= %0.3f GFLOP/s at %d flops per interaction\n", gflops, 20);
}

void display()
{
    CUT_SAFE_CALL(cutStartTimer(timer));  

    // update the simulation
    if (!bPause)
    {
        nbodyCUDA->update(timestep);

        renderer->setPBO(nbodyCUDA->getCurrentReadBuffer(), nbodyCUDA->getNumBodies());
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
    if (displayEnabled)
    {
        // view transform
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        for (int c = 0; c < 3; ++c)
        {
            camera_trans_lag[c] += (camera_trans[c] - camera_trans_lag[c]) * inertia;
            camera_rot_lag[c] += (camera_rot[c] - camera_rot_lag[c]) * inertia;
        }
        glTranslatef(camera_trans_lag[0], camera_trans_lag[1], camera_trans_lag[2]);
        glRotatef(camera_rot_lag[0], 1.0, 0.0, 0.0);
        glRotatef(camera_rot_lag[1], 0.0, 1.0, 0.0);

        renderer->display(displayMode);
    }
   
    CUT_SAFE_CALL(cutStopTimer(timer));  

    // Display user interface.
	if (bShowSliders)
	{
        glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO); // invert color
        glEnable(GL_BLEND);
		paramlist->Render(0, 0);
        glDisable(GL_BLEND);
	}

    glutSwapBuffers();

    fpsCount++;
    // this displays the frame rate updated every second (independent of frame rate)
    if (fpsCount >= fpsLimit) {
        char fps[256];
        float milliseconds = cutGetAverageTimerValue(timer);
        double interactionsPerSecond = 0;
        double gflops = 0;
        computePerfStats(interactionsPerSecond, gflops, milliseconds, 1);

        float ifps = 1.f / (milliseconds / 1000.f);
        sprintf(fps, "CUDA N-Body (%d bodies): %0.1f fps | %0.1f IPS | %0.1f GFLOP/s", numBodies, ifps, interactionsPerSecond, gflops);  
        glutSetWindowTitle(fps);
        fpsCount = 0; 
        fpsLimit = (ifps > 1.f) ? (int)ifps : 1;
        if (bPause) fpsLimit = 0;
        CUT_SAFE_CALL(cutResetTimer(timer));  
    }


    glutReportErrors();
}

void reshape(int w, int h)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float) w / (float) h, 0.1, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glViewport(0, 0, w, h);
}

void mouse(int button, int state, int x, int y)
{
    if (bShowSliders) 
	{
		// call list mouse function
        if (paramlist->Mouse(x, y, button, state))
        {
            nbodyCUDA->setDamping(damping);
        }
	}
    
    int mods;

    if (state == GLUT_DOWN)
        buttonState |= 1<<button;
    else if (state == GLUT_UP)
        buttonState = 0;

    mods = glutGetModifiers();
    if (mods & GLUT_ACTIVE_SHIFT) {
        buttonState = 2;
    } else if (mods & GLUT_ACTIVE_CTRL) {
        buttonState = 3;
    }

    ox = x; oy = y;

    glutPostRedisplay();
}

void motion(int x, int y)
{

    if (bShowSliders) 
	{
		// call parameter list motion function
		if (paramlist->Motion(x, y))
        {
            nbodyCUDA->setDamping(damping);
            glutPostRedisplay();
		    return;
        }
	}

    float dx = x - ox;
    float dy = y - oy;

    if (buttonState == 3) {
        // left+middle = zoom
        camera_trans[2] += (dy / 100.0) * 0.5 * fabs(camera_trans[2]);
    } 
    else if (buttonState & 2) {
        // middle = translate
        camera_trans[0] += dx / 100.0;
        camera_trans[1] -= dy / 100.0;
    }
    else if (buttonState & 1) {
        // left = rotate
        camera_rot[0] += dy / 5.0;
        camera_rot[1] += dx / 5.0;
    }
    
    ox = x; oy = y;
    glutPostRedisplay();
}

// commented out to remove unused parameter warnings in Linux
void key(unsigned char key, int /*x*/, int /*y*/)
{
    switch (key) 
    {
    case ' ':
        bPause = !bPause;
        break;
    case '\033':
    case 'q':
        exit(0);
        break;
    case '`':
        bShowSliders = !bShowSliders;
        break;
    case 'p':
        displayMode = (ParticleRenderer::DisplayMode)
                      ((displayMode + 1) % ParticleRenderer::PARTICLE_NUM_MODES);
        break;
    case 'd':
        displayEnabled = !displayEnabled;
        break;
    case '1':
        reset(NBODY_CONFIG_SHELL);
        break;
    case '2':
        reset(NBODY_CONFIG_RANDOM);
        break;
    case '3':
        reset(NBODY_CONFIG_EXPAND);
        break;
    }

    glutPostRedisplay();
}

void special(int key, int x, int y)
{
	paramlist->Special(key, x, y);
	glutPostRedisplay();
}

void idle(void)
{
    glutPostRedisplay();
}


////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv) {
    numIterations = 0;
    int p = 256;
    int q = 1;

    cutGetCmdLineArgumenti( argc, (const char**) argv, "n", &numBodies);

    switch (numBodies) {
    case 1024:
        clusterScale = 1.52f;
        velocityScale = 2.f;
        break;
    case 2048:
        clusterScale = 1.56f;
        velocityScale = 2.64f;
        break;
    case 4096:
        clusterScale = 1.68f;
        velocityScale = 2.98f;
        break;
    case 8192:
        clusterScale = 1.98f;
        velocityScale = 2.9f;
        break;
    case 16384:
        clusterScale = 1.54f;
        velocityScale = 8.f;
        break;
    case 32768:
        clusterScale = 1.44f;
        velocityScale = 11.f;
        break;
    }


    bool benchmark = cutCheckCmdLineFlag(argc, (const char**) argv, "benchmark") != 0;
    bool tiles = cutCheckCmdLineFlag(argc, (const char**) argv, "tiles") != 0;
    bool cpu = cutCheckCmdLineFlag(argc, (const char**) argv, "cpu") != 0;

    cutGetCmdLineArgumenti( argc, (const char**) argv, "i", &numIterations);
    cutGetCmdLineArgumenti( argc, (const char**) argv, "p", &p);
    cutGetCmdLineArgumenti( argc, (const char**) argv, "q", &q);

    bool compareToCPU = cutCheckCmdLineFlag( argc, (const char**) argv, "compare") != 0;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(768, 768);
    glutCreateWindow("CUDA n-body system");

    initGL();
    init(numBodies, p, q, tiles);
    initParameters();
    
    reset(NBODY_CONFIG_SHELL);

    if (compareToCPU) {
        compareResults(numBodies);
        return 0;
    }

    if (benchmark) {
        if (numIterations <= 0) numIterations = 300;
        runBenchmark(numIterations, cpu);
    }
    else {
        glutDisplayFunc(display);
        glutReshapeFunc(reshape);
        glutMouseFunc(mouse);
        glutMotionFunc(motion);
        glutKeyboardFunc(key);
        glutSpecialFunc(special);
        glutIdleFunc(idle);

        glutMainLoop();
    }

    printf("\nFreeing memory\n");

    if (nbodyCPU) delete nbodyCPU;
    if (nbodyCUDA) delete nbodyCUDA;

    if (hPos) delete [] hPos;
    if (hVel) delete [] hVel;
    if (hColor) delete [] hColor;

    return 0;
}
