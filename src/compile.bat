@echo off
nvcc -O3 -o out/nbody -m32 -Iinc -Llib -lglut32 -lparamgl32 -lcutil -lglew32 cpu/bodysystemcpu.cpp cuda/bodysystemcuda.cpp cuda/bodysystemcuda.cu render/render_particles.cpp nbody.cpp
