# List of the ChibiOS generic STM32F4xx startup and CMSIS files.
STARTUPSRC = driver/ports/ARMCMx/compilers/GCC/crt1.c \
             driver/ports/ARMCMx/compilers/GCC/vectors.c
          
STARTUPASM = driver/ports/ARMCMx/compilers/GCC/crt0_v7m.s

STARTUPINC = driver/ports/ARMCMx/devices/STM32F4xx

STARTUPLD  = driver/ports/ARMCMx/compilers/GCC/ld
