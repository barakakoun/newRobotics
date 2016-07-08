################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ConfigurationManager.cpp \
../LocalizationManager.cpp \
../Map.cpp \
../MathUtils.cpp \
../Particle.cpp \
../Robot.cpp \
../WaypointManager.cpp \
../lodepng.cpp \
../main.cpp 

OBJS += \
./ConfigurationManager.o \
./LocalizationManager.o \
./Map.o \
./MathUtils.o \
./Particle.o \
./Robot.o \
./WaypointManager.o \
./lodepng.o \
./main.o 

CPP_DEPS += \
./ConfigurationManager.d \
./LocalizationManager.d \
./Map.d \
./MathUtils.d \
./Particle.d \
./Robot.d \
./WaypointManager.d \
./lodepng.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/player-2.0 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


