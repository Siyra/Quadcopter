#////////////////////////////////////////////////////////////////////////////
#//
#//  This file is part of RTIMULib
#//
#//  Copyright (c) 2014, richards-tech
#//
#//  Permission is hereby granted, free of charge, to any person obtaining a copy of
#//  this software and associated documentation files (the "Software"), to deal in
#//  the Software without restriction, including without limitation the rights to use,
#//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
#//  Software, and to permit persons to whom the Software is furnished to do so,
#//  subject to the following conditions:
#//
#//  The above copyright notice and this permission notice shall be included in all
#//  copies or substantial portions of the Software.
#//
#//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
#//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Compiler, tools and options

RTIMULIBPATH  = RTIMULib
MAVLINKLIBPATH = MAVLink
ARMADILLOPATH = armadillo

CC    			= gcc
CXX   			= g++
DEFINES       	=
CFLAGS			= -pipe -O2 -Wall -W $(DEFINES) -lrt -DARMA_DONT_USE_WRAPPER -lblas -llapack
CXXFLAGS      	= -pipe -O2 -Wall -W $(DEFINES) -lrt -DARMA_DONT_USE_WRAPPER -lblas -llapack
#INCPATH       	= -I. -I$(RTIMULIBPATH) -I$(MAVLINKLIBPATH) -I$(ARMADILLOPATH)
INCPATH       	= -I. -I$(RTIMULIBPATH) -I$(MAVLINKLIBPATH)
LINK  			= g++
LFLAGS			= -Wl,-O1 -lrt -DARMA_DONT_USE_WRAPPER -lblas -llapack
LIBS  			= -L/usr/lib/arm-linux-gnueabihf
COPY  			= cp -f
COPY_FILE     	= $(COPY)
COPY_DIR      	= $(COPY) -r
STRIP 			= strip
INSTALL_FILE  	= install -m 644 -p
INSTALL_DIR   	= $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      	= rm -f
SYMLINK       	= ln -f -s
DEL_DIR       	= rmdir
MOVE  			= mv -f
CHK_DIR_EXISTS	= test -d
MKDIR			= mkdir -p

# Output directory

OBJECTS_DIR   = objects/

# Files

DEPS    = $(RTIMULIBPATH)/RTMath.h \
    $(RTIMULIBPATH)/RTIMULib.h \
    $(RTIMULIBPATH)/RTIMULibDefs.h \
    $(RTIMULIBPATH)/RTIMUHal.h \
    $(RTIMULIBPATH)/RTFusion.h \
    $(RTIMULIBPATH)/RTFusionKalman4.h \
    $(RTIMULIBPATH)/RTFusionRTQF.h \
    $(RTIMULIBPATH)/RTIMUSettings.h \
    $(RTIMULIBPATH)/RTIMUAccelCal.h \
    $(RTIMULIBPATH)/RTIMUMagCal.h \
    $(RTIMULIBPATH)/RTIMUCalDefs.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMU.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUNull.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUMPU9150.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUMPU9250.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20HM303D.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20M303DLHC.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUGD20HM303DLHC.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMULSM9DS0.h \
    $(RTIMULIBPATH)/IMUDrivers/RTIMUBMX055.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressure.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureBMP180.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureLPS25H.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureMS5611.h \
    $(RTIMULIBPATH)/IMUDrivers/RTPressureMS5637.h 

OBJECTS = objects/quadcopter.o \
	objects/esc.o \
	objects/pid.o \
	objects/comms.o \
    objects/RTMath.o \
    objects/RTIMUHal.o \
    objects/RTFusion.o \
    objects/RTFusionKalman4.o \
    objects/RTFusionRTQF.o \
    objects/RTIMUSettings.o \
    objects/RTIMUAccelCal.o \
    objects/RTIMUMagCal.o \
    objects/RTIMU.o \
    objects/RTIMUNull.o \
    objects/RTIMUMPU9150.o \
    objects/RTIMUMPU9250.o \
    objects/RTIMUGD20HM303D.o \
    objects/RTIMUGD20M303DLHC.o \
    objects/RTIMUGD20HM303DLHC.o \
    objects/RTIMULSM9DS0.o \
    objects/RTIMUBMX055.o \
    objects/RTPressure.o \
    objects/RTPressureBMP180.o \
    objects/RTPressureLPS25H.o \
    objects/RTPressureMS5611.o \
    objects/RTPressureMS5637.o 

MAKE_TARGET	= quadcopter
DESTDIR		= Output/
TARGET		= Output/$(MAKE_TARGET)

# Build rules

$(TARGET): $(OBJECTS)
	@$(CHK_DIR_EXISTS) Output/ || $(MKDIR) Output/
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core

# Compile

$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/%.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)
	
$(OBJECTS_DIR)%.o : $(RTIMULIBPATH)/IMUDrivers/%.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

$(OBJECTS_DIR)%.o : %.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)
