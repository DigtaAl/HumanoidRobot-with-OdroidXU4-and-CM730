##################################################
# PROJECT: Darwin - University of Yogyakarta
##################################################

#---------------------------------------------------------------------
# C++ COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
DIR_DARWIN	= .
DIR_OBJS	= build
DIR_QT	= /usr/include/qt4

INSTALL_ROOT = /usr/local

MAJ_VERSION = 1
MIN_VERSION = 0
REV_VERSION = 0

TARGET	= libdarwin.so
TARGET1	= $(TARGET).$(MAJ_VERSION)
TARGET2	= $(TARGET).$(MAJ_VERSION).$(MIN_VERSION)
TARGET3	= $(TARGET).$(MAJ_VERSION).$(MIN_VERSION).$(REV_VERSION)

CHK_DIR_EXISTS = test -d
PRINT	= echo
STRIP	= strip
AR	= ar
ARFLAGS	= cr
LD	= g++
LDFLAGS	= -shared -fPIC #-Wl,-soname,darwin
LD_CONFIG	= ldconfig
CP	= cp
CP_ALL	= cp -r
RM	= rm
RM_ALL	= rm -rf
SYMLINK	= ln -s
MKDIR	= mkdir
CC	= gcc
CX	= g++
CCFLAGS	= -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) -fPIC -g
CXFLAGS	= -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) -fPIC -g
# CXFLAGS	+= -DDEBUG
INCLUDES	+= -I$(DIR_QT)/QtCore -I$(DIR_QT)/QtGui -I$(DIR_QT)/QtOpenGL -I$(DIR_QT)/QtXml -I$(DIR_QT)
INCLUDES	+= -I$(DIR_DARWIN)/include -I$(DIR_DARWIN)/include/darwin/framework -I$(DIR_DARWIN)/include/darwin/linux

#---------------------------------------------------------------------
# Required external libraries
#---------------------------------------------------------------------
#LIBS	+= -Lespeak
LIBS	+= -Lcv
LIBS	+= -Lhighgui
LIBS	+= -Lmlibrary
LIBS	+= -Lvideo
#---------------------------------------------------------------------
# Framework Files
#---------------------------------------------------------------------
SOURCES	= src/framework/CM730.cpp \
	src/framework/math/Matrix.cpp \
	src/framework/math/Plane.cpp \
	src/framework/math/Point.cpp \
	src/framework/math/Vector.cpp \
	src/framework/minIni/minIni.c \
	src/framework/motion/JointData.cpp \
	src/framework/motion/Kinematics.cpp \
	src/framework/motion/MotionManager.cpp \
	src/framework/motion/MotionStatus.cpp \
	src/framework/motion/MX28.cpp \
	src/framework/motion/modules/Action.cpp \
	src/framework/motion/modules/Head.cpp \
	src/framework/motion/modules/Walking.cpp \
	src/framework/vision/BallFollower.cpp \
	#src/framework/vision/BallTracker.cpp \
	#src/framework/vision/ColorFinder.cpp \
	#src/framework/vision/Image.cpp \
	#src/framework/vision/ImgProcess.cpp \
#src/framework/vision/Camera.cpp \
#---------------------------------------------------------------------
# Linux Files
#---------------------------------------------------------------------
SOURCES += src/linux/LinuxActionScript.cpp \
	src/linux/LinuxCM730.cpp \
	#src/linux/LinuxCamera.cpp \
	src/linux/LinuxMotionTimer.cpp \
	src/linux/LinuxNetwork.cpp \
	#src/linux/streamer/httpd.cpp \
	#src/linux/streamer/jpeg_utils.cpp \
	#src/linux/streamer/mjpg_streamer.cpp \

OBJECTS=$(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))

#---------------------------------------------------------------------
# COMPILING RULES
#---------------------------------------------------------------------
$(TARGET): makedirs $(OBJECTS)
	#$(AR) $(ARFLAGS) $(DIR_DARWIN)/lib/$(TARGET) $(OBJECTS)
	$(LD) $(LDFLAGS) -o $(DIR_DARWIN)/lib/$(TARGET) $(OBJECTS) $(LIBS)
	
makedirs:
	mkdir -p build
	mkdir -p lib
clean:
	rm -f $(OBJECTS) $(DIR_DARWIN)/lib/$(TARGET)

install:$(TARGET)	
# copy the binaries into the lib directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/lib || $(MKDIR) $(INSTALL_ROOT)/lib 
	-$(CP) "$(DIR_DARWIN)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET)" 
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET1)" 
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET2)" 
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET3)"
	
# copy the headers into the include directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include || $(MKDIR) $(INSTALL_ROOT)/include
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include/darwin || $(MKDIR) $(INSTALL_ROOT)/include/darwin
	$(CP_ALL) $(DIR_DARWIN)/include/* $(INSTALL_ROOT)/include/
	#chmod 755 $(TARGET)							
	$(LD_CONFIG)
	
uninstall:
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET1)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET2)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET3)
	chmod 755 $(TARGET)
	$(RM_ALL) $(INSTALL_ROOT)/include/darwin
	$(RM) $(INSTALL_ROOT)/include/DARwIn.h
	$(RM) $(INSTALL_ROOT)/include/LinuxDARwIn.h

reinstall: uninstall install


#---------------------------------------------------------------------
# Make rules for all .c and .cpp files in each directory
#---------------------------------------------------------------------
$(DIR_OBJS)/%.o: src/framework/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/math/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/math/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/minIni/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/minIni/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/motion/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/motion/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/motion/modules/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/motion/modules/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/vision/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/vision/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/linux/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/linux/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/linux/streamer/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/linux/streamer/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# END OF MAKEFILE
#---------------------------------------------------------------------
