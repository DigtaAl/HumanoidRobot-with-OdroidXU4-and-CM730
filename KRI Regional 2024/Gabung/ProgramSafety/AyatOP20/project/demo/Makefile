# *** ENTER THE TARGET NAME HERE ***
TARGET      = demo

# important directories used by assorted rules and other variables
DIR_DARWIN  = /usr/local
DIR_OBJS    = .objects
#DIR_QT      = /usr/include/qt4

# compiler options
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -g -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES)
CXFLAGS     = -O2 -O3 -g -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES)
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_DARWIN)/lib

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_DARWIN)/include/darwin/framework 
INCLUDES   += -I$(DIR_DARWIN)/include/darwin/linux 
#INCLUDES   += -I$(DIR_DARWIN)/include/motionlibrary

LIBRARIES  += -ldarwin 
LIBRARIES  += -lpthread 
LIBRARIES  += -ldl 
#LIBRARIES  += -lespeak 
LIBRARIES  += -lopencv_core
LIBRARIES  += -lopencv_highgui
LIBRARIES  += -lopencv_imgproc
LIBRARIES  += -lopencv_videoio
#LIBRARIES  += -lmlibrary

#---------------------------------------------------------------------
# Qt components (not always needed)
#---------------------------------------------------------------------
#INCLUDES   += -I$(DIR_QT)/QtCore 
#INCLUDES   += -I$(DIR_QT)/QtGui 
#INCLUDES   += -I$(DIR_QT)/QtXml 
#INCLUDES   += -I$(DIR_QT)

#LIBRARIES  += -lQtXml 
#LIBRARIES  += -lQtGui 
#LIBRARIES  += -lQtCore



#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = main.cpp \
		StatusCheck.cpp \
		LinuxMotionTimer.cpp \
		.o\		

	# *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))
#OBJECTS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***


#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) $(LIBRARIES) -o $(TARGET)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: %.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: %.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/DARwIn_demo_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *

archive: tgz

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------

