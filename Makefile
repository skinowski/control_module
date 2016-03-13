#
# Robot Control Module Makefile
#
CROSSTOOL_PATH=${HOME}/robot/crosstool-ng/x-tools7h/arm-unknown-linux-gnueabihf/bin
NAME=control_module.out

CPP=$(CROSSTOOL_PATH)/arm-unknown-linux-gnueabihf-g++
CPPFLAGS=-g -O2 -MMD -std=c++0x -fno-exceptions -fno-rtti -pthread
LDFLAGS=-lrt -pthread -static

MODULES := utils sensors protocol

SOURCES := $(wildcard *.cpp)

-include $(patsubst %, %/module.mk, $(MODULES))

OBJECTS := $(patsubst %.cpp, %.o, $(filter %.cpp,$(SOURCES)))

.PHONY: all
all : compile_all

%.o : %.cpp
	$(CPP) $(CPPFLAGS) $(INCLUDES) -c -o $@ $<

$(NAME) : $(OBJECTS)
	$(CPP) -o $@ $^ $(LDFLAGS)

.PHONY: compile_all
compile_all: $(NAME)

.PHONY: clean
clean :
	@rm -f $(OBJECTS) $(NAME)
	@rm -f $(patsubst %.o, %.d, $(filter %.o,$(OBJECTS)))

-include $(OBJECTS:.o=.d)
