# -*- Makefile -*-
#
# $Id$

MODE=Debug

# start "make MODE=Release" for release mode
_gccRes := $(shell gcc --version)

GCCVERSION := $(shell gcc -dumpversion)
ARCH := $(shell arch)
UNAME := $(shell uname)
OBJDIR := $(UNAME)-$(ARCH)-gcc$(GCCVERSION)

CC = gcc

GPSTKDIR  = /Users/chm/insted
CXXFLAGS  += -I/opt/local/include 
LDFLAGS   += -L/opt/local/lib -lboost_system -lboost_thread -lboost_filesystem -lboost_date_time -lperseus-sdr -lfftw3 -lfftw3f -lm

#LOADLIBES += -L${GPSTKDIR}/lib -lgpstk
#LOADLIBES += -L${OCTAVEDIR}/lib/octave-3.0.3 -L${OCTAVEDIR}/lib -Wl,-rpath -Wl,${OCTAVEDIR}/lib/octave-3.0.3 -loctinterp -l#octave -lcruft 
#LOADLIBES += -L${OCTAVEDIR}/lib -lgpstk -lblas -llapack  
#LOADLIBES += -lm


# rules
O=$(OBJDIR)/$(MODE)

$(OBJDIR)/Debug/%.o: CXXFLAGS +=  -pipe -Wall -g 
$(OBJDIR)/Release/%.o: CXXFLAGS += -pipe -O2 -ffloat-store 
$(OBJDIR)/Profile/%.o: CXXFLAGS += -pipe -g -pg
$(OBJDIR)/Coverage/%.o: CXXFLAGS += -pipe -fprofile-arcs -ftest-coverage
$(OBJDIR)/RelProf/%.o: CXXFLAGS += -pipe -O2 -g -pg

# Make automatic (include) dependency files
$O/%.d: CXXFLAGS += -MM -MT 'syn/$*.syn' -MT '$$O/$*.o' -MT '$$O/$*.d'

$O/%.o: %.c
	$(COMPILE.cc) $< $(OUTPUT_OPTION)
$O/%.o: %.C
	$(COMPILE.cc) $< $(OUTPUT_OPTION)
$O/%.o: %.cpp
	$(COMPILE.cc) $< $(OUTPUT_OPTION)
$O/%.d: %.c
	$(COMPILE.cc) $< >$@
$O/%.d: %.C
	$(COMPILE.cc) $< >$@
$O/%.d: %.cpp
	$(COMPILE.cc) $< >$@

# Make automatic "link" files (containing link module dependencies)
#                                           this pattern must by in a header file to find the implementation (basename at 5 word)
#                                           v
LINK_GEN = $(CC) -E $(CXXFLAGS) $< | awk '/.*___ Implemented in .*/ { v[$$5]=1 }; \
END { printf "$$(OBJDIR)/Profile/$(@:$O/%.link=%): CXXFLAGS += -g -pg\n$$(OBJDIR)/RelProf/$(@:$O/%.link=%): CXXFLAGS += -g -pg\n$$O/$(@:$O/%.link=%):$$O/$(@:$O/%.link=%_main.o) " ;\
      for(i in v) { printf "$$O/%s.o ",i } ; printf "\n\t$$(LINK.cc) $$^ $$(LOADLIBES) $$(LDLIBS) -o $$@\n"; \
      for(i in v) { printf "OBJS += $O/%s.o\n",i }}' - >$@
$O/%.link: %_main.c
	$(LINK_GEN)
$O/%.link: %_main.C
	$(LINK_GEN)
$O/%.link: %_main.cpp
	$(LINK_GEN)

dummy := $(shell mkdir -p $O)
CPP_MAINS =$(wildcard *_main.cpp)
CC_MAINS =$(wildcard *_main.C)
C_MAINS =$(wildcard *_main.c)
TARGETS = $(CPP_MAINS:%_main.cpp=$O/%) $(CC_MAINS:%_main.C=$O/%) $(C_MAINS:%_main.c=$O/%)

OBJS += $(TARGETS:%=%_main.o)

all: $(TARGETS)

doc: syn/html

include $(TARGETS:%=%.link)

clean:
	$(RM) $(TARGETS) $(OBJS)

include $(OBJS:%.o=%.d)
