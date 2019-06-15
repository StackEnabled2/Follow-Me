TARGETS=mpsControl
SRCDIR=src
OBJDIR=obj

$(info "teest a message")

# Compiler setup
CXX=g++
CPPFLAGS=-Isrc
CXXFLAGS=-std=gnu++14 -pedantic -Wall -Wextra
LIBS=-pthread

# Programs
RM=rm -f

$("info teest a message")

# Rules
.PHONY: all debug clean rebuild

all: CPPFLAGS+=-DNDEBUG
all: CXXFLAGS+=-O3
all: $(TARGETS)

debug: CXXFLAGS+=-g
debug: $(TARGETS)

$("info teest a message ")
$(info $(OBJDIR))

mpsControl: $(SRCDIR)/mpsControl.o $(SRCDIR)/util.o
#mpsControl: $(OBJDIR)/mpsControl.o $(OBJDIR)/util.o
	$(CXX) -o $@ $^ $(LIBS)

$(info $(OBJDIR))


%.o: $(SRCDIR)%.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $(OBJDIR) $<
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(info "clean")

	$(RM) $(SRCDIR)/*.o
	$(RM) bin/*.*
	$(RM) bin/Debug/*.*
	$(RM) mpsControl

rebuild: clean all

cleanDebug:

$(info "cleanDebug")

Debug:
$(info "Debug")
