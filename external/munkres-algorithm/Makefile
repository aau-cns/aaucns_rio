
TEST_SRCS:=example.cpp

CC=gcc
CPP_FLAGS:=-std=c++14 -I$(CURDIR) -Wall -Wextra -pedantic -Werror -fsanitize=address
LINK_FLAGS:=-fsanitize=address -lm -lstdc++

OBJDIR:=build
OBJFILES:=$(patsubst %.cpp,${OBJDIR}/%.o,${TEST_SRCS})

.PHONY: clean run_tests

example: $(OBJDIR)/example.o
	$(CC) $(CPP_FLAGS) $(OBJDIR)/example.o $(LINK_FLAGS) -o example

test: $(OBJDIR)/test.o
	$(CC) $(CPP_FLAGS) $(OBJDIR)/test.o $(LINK_FLAGS) -o test

run_tests: test
	./test

$(OBJDIR)/%.o: %.cpp
	@mkdir -p "$$(dirname "$@")"
	$(CC) -x c++ $(CPP_FLAGS) -o $@ -c $<

clean:
	rm -rf build
	rm -f test
	rm -f example

