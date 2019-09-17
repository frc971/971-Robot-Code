c_sources :=	jsont.c

all: example1 example2 test

object_dir = .objs
objects = $(patsubst %,$(object_dir)/%,${c_sources:.c=.o})
object_dirs = $(sort $(foreach fn,$(objects),$(dir $(fn))))
-include ${objects:.o=.d}

test_dir = test
test_sources  := $(wildcard test/test*.c)
test_object_dir = $(test_dir)/.objs
test_build_dir  = $(test_dir)/build
test_objects    = $(patsubst test/%,$(test_object_dir)/%,${test_sources:.c=.o})
test_programs   = $(patsubst test/%.c,$(test_build_dir)/%,$(test_sources))
test_object_dirs = $(sort $(foreach fn,$(test_objects),$(dir $(fn))))

CC = clang
LD = clang

CFLAGS 	+= -Wall -g -MMD -std=c99 -I.
TEST_CFLAGS := $(CFLAGS) -O0
#LDFLAGS +=
ifneq ($(DEBUG),)
	CFLAGS += -O0 -DDEBUG=1
else
	CFLAGS += -O3 -DNDEBUG
endif

clean:
	rm -f jsont example1 example2
	rm -rf $(object_dir)
	rm -rf $(test_object_dir)
	rm -rf $(test_build_dir)

example1: $(objects) $(object_dir)/example1.o
	$(LD) $(LDFLAGS) -o $@ $^

example2: $(objects) $(object_dir)/example2.o
	$(LD) $(LDFLAGS) -o $@ $^

test: $(objects) $(test_programs)
	$(test_programs)

$(test_build_dir)/%: $(objects) $(test_object_dir)/%.o
	@mkdir -p `dirname $@`
	$(LD) $(LDFLAGS) -o $@ $^

$(test_object_dir)/%.o: $(test_dir)/%.c
	@mkdir -p `dirname $@`
	$(CC) $(TEST_CFLAGS) -c -o $@ $<

$(object_dir)/%.o: %.c
	@mkdir -p `dirname $@`
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: clean all test
