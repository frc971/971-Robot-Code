SRCS = get.cc libusb_wrap.cc signal_handler.cc thread.cc
CFLAGS  = -I./ -I/usr/include/libusb-1.0 -g -std=gnu++0x
LDFLAGS = -g -lgflags -lglog -lusb-1.0 -lboost_thread-mt -lrt

get: $(SRCS:.cc=.o)
	g++ $^ -o $@ $(LDFLAGS)

%.o: %.cc
	g++ -MMD -MP -c $(CFLAGS) $< -o $@

clean:
	$(RM) $(SRCS:.cc=.o) $(SRCS:.cc=.d) get

.PHONY: clean
-include $(SRCS:.cc=.d)
