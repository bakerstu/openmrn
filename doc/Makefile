
SRCS = $(wildcard ../src/os/*.h) $(wildcard ../src/os/*.cxx) \
       $(wildcard ../src/os/*.c) $(wildcard ../src/os/*.hxx) \
       $(wildcard ../src/if/*.h) $(wildcard ../src/if/*.cxx) \
       $(wildcard ../src/if/*.c) $(wildcard ../src/if/*.hxx) \
       $(wildcard ../src/core/*.h) $(wildcard ../src/core/*.cxx) \
       $(wildcard ../src/core/*.c) $(wildcard ../src/core/*.hxx)

all:

.PHONY:
docs: html

html: $(SRCS)
	doxygen Doxyfile

clean::
	rm -rf latex html warnings

veryclean:: clean
	

