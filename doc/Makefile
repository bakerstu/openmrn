SRCS = $(wildcard *.dox) $(wildcard *.md) \
       $(wildcard ../src/os/*.h) $(wildcard ../src/os/*.cxx) \
       $(wildcard ../src/os/*.c) $(wildcard ../src/os/*.hxx) \
       $(wildcard ../src/if/*.h) $(wildcard ../src/if/*.cxx) \
       $(wildcard ../src/if/*.c) $(wildcard ../src/if/*.hxx) \
       $(wildcard ../src/core/*.h) $(wildcard ../src/core/*.cxx) \
       $(wildcard ../src/core/*.c) $(wildcard ../src/core/*.hxx) \
       $(wildcard ../src/freertos_drivers/common/*.h) \
       $(wildcard ../src/freertos_drivers/common/*.c) \
       $(wildcard ../src/freertos_drivers/common/*.hxx) \
       $(wildcard ../src/freertos_drivers/common/*.cxx) \
       $(wildcard ../src/freertos_drivers/ti/*.h) \
       $(wildcard ../src/freertos_drivers/ti/*.c) \
       $(wildcard ../src/freertos_drivers/ti/*.hxx) \
       $(wildcard ../src/freertos_drivers/ti/*.cxx) \
       $(wildcard ../applications/*/*.md) \
       Doxyfile

all:

.PHONY:
docs: html

html: $(SRCS)
	doxygen Doxyfile
	echo "warnings:"
	cat warnings

clean::
	rm -rf latex html warnings openmrn.html

veryclean:: clean

tests:

mksubdirs:
