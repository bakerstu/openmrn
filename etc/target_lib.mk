# This directory contains copies of the platform libraries as built by the
# individual platform subdirectories. They get copied here by those
# subdirectories, thus there is no build rule here, just cleaning.
all:

clean::
	rm -rf *.a *.so *.dll timestamp

veryclean:: clean

tests:

mksubdirs:
