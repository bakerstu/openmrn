ifndef OPENMRNPATH
export OPENMRNPATH=$(realpath $(dir $(lastword $(MAKEFILE_LIST)))/..)
endif
