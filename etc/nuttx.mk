ifndef NUTTXPATH
NUTTXPATH := $(shell \
sh -c "if [ \"X`printenv NUTTXPATH`\" != \"X\" ]; then printenv NUTTXPATH; \
     elif [ -d /opt/nuttx/include ]; then echo /opt/nuttx; \
     elif [ -d ~/nuttx/include ]; then echo ~/nuttx; \
     else echo ; fi" \
)
endif
