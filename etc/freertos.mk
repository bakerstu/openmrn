FREERTOSPATH ?= $(shell \
sh -c "if [ \"X`printenv FREERTOSPATH`\" != \"X\" ]; then printenv FREERTOSPATH; \
     elif [ -d /opt/FreeRTOS/Source ]; then echo /opt/FreeRTOS; \
     elif [ -d ~/FreeRTOS/Source ]; then echo ~/FreeRTOS; \
     else echo ; fi" \
)

