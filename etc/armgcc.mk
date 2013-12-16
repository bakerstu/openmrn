TOOLPATH ?= $(shell \
sh -c "if [ \"X`printenv TOOLPATH`\" != \"X\" ]; then printenv TOOLPATH; \
     elif [ -d /usr/local/lpcxpresso_*/lpcxpresso/tools/bin ]; then ls -d /usr/local/lpcxpresso_*/lpcxpresso/tools | tail -n 1; \
     elif [ -d /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin ]; then echo /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI; \
     elif [ -d /opt/armgcc/default/bin ]; then echo /opt/armgcc/default; \
     else echo TOOLPATH not found; fi" \
)

ifneq ($(TOOLPATH),TOOLPATH not found)
HAVE_ARMGCC = 1
else
TOOLPATH=
endif
