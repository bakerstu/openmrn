TOOLPATH ?= $(shell \
sh -c "if [ \"X`printenv TOOLPATH`\" != \"X\" ]; then printenv TOOLPATH; \
     elif [ -d /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin ]; then echo /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI; \
     else echo TOOLPATH not found; fi" \
)

