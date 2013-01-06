CFLAGSENV ?= $(shell \
sh -c "if [ \"X`printenv CFLAGSENV`\" != \"X\" ]; then printenv CFLAGSENV; \
     else echo ; fi" \
)

CXXFLAGSENV ?= $(shell \
sh -c "if [ \"X`printenv CXXFLAGSENV`\" != \"X\" ]; then printenv CXXFLAGSENV; \
     else echo ; fi" \
)

LDFLAGSENV ?= $(shell \
sh -c "if [ \"X`printenv LDFLAGSENV`\" != \"X\" ]; then printenv LDFLAGSENV; \
     else echo ; fi" \
)

