# Build the platform libraries.
LIBRARIES = os if core lib freertos freertos_drivers tinyxml2 cJSON

all:
	$(foreach lib, $(LIBRARIES), $(MAKE) -C $(lib) all || exit 1;)

clean::
	$(foreach lib, $(LIBRARIES), $(MAKE) -C $(lib) clean || exit 1;)

veryclean:: clean
	$(foreach lib, $(LIBRARIES), $(MAKE) -C $(lib) veryclean || exit 1;)

