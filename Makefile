ifeq ($(DEBUG_MODE),1)
include Makefile.debug
else
ifeq ($(RELEASE_MODE),1)
include Makefile.release
else
include Makefile.tos
endif
endif
