s° = $(subst $(empty) ,°,$1)
°s = $(subst °, ,$1)
abspathx = $(call °s,$(abspath $(call s°,$1)))

ROOT = $(call abspathx,$(CURDIR)/$(RELATIVE_ROOT))

export LD_LIBRARY_PATH := $(LD_LIBRARY_PATH):$(ROOT)/firmware/libs

