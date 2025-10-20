includes = \
	$(WEBOTS_HOME_PATH)/resources/projects/default/plugins/physics/Makefile.include \
	$(WEBOTS_HOME_PATH)/resources/physics/plugins/Makefile.include \
	$(WEBOTS_HOME_PATH)/resources/plugins/physics/Makefile.include \
	$(WEBOTS_HOME_PATH)/projects/default/physics/plugins/Makefile.include

first = $(word 1,$(wildcard $(includes)))

ifneq ($(first),"")
	include $(first)
endif
