obj-y 	+= IRsensor_Hardware.o
obj-y	+= cm36686/
obj-n	+= ap3425/
obj-y	+= ap3045/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

