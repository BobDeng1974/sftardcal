# GNU make file for sftardcal - GNU make will use this one
# Copyright 2015 by S.F.T. Inc. - all rights reserved
# For licensing information, see 'sftardcal.c'

# usage:  make TESTER=1        builds tester version
#         make POWERSUPPLY=1   builds power supply version
#         make clean           cleans


CC?=gcc
DEBUG=0
POWERSUPPLY=0
TESTER=0
DUALSERIAL=0
#CFLAGS=
DEVICE_SPECIFIC_OBJ=
MY_TARGET=sftardcal

THE_CURRENT_TIME:=$(shell date -u '+%C%y%m%d%H%M%S')
STANDARD_DEFINES:= $(CFLAGS) -DBUILD_DATE_TIME=$(THE_CURRENT_TIME)

ifneq ($(POWERSUPPLY),0)
  DEVICE_SPECIFIC_C = PowerSupply.c
  DEVICE_SPECIFIC_H = PowerSupply.h
  DEVICE_SPECIFIC_DEP =
#  DEVICE_SPECIFIC_OBJ = PowerSupply.o
  DEVICE_DEFINES = -DPOWERSUPPLY
MY_TARGET=sftardcalPS
else
  ifneq ($(TESTER),0)
    DEVICE_SPECIFIC_C = Tester.c
    DEVICE_SPECIFIC_H = Tester.h
    DEVICE_SPECIFIC_DEP =
#    DEVICE_SPECIFIC_OBJ =
    DEVICE_DEFINES= -DTESTER
    MY_TARGET=sftardcalT
  else
    ifneq ($(DUALSERIAL),0)
      DEVICE_SPECIFIC_C = DualSerial.c
      DEVICE_SPECIFIC_H =
      DEVICE_SPECIFIC_DEP =
#      DEVICE_DEFINES = -DDEBUG=1
      DEVICE_DEFINES = -DDUALSERIAL
      MY_TARGET=DualSerial
    else
      DEVICE_SPECIFIC_C =
      DEVICE_SPECIFIC_H =
      DEVICE_SPECIFIC_DEP = xmodem.c xmodem.h
      DEVICE_DEFINES = -DSTAND_ALONE -DWITH_XMODEM
      MY_TARGET=sftardcal
    endif
  endif
endif

ifneq (DEBUG,0)
  DEVICE_DEFINES += -g
else
  DEVICE_DEFINES += -Wall
endif


DEVICE_SPECIFIC := $(DEVICE_SPECIFIC_C) $(DEVICE_SPECIFIC_H) $(DEVICE_SPECIFIC_DEP)

all:: message $(MY_TARGET)


message:
	@echo "Using Makefile for GNU-compatible 'make'"


# all of the targets are in Makefile.incl

include Makefile.incl

