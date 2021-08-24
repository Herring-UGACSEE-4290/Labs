#----------------------------------------------------------------------------
#The information contained in this file may only be used by a person
#authorised under and to the extent permitted by a subsisting licensing 
#agreement from Arm Limited or its affiliates 
#
#(C) COPYRIGHT 2020 Arm Limited or its affiliates
#ALL RIGHTS RESERVED.
#Licensed under the ARM EDUCATION INTRODUCTION TO COMPUTER ARCHITECTURE 
#EDUCATION KIT END USER LICENSE AGREEMENT.
#See https://www.arm.com/-/media/Files/pdf/education/computer-architecture-education-kit-eula
#
#This entire notice must be reproduced on all copies of this file
#and copies of this file may only be made by a person if such person is
#permitted to do so under the terms of a subsisting license agreement
#from Arm Limited or its affiliates.
#----------------------------------------------------------------------------
################################################################################
#   Verilog makefile
################################################################################
VC      = iverilog.exe
VCFLAGS = -Wall -Wno-timescale -Wno-implicit-dimensions -I$(HEAD) -t vvp \
          -y$(SRC) -D SIMULATION

VI      = vvp.exe
VI_OUT  = lx2
VIFLAGS = -s $(addprefix -m,$(VPI_TGT))
VI_ARGS = $(ARGS)
DUMP_FILE ?= $@

VPI     = iverilog-vpi.exe 
VPIFLAGS= --name=$(basename $@) 

CC          = gcc.exe
VPI_CFLAGS := $(shell iverilog-vpi.exe -ivl=c:\\iverilog -mingw=c:\\mingw --cflags)
#VPI_CFLAGS := $(shell iverilog-vpi.exe -ivl=/mnt/c/iverilog)
#VPI_CFLAGS :=  -Wall -Wshadow -g -O2 -I"c:\iverilog\\include\\iverilog"
CFLAGS      = $(VPI_CFLAGS) -I$(VPIHEAD) -c -Wno-strict-prototypes -o $@

################################################################################

SRC     := ./src
VVP     := ./vvp
TESTS   := ./tests
SIMDMP  := ./sim_dump
HEAD    := ./head
VPISRC  := ./vpi_src
VPIOBJ  := ./vpi_obj
VPIBIN  := ./vpi_bin
VPIHEAD	:= ./vpi_head

get_file    =$(basename $(notdir $(1)))
make_target =$(addprefix $(1)/,$(call get_file,$(wildcard $(2)/*.$(3))))
get_module  =$(call get_file,$(1))

VPI_TGT := $(addsuffix .vpi,$(call make_target,$(VPIBIN),$(VPISRC),c))
VPI_OBJ := $(addsuffix .o,  $(call make_target,$(VPIOBJ),$(VPISRC),c))
VVP_TGT := $(addsuffix .vvp,$(call make_target,$(VVP),$(TESTS),v))
SIM_TGT := $(addsuffix .$(VI_OUT),$(call make_target,$(SIMDMP),$(TESTS),v))
HEADERS := $(wildcard $(HEAD)/*.vh)
SOURCES := $(wildcard $(SRC)/*.v)
VPI_HEAD:= $(wildcard $(VPIHEAD)/*.h)

export PATH:= bin:$(PATH)
################################################################################

.PHONY: clean software

.SECONDARY:

all: $(SIM_TGT)

$(VVP):
	mkdir $(VVP)/

$(VPIBIN):
	mkdir $(VPIBIN)/

$(VPIOBJ):
	mkdir $(VPIOBJ)/

$(SIMDMP):
	mkdir $(SIMDMP)/

software:
	$(MAKE) -C sw || (exit 1)

%.sim: $(VVP)/%.vvp $(VPI_TGT) | $(SIMDMP)
	-$(VI) $(VIFLAGS) $< -$(VI_OUT) $(VI_ARGS)
	-@mv dump.$(VI_OUT) $(SIMDMP)/$(DUMP_FILE).$(VI_OUT)

$(VVP)/%.vvp: $(TESTS)/%.v $(SOURCES) $(HEADERS) | $(VVP)
	$(VC) $(VCFLAGS) -s $(call get_module,$<) -o $@ $<

$(VPIBIN)/%.vpi: $(VPIOBJ)/%.o | $(VPIBIN)
	$(VPI) $(VPIFLAGS) $<

$(VPIOBJ)/%.o: $(VPISRC)/%.c $(VPI_HEAD) | $(VPIOBJ)
	$(CC) $(CFLAGS) $<

clean:
	@echo $(VPI_TGT) $(VPI_OBJ) $(VVP_TGT) $(SIM_TGT)
	#@rm -fv $(VPI_TGT) $(VPI_OBJ) $(VVP_TGT) $(SIM_TGT)
	#@del $(VPI_TGT) $(VPI_OBJ) $(VVP_TGT) $(SIM_TGT)
