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
#   Baremetal software makefile
################################################################################
CC        = aarch64-none-elf-gcc.exe
CFLAGS    =  -nostdlib -nodefaultlibs -gdwarf-4 -Wa,-march=armv8-a -Wl,-Ttext=0x0 -Wl,-N -I $(HEAD)
            

ELFCC     = aarch64-none-elf-objcopy.exe
ELFFLAGS  = -O verilog

#CC        = armclang
#CFLAGS    = -g --target=aarch64-arm-none-eabi -I$(HEAD) \
            -Xlinker --ro_base=0x0

#ELFCC     = fromelf
#ELFFLAGS  = --vhx


################################################################################

HEAD := ./head
SRC  := ./src
ELF  := ./elf
VHX  := ./mem


get_file    =$(basename $(notdir $(1)))
make_target =$(addprefix $(1)/,$(call get_file,$(wildcard $(2)/*.$(3))))

SRC_SUFFIX := S
ELF_SUFFIX := elf
VHX_SUFFIX := mem

ELF_OBJ := $(addsuffix .$(ELF_SUFFIX),$(call make_target,$(ELF),$(SRC),$(SRC_SUFFIX)))
VHX_OBJ := $(addsuffix .$(VHX_SUFFIX),$(call make_target,$(VHX),$(SRC),$(SRC_SUFFIX)))
HEADERS := $(wildcard $(HEAD)/*.$(SRC_SUFFIX))
SOURCES := $(wildcard $(SRC)/*.$(SRC_SUFFIX))

################################################################################

.PHONY: clean

.SECONDARY:

all: $(VHX_OBJ)

$(VHX):
	mkdir $(VHX)/

$(ELF):
	mkdir $(ELF)/

$(VHX)/%.$(VHX_SUFFIX): $(ELF)/%.$(ELF_SUFFIX) | $(VHX)
	$(ELFCC) $(ELFFLAGS) $< $@ 

$(ELF)/%.$(ELF_SUFFIX): $(SRC)/%.$(SRC_SUFFIX) $(HEADERS) | $(ELF)
	$(CC) $(CFLAGS) -o $@ $< $(HEADERS)

clean:
	@rm -fv $(VHX_OBJ) $(ELF_OBJ)
