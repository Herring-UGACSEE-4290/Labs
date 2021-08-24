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
#!/bin/bash

TESTS="sw/mem/*"
LOG_FILE="run_tests.log"

# Generate binaries to be picked up by wildcard in the following loop
make software

if [ $? -ne 0 ]; then
	exit 1
fi

echo "" > $LOG_FILE

for test in $TESTS
do
	DUMP_FILE=`basename $test`
	ARGS="+TEST_CASE=$test"

	make DUMP_FILE=$DUMP_FILE ARGS=$ARGS test_Educore.sim <<< cont \
	| tee -a $LOG_FILE \
	| grep "\[EDUCORE"
done
