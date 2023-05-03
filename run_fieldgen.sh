#!/bin/bash


echo computing $2 field with s = $3 and t = $4
./../deps/fieldgen/fieldgen ../test/temp_obj.eobj ../test/final_fields.obj --degree=$1 $2 --s=$3 --t=$4 