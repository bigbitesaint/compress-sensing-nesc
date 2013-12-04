#!/bin/bash
./parse_dat.sh lossless_raw.csv 1 > lossless.csv
./rebuild lossless.csv
