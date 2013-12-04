#!/bin/bash
./parse_dat.sh ./baseline_raw.csv > ./baseline.csv
sudo octave --silent --path /home/adsc/Dropbox/compressive/Blink/octave --eval 'reconstruct_baseline(16,4)'
