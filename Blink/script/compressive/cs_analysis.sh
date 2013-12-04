#!/bin/bash
./parse_dat.sh ./compress_raw.csv > ./compress.csv
sudo octave --silent --path /home/adsc/Dropbox/compressive/Blink/octave --eval 'reconstruct_cs(2048,512)'
