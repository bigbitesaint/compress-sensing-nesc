base=184
for (( i=0; i<4; i++ ))
do
    id=$(($base+$i))
    make telosb reinstall,$id bsl,/dev/ttyUSB$i
done