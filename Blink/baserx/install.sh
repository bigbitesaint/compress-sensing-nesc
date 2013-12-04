port=$(motelist | grep "A700azND" | awk '{print $2;}')
make telosb reinstall,1234 bsl,$port
