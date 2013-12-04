configuration SlzwAppC{
	provides interface Compressor;
}

implementation {
	components RandomC, SlzwC, LocalTimeMilliC;

	
	SlzwC.Random -> RandomC;
	SlzwC.LocalTime -> LocalTimeMilliC;

	Compressor = SlzwC.Compressor;
}
