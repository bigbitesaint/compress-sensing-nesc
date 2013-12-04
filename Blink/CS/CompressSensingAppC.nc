configuration CompressSensingAppC {
	provides interface Compressor;
}

implementation {
	components CompressSensingC, LocalTimeMilliC;

	
	CompressSensingC.LocalTime -> LocalTimeMilliC;

	Compressor = CompressSensingC.Compressor;
}

