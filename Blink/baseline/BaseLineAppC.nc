configuration BaseLineAppC {
	provides interface Compressor;
}

implementation {
	components BaseLineC, LocalTimeMilliC;

	
	BaseLineC.LocalTime -> LocalTimeMilliC;

	Compressor = BaseLineC.Compressor;
}

