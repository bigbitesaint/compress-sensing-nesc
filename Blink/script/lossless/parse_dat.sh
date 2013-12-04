#!/bin/bash
cat $1 | awk -F, 'BEGIN { OFS=","; ELINE=0;CLINE=0;ALL_P=0;REC_P=0; FIRST_TS=0; LAST_TS=0;R_BLOCK=0}
{ 
	if (NF != 0)
	{
		if (FIRST_TS == 0)
			FIRST_TS = $2;
		LAST_TS=$2;
		if ($6 == 0)
		{
			R_BLOCK++;
			if (int(CLINE) != int(ELINE))
				R_BLOCK--;
			REC_P += int(CLINE);
			ALL_P += int(ELINE);


			for (i=int(CLINE); i< int(ELINE); ++i)
				printf "-1\n";

			printf "%d,%d",$2,$3;
			for (i=10; i<=NF; ++i)
				printf ",%d",$i;
			printf "\n"
			ELINE = int(int($10)/16);
			if ( int($10)%16 != 0 )
				ELINE = ELINE + 1;
			ELINE = ELINE*16;
			ELINE += 2;
			#printf "[%d]\n",ELINE;
			if ( ELINE%32 != 0)
				ELINE = ELINE/32 + 1;
			else
				ELINE = ELINE/32;
			CLINE=1;
		}
		else
		{
			CLINE ++;
			printf "%d",$10;
			for (i=11; i<=NF; ++i)
				printf ",%d",$i;
			printf "\n"
		}
	}
}
END{ printf "PRR: %.2f%%\n",(REC_P/ALL_P)*100 > "/dev/stderr";
	E_BLOCK = (LAST_TS - FIRST_TS)/4096;
	printf "BRR: %.2f%%\n", R_BLOCK/E_BLOCK*100 > "/dev/stderr";
}'
