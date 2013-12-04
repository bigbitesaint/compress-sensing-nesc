#!/bin/bash
cat $1 | awk -F, 'BEGIN { OFS=","; success=0; fail=0;}
{ 
	if ($4 == 0)
		fail++;
	if ($4 == 7)
	{
		fail --;
		success ++;
	}
}
END{
	printf "Success: %d, Fail: %d\n", success, fail;
}
'
