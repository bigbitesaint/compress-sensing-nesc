#include <stdio.h>
#define N 2048
#define M 512
#define D 4

unsigned long seed = 100;

unsigned long rand()
{
		  unsigned long mlcg,p,q;
		  unsigned long long tmpseed;
		  tmpseed =  (unsigned long long)33614U * (unsigned long long)seed;
		  q = tmpseed; 	/* low */
		  q = q >> 1;
		  p = tmpseed >> 32 ;		/* hi */
		  mlcg = p + q;
		  if (mlcg & 0x80000000) { 
					 mlcg = mlcg & 0x7FFFFFFF;
					 mlcg++;
		  }
		  seed = mlcg;
		  return mlcg; 
}


int main()
{
		int i,j,t;
		unsigned long r;
		int rand_array[M];
		for (i=0; i< M; ++i)
			rand_array[i] = i+1;

		for (i=0; i< N; ++i)
		{
			for (j=0; j<D; ++j)
			{
				r = rand();

				// permute 
				r = j+ (r % (M-j));

				// swap 
				t = rand_array[j];
				rand_array[j] = rand_array[r];
				rand_array[r] = t;

				printf("%2d ", rand_array[j]);

			}		
		}
		printf("\n");
		
		return 0;
}
