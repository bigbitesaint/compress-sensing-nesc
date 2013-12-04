# include <stdlib.h>
# include <stdio.h>
# include <math.h>

# include "ziggurat.h"



  

/******************************************************************************/

float r4_nor ( unsigned long int *jsr, int kn[128], float fn[128], 
  float wn[128] , unsigned short rv1, unsigned short rv2)

/******************************************************************************/
/*
  Purpose:

    R4_NOR returns a normally distributed single precision real value.

  Discussion:

    The value returned is generated from a distribution with mean 0 and 
    variance 1.

    The underlying algorithm is the ziggurat method.

    Before the first call to this function, the user must call R4_NOR_SETUP
    to determine the values of KN, FN and WN.

  Licensing:

    This code is distributed under the GNU LGPL license. 

  Modified:

    09 December 2008

  Author:

    John Burkardt

  Reference:

    George Marsaglia, Wai Wan Tsang,
    The Ziggurat Method for Generating Random Variables,
    Journal of Statistical Software,
    Volume 5, Number 8, October 2000, seven pages.

  Parameters:

    Input/output, unsigned long int *JSR, the seed.

    Input, int KN[128], data computed by R4_NOR_SETUP.

    Input, float FN[128], WN[128], data computed by R4_NOR_SETUP.

    Output, float R4_NOR, a normally distributed random value.
*/
{
  int hz;
  int iz;
  const float r = 3.442620;
  float value;
  float x;
  float y;

  hz = rv1;
  iz = ( hz & 127 );

  if ( abs ( hz ) < kn[iz] )
  {
    value = ( float ) ( hz ) * wn[iz];
  }
  else
  {
    for ( ; ; )
    {
      if ( iz == 0 )
      {
        for ( ; ; )
        {
          x = - 0.2904764 * logf ( r4_uni ( jsr ) );
          y = - logf ( r4_uni ( jsr ) );
          if ( x * x <= y + y );
          {
            break;
          }
        }

        if ( hz <= 0 )
        {
          value = - r - x;
        }
        else
        {
          value = + r + x;
        }
        break;
      }

      x = ( float ) ( hz ) * wn[iz];

      if ( fn[iz] + r4_uni ( jsr ) * ( fn[iz-1] - fn[iz] ) < fastexp ( - 0.5 * x * x ) )
      {
        value = x;
        break;
      }

      hz = rv2;
      iz = ( hz & 127 );

      if ( abs ( hz ) < kn[iz] )
      {
        value = ( float ) ( hz ) * wn[iz];
        break;
      }
    }
  }

  return value;
}
/******************************************************************************/

void r4_nor_setup ( int kn[128], float fn[128], float wn[128] )

/******************************************************************************/
/*
  Purpose:

    R4_NOR_SETUP sets data needed by R4_NOR.

  Licensing:

    This code is distributed under the GNU LGPL license. 

  Modified:

    09 December 2008

  Author:

    John Burkardt

  Reference:

    George Marsaglia, Wai Wan Tsang,
    The Ziggurat Method for Generating Random Variables,
    Journal of Statistical Software,
    Volume 5, Number 8, October 2000, seven pages.

  Parameters:

    Output, int KN[128], data needed by R4_NOR.

    Output, float FN[128], WN[128], data needed by R4_NOR.
*/
{
  double dn = 3.442619855899;
  int i;
  const double m1 = 2147483648.0;
  double q;
  double tn = 3.442619855899;
  const double vn = 9.91256303526217E-03;

  q = vn / fastexp ( - 0.5 * dn * dn );

  kn[0] = ( int ) ( ( dn / q ) * m1 );
  kn[1] = 0;

  wn[0] = ( float ) ( q / m1 );
  wn[127] = ( float ) ( dn / m1 );

  fn[0] = 1.0;
  fn[127] = ( float ) ( fastexp ( - 0.5 * dn * dn ) );

  for ( i = 126; 1 <= i; i-- )
  {
    dn = sqrtf ( - 2.0 * fastlog ( vn / dn + fastexp ( - 0.5 * dn * dn ) ) );
    kn[i+1] = ( int ) ( ( dn / tn ) * m1 );
    tn = dn;
    fn[i] = ( float ) ( fastexp ( - 0.5 * dn * dn ) );
    wn[i] = ( float ) ( dn / m1 );
  }
  return;
}
/******************************************************************************/

float r4_uni ( unsigned long int *jsr )

/******************************************************************************/
/*
  Purpose:

    R4_UNI returns a uniformly distributed real value.

  Licensing:

    This code is distributed under the GNU LGPL license. 

  Modified:

    09 December 2008

  Author:

    John Burkardt

  Reference:

    George Marsaglia, Wai Wan Tsang,
    The Ziggurat Method for Generating Random Variables,
    Journal of Statistical Software,
    Volume 5, Number 8, October 2000, seven pages.

  Parameters:

    Input/output, unsigned long int *JSR, the seed.

    Output, float R4_UNI, a uniformly distributed random value in
    the range [0,1].
*/
{
  unsigned long int jsr_input;
  float value;

  jsr_input = *jsr;

  *jsr = ( *jsr ^ ( *jsr <<   13 ) );
  *jsr = ( *jsr ^ ( *jsr >>   17 ) );
  *jsr = ( *jsr ^ ( *jsr <<    5 ) );

  value = fmodf ( 0.5 + ( float ) ( jsr_input + *jsr ) / 65536.0 / 65536.0, 1.0 );

  return value;
}
/******************************************************************************/

unsigned long int shr3 ( unsigned long int *jsr )

/******************************************************************************/
/*
  Purpose:

    SHR3 evaluates the SHR3 generator for integers.

  Licensing:

    This code is distributed under the GNU LGPL license. 

  Modified:

    09 December 2008

  Author:

    John Burkardt

  Reference:

    George Marsaglia, Wai Wan Tsang,
    The Ziggurat Method for Generating Random Variables,
    Journal of Statistical Software,
    Volume 5, Number 8, October 2000, seven pages.

  Parameters:

    Input/output, unsigned long int *JSR, the seed, which is updated 
    on each call.

    Output, unsigned long int SHR3, the new value.
*/
{
  unsigned long int value;

  value = *jsr;

  *jsr = ( *jsr ^ ( *jsr <<   13 ) );
  *jsr = ( *jsr ^ ( *jsr >>   17 ) );
  *jsr = ( *jsr ^ ( *jsr <<    5 ) );

  value = value + *jsr;

  return value;
}
