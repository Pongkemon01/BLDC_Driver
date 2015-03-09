/* BLDC_Fuzzy.c */
#define MEMBERSHIP_FUNCTION_MAX		5
#define FUZZY_MEMBER_MIN		(-50)
#define FUZZY_MEMBER_MAX		50
#define FUZZY_DEGREE_MAX		250U
#define FUZZY_MEMBER_HALF_BASE	25U
#define FUZZY_SLOPE_POSITIVE	( FUZZY_DEGREE_MAX / FUZZY_MEMBER_HALF_BASE )
typedef struct
{
	uint8_t	MemberDegree[MEMBERSHIP_FUNCTION_MAX];
}FuzzyDegree_t;

FuzzyDegree_t fuz_error, fuz_err_chg, fuz_pwm_chg;
/*-----------------------------------------------------------*/

/************************************************************************
 * A fuzzy logic controller for adjusting PWM duty cycle according to specified
 * BLDC speed. The fuzzy system consists of the following specifications
 *
 * Fuzzy variables
 * There are 3 fuzzy variables as follow. All are capped to the range
 * [-50, 50].
 *
 * - fuz_error : The difference between desired speed and current speed.
 *               (called E)
 * - fuz_err_chg : The difference between current fuz_error and the previous
 *                 fuz_error. (called EC)
 * - fuz_pwm_chg : The output from the inference rules. (called PC)
 *
 * Fuzzy membership functions
 * There are 5 membership functions for each fuzzy variable implemented as 
 * triangular function. Their details are depicted as:
 *
 *   250 +-------+-------+-------+-------+
 *       |\     / \     / \     / \     /|
 *       | \   /   \   /   \   /   \   / |
 *       |NL\ /  NM \ /  Z  \ /  PM \ /PL|
 *       |   X       X       X       X   |
 *       |  / \     / \     / \     / \  |
 *       | /   \   /   \   /   \   /   \ |
 *       |/     \ /     \ /     \ /     \|
 *     0 +-------+-------+-------+-------+
 *     -50     -25       0      25       50
 *
 * Fuzzy inference rules
 * We use min() and max() functions to represent AND and OR operations
 * respectively. The inference table is:
 *                          E
 *               NL   NM    Z   PM   PL
 *             +----+----+----+----+----+
 *          NL | NL | NL | NM | NM |  Z |
 *             +----+----+----+----+----+
 *          NM | NL | NM | NM |  Z | PM |
 *             +----+----+----+----+----+
 *   EC      Z | NM | NM |  Z | PM | PM |
 *             +----+----+----+----+----+
 *          PM | NM |  Z | PM | PM | PL |
 *             +----+----+----+----+----+
 *          PL |  Z | PM | PM | PL | PL |
 *             +----+----+----+----+----+
 *
 *
 ************************************************************************/

/************************************************************************
*                                                                       *
*      Function: 	     Fuzzify                                        *
*                                                                       *
*      Description:      Fuzzify a scalar value into specified fuzzy set.*
*                                                                       *
*      Parameters:       crisp : Scalar value                           *
*                        fuzzy : Pointer to a fuzzy-logic variable that *
*                                will store the result.                 *
*                                                                       *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
void Fuzzify( int16_t crisp, FuzzyDegree_t *fuzzy )
{
	uint8_t	i;
	int16_t limit_min;
	
	/* Initialize all membership degrees */
	for( i = 0; i < MEMBERSHIP_FUNCTION_MAX; i++ )
		fuzzy->MemberDegree[i] = 0;
	
	/* Fuzzification */
	if( crisp <=  FUZZY_MEMBER_MIN )
	{
		fuzzy->MemberDegree[0] = FUZZY_DEGREE_MAX;
	}
	else if( crisp > FUZZY_MEMBER_MAX )
	{
		fuzzy->MemberDegree[MEMBERSHIP_FUNCTION_MAX - 1] = FUZZY_DEGREE_MAX;
	}
	else /* FUZZY_MEMBER_MIN < crisp <= FUZZY_MEMBER_MAX */
	{
		limit_min = FUZZY_MEMBER_MIN;
		for( i = 1; i < MEMBERSHIP_FUNCTION_MAX; i++ )
		{
			if( crisp > limit_min && crisp <= ( limit_min + (int16_t)FUZZY_MEMBER_HALF_BASE ) )
			{
				fuzzy->MemberDegree[i] = (uint8_t)( FUZZY_SLOPE_POSITIVE * (uint16_t)( crisp - limit_min ) );
				fuzzy->MemberDegree[i - 1] = (uint8_t)(FUZZY_DEGREE_MAX - fuzzy->MemberDegree[i]);
			}
			limit_min += FUZZY_MEMBER_HALF_BASE;
		}
	}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     Inference                                      *
*                                                                       *
*      Description:      Inference error and error change with          *
*                        predefined rules.                              *
*                                                                       *
*      Parameters:                                                      *
*                                                                       *
*      Return value:                                                    *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/
const uint8_t FuzzyRules[MEMBERSHIP_FUNCTION_MAX][MEMBERSHIP_FUNCTION_MAX] =
{ { 0, 0, 1, 1, 2 },
  { 0, 1, 1, 2, 3 },
  { 1, 1, 2, 3, 3 },
  { 1, 2, 3, 3, 4 },
  { 2, 3, 3, 4, 4 } };
void Inference( void )
{
	uint8_t	i, in_index1, in_index2, out_index, infer_degree;
	
	/* Initialize all output membership degrees */
	for( i = 0; i < MEMBERSHIP_FUNCTION_MAX; i++ )
		fuz_pwm_chg.MemberDegree[i] = 0;
	
	/* Inference */
	for( in_index1 = 0; in_index1 < MEMBERSHIP_FUNCTION_MAX; in_index1++ )
		for( in_index2 = 0; in_index2 < MEMBERSHIP_FUNCTION_MAX; in_index2++ )
		{
			/* AND operation (min operation) */
			if( fuz_error.MemberDegree[in_index1] < fuz_err_chg.MemberDegree[in_index2] )
				infer_degree = fuz_error.MemberDegree[in_index1];
			else
				infer_degree = fuz_err_chg.MemberDegree[in_index2];
			
			/* Infer the rule */
			if( infer_degree > 0 )
			{
				out_index = FuzzyRules[in_index1][in_index2];
			
				/* OR operation (max operation) */
				if( fuz_pwm_chg.MemberDegree[out_index] < infer_degree )
					fuz_pwm_chg.MemberDegree[out_index] = infer_degree;
			}
		}
}
/*---------------------------------------------------------------------------*/

/************************************************************************
*                                                                       *
*      Function: 	     Defuzzify                                      *
*                                                                       *
*      Description:      De-fuzzify a fuzzy set into a scalar value.    *
*                                                                       *
*      Parameters:       fuzzy : Pointer to a fuzzy-logic variable that *
*                                will be de-fuzzified.                  *
*                                                                       *
*      Return value:     Crisp value from defuzzification               *
*                                                                       *
*      Note:             Since all further processing requires int16_t  *
*                        and all calculation in this function operates  *
*                        on int16_t; therefore, we return the raw result*
*                        without any type casting.                      *
*                                                                       *
*************************************************************************/
int16_t Defuzzify( FuzzyDegree_t *fuzzy )
{
	int16_t area, center, peak, dftemp;
	uint8_t i;
	
	/* Calculation using Center-of-Mass method. This is very same as
	weighted average of the peak of each membership function. */

	/* Initialize result */
	area = 0;
	center = 0;
	peak = FUZZY_MEMBER_MIN;

	/* Summation loop */
	for( i = 0; i < MEMBERSHIP_FUNCTION_MAX; i++ )
	{
		dftemp = ( ( int16_t )( fuzzy->MemberDegree[i] ));
		area +=  dftemp * peak;
		center += dftemp;
		peak += FUZZY_MEMBER_HALF_BASE;
	}
	
	/* return the result */
	return( area / center );
}

/************************************************************************
*                                                                       *
*      Function: 	     PWMControlEngine (aka. FuzzyEngine)            *
*                                                                       *
*      Description:      Implementation of the fuzzy controller         *
*                        operates as described above.                   *
*                                                                       *
*      Parameters:       err : Desired speed - Current speed            *
*                        err_chg : err - previous err                   *
*      Return value:     Delta PWM use to add to the current PWM value. *
*                                                                       *
*      Note:                                                            *
*                                                                       *
*************************************************************************/

int16_t PWMControlEngine( int16_t err, int16_t err_chg )
{
	Fuzzify( err, &fuz_error );
	Fuzzify( err_chg, &fuz_err_chg );
	
	Inference();
	
	return( Defuzzify( &fuz_pwm_chg ) );
}
 
/*---------------------------------------------------------------------------*/
/*===========================================================================*/

