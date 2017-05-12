
/*! \file util_numbers.c
 *
 * Number utilities
 *
 * @defgroup util_numbers  Numbers Utilities
 * @{
 */

/*!
 * Reference
 * https://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
 *
 * Note: Clear bits 13->31 prior to calling.
 * Warning: Untested on 64 bit integers.
 *
 * example
 *			//clear dv bit
 *			tint_msb = ((~(1 << 7)) & tint_msb);
 *			tint = sign_extend_12bit((tint_msb << 8) | tint_lsb);
 *
 */
signed int sign_extend_12bit(signed int x)
{
	signed int r = 0;
	struct
	{
		signed int x: 12;
	} s;

	r = s.x = x;

	return r;
}

//! @}
