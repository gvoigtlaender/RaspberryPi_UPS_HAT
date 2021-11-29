/*! \file bitop.h
 * Contains macros for simple bit operations.
 */
/* Copyright (c) 2012-2013 Domen Ipavec (domen.ipavec@z-v.si)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef BITTOP_H
#define BITTOP_H

#include <stdint.h>

/*!
 * \defgroup Bitop 
 * \brief A few useful macros for setting, clearing, checking, ... bit(s).
 * 
 * Bit count starts at 0 for the least significant bit.
 * @{
 */

/*!
 * Returns binary mask of n-th bit.
 * Example: 2 --> 0100b
 * \param n Number of bit.
 */
#define BIT(n) (((uint32_t)1) << (n))

/*!
 * Sets bits in x, that are set in y.
 * \param x Variable.
 * \param y Binary mask of bits to set in x.
 */
#define SETBITS(x,y) ((x) |= (y))

/*!
 * Clear bits in x, that are set in y.
 * \param x Variable.
 * \param y Binary mask of bits to clear in x.
 */ 
#define CLEARBITS(x,y) ((x) &= (~(y)))

/*!
 * Toggle bits in x, that are set in y.
 * \param x Variable.
 * \param y Binary mask of bits to toggle in x.
 */
#define TOGGLEBITS(x,y) ((x) ^= (y))

/*!
 * Set n-th bit in x.
 * \param x Variable.
 * \param n Number of bit to set.
 */
#define SETBIT(x,n) (SETBITS((x), BIT((n))))

/*!
 * Clear n-th bit in x.
 * \param x Variable.
 * \param n Number of bit to clear.
 */
#define CLEARBIT(x,n) (CLEARBITS((x), BIT((n))))

/*!
 * Toggle n-th bit in x.
 * \param x Variable.
 * \param n Number of bit to toggle.
 */
#define TOGGLEBIT(x,n) (TOGGLEBITS((x), BIT((n))))

/*!
 * Check if n-th bit in x is set.
 * \param x Variable.
 * \param n Number of bit to check.
 */ 
#define BITSET(x,n) ((x) & BIT((n)))

/*!
 * Check if n-th bit in x is clear.
 * \param x Variable.
 * \param n Number of bit to check.
 */ 
#define BITCLEAR(x,y) (!BITSET((x), (y)))

/*!
 * Check if bits set in y are also set in x.
 * \param x Variable.
 * \param y Binary mask of bits to check.
 */
#define BITSSET(x,y) (((x) & (y)) == (y))

/*!
 * Check if bits set in y are clear in x.
 * \param x Variable.
 * \param y Binary mask of bits to check.
 */
#define BITSCLEAR(x,y) (((x) & (y)) == 0)

/*!
 * Get value of n-th bit (0 or 1).
 * \param x Variable.
 * \param n Number of bit.
 */
#define BITVAL(x,n) (((x)>>(n)) & 1)

/*! }@ */

#endif
