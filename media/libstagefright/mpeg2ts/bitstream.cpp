/*
 *  Bit stream reader
 *  Copyright (C) 2007 Andreas Öman
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * changelog:
 * change it to fit latmaac, jan@2010-07-10
 */

#include <stdio.h>
#include <inttypes.h>
#include "bitstream.h"


void
init_bits(bitstream_t *bs, uint8_t *data, int bits)
{
  bs->data = data;
  bs->offset = 0;
  bs->len = bits;
}

void
skip_bits(bitstream_t *bs, int num)
{
  bs->offset += num;
}

unsigned int
read_bits(bitstream_t *bs, int num)
{
  int r = 0;

  while(num > 0) {
    if(bs->offset >= bs->len)
      return 0;

    num--;

    if(bs->data[bs->offset / 8] & (1 << (7 - (bs->offset & 7))))
      r |= 1 << num;

    bs->offset++;
  }
  return r;
}

unsigned int
read_bits1(bitstream_t *bs)
{
  return read_bits(bs, 1);
}

unsigned int
read_golomb_ue(bitstream_t *bs)
{
  int b, lzb = -1;

  for(b = 0; !b; lzb++)
    b = read_bits1(bs);

  return (1 << lzb) - 1 + read_bits(bs, lzb);
}


signed int
read_golomb_se(bitstream_t *bs)
{
  int v, neg;
  v = read_golomb_ue(bs);
  if(v == 0)
    return 0;

  neg = v & 1;
  v = (v + 1) >> 1;
  return neg ? -v : v;
}


unsigned int
remaining_bits(bitstream_t *bs)
{
  return bs->len - bs->offset;
}


void
put_bits(bitstream_t *bs, int val, int num)
{
  while(num > 0) {
    if(bs->offset >= bs->len)
      return;

    num--;

    if(val & (1 << num))
      bs->data[bs->offset / 8] |= 1 << (7 - (bs->offset & 7));
    else
      bs->data[bs->offset / 8] &= ~(1 << (7 - (bs->offset & 7)));

    bs->offset++;
  }
}

int put_bits_count(bitstream_t *bs)
{
    return bs->offset;
}

void flush_put_bits(bitstream_t *bs)
{
	put_bits(bs, 0, remaining_bits(bs));
}


/**
 * for pes parser
 */
int getu16(unsigned char **b, size_t *l) 		
{
  unsigned short x = ((*b)[0] << 8 | (*b)[1]);		
  *b+=2;						
  *l-=2;						
  return x;						
}

int getu8(unsigned char **b, size_t *l) 		
{
  unsigned char x = (*b)[0];				
  *b+=1;						
  *l-=1;						
  return x;						
}

int64_t getpts(unsigned char **b, size_t *l) 		
{
  int64_t _pts;						
  _pts = (int64_t)((getu8(b, l) >> 1) & 0x07) << 30;	
  _pts |= (int64_t)(getu16(b, l) >> 1) << 15;		
  _pts |= (int64_t)(getu16(b, l) >> 1);			
  return _pts;							
}


int get_bits_left(bitstream_t *bs)
{
	return bs->len - bs->offset;
}

int get_bits_count(bitstream_t *bs)
{
	return bs->offset;
}

int show_bits(bitstream_t *bs, int num)
{
	int r = 0;
	int offset = bs->offset;
	while(num > 0) {
		if(offset >= bs->len)
		  return 0;

		num--;

		if(bs->data[offset / 8] & (1 << (7 - (offset & 7))))
		  r |= 1 << num;

		offset++;
	}
	
  return r;
}

void align_get_bits(bitstream_t *bs)
{
    int n = -get_bits_count(bs) & 7;
    if (n) skip_bits(bs, n);
}




