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

#ifndef BITSTREAM_H_
#define BITSTREAM_H_


//2 rockchip platform:
//typedef unsigned char uint8_t;
//typedef unsigned int uint32_t;
//typedef __int64    int64_t;
//#define printf(...)

typedef struct bitstream {
  unsigned char *data;
  int offset;
  int len;
} bitstream_t;

void skip_bits(bitstream_t *bs, int num);

void init_bits(bitstream_t *bs, unsigned char *data, int bits);

unsigned int read_bits(bitstream_t *gb, int num);

unsigned int read_bits1(bitstream_t *gb);

unsigned int read_golomb_ue(bitstream_t *gb);

signed int read_golomb_se(bitstream_t *gb);

unsigned int remaining_bits(bitstream_t *gb);

void put_bits(bitstream_t *bs, int val, int num);

int put_bits_count(bitstream_t *bs);

void flush_put_bits(bitstream_t *bs);

int getu16(unsigned char **b, size_t *l); 		

int getu8(unsigned char **b, size_t *l);

int64_t getpts(unsigned char **b, size_t *l);

int get_bits_left(bitstream_t *bs);
int get_bits_count(bitstream_t *bs);
int show_bits(bitstream_t *bs, int num);

void align_get_bits(bitstream_t *bs);


#endif /* BITSTREAM_H_ */
