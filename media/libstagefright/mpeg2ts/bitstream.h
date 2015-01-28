/*
 * Copyright 2013 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
