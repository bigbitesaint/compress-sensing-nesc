/* S-LZW with Mini-Cache - Functions to perform the S-LZW compression algorithm presented in:
*
*  C. Sadler and M. Martonosi, “Data Compression Algorithms for Energy-Constrained Devices in Delay 
*  Tolerant Networks,” Proceedings of the ACM Conference on Embedded Networked Sensor Systems 
*  (SenSys) 2006, November 2006.
*
* Note:  I do not include the BWT transform code here because it is based off of code that I
* downloaded from http://www.dogma.net/markn/articles/bwt/bwt.htm and I have not attempted to get
* permission to post it.  If you want a copy, though, email me and I'll send you the original code
* and my amendments so you can see what I did.
* 
* Copyright (C) 2005-2006 Princeton University, Christopher M. Sadler (csadler@princeton.edu)
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License along
*    with this program; if not, write to the Free Software Foundation, Inc.,
*    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

//  Revision History:
//		25 Oct 2006		Chris Sadler		Initial Release
//		10 Mar 2007		Chris Sadler		Increased WRITE_BUFFER_SIZE to properly handle the
//											case where the data is very random and compression
//											actually causes it to expand.

#ifndef __SLZW_H
#define __SLZW_H
#include "../lossless.h"
// This page size is based on a 4 Mbit Atmel Flash.
#define FLASH_PAGE_SIZE				512

// Two flash pages.  This is the maximum amount of data that you can compress.  If you increase it,
// it will cost you more RAM.
#define SLZW_BLOCK_SIZE					(FLASH_PAGE_SIZE << 1)
									
// You can increase the number of dictionary entries to 4096, but there are only 12 bits allocated
// in each dictionary entry so it won't work for more than that.							  
#define MAX_DICT_ENTRIES			512
#define MAX_ENTRY_BITS				9

// Uncomment this line if you want the dictionary to reset when it fills.
//#define RESETABLE_DICTIONARY

// These definitions are for the mini-cache.
#define USE_MINICACHE

// If this #define is uncommented, it will add entries to the mini-cache as they are created.
// We recommend that you enable this option.
#define ADD_NEWEST

#define MAX_SIZE_OF_MINI_CACHE		32

// Structured Transform Parameters.  The numbers here are for the sample file attached.
#define STRUCTURED_TRANSFORM
#define SIZE_OF_READING				1
#define NUMBER_OF_READINGS			(BLOCK_SIZE*4)				// The sample file has 500B of actual data.

// -------------------------------------------------------------------------------------------
// I recommend not changing anything below this line.
// -------------------------------------------------------------------------------------------

// Technically, if the data is completely random, LZW can increase the size of the data.  If you
// have 10 bit entries (9 bit dictionary plus a 1 bit escape character for the mini-cache), the
// maximum size is (10/8)*size of the buffer.  We use a larger buffer to make sure we don't
// accidentally over run.
#define WRITE_BUFFER_SIZE			((FLASH_PAGE_SIZE << 1) + (FLASH_PAGE_SIZE >> 1))

#define LZW_BUFFER_SIZE				WRITE_BUFFER_SIZE

// Due to bit alignment issues on the MSP, I has to break part of the hash up into two parts.
//  These masks help the program figure out which part is which.
#define HASH1_MASK					0xFF00
#define HASH2_MASK					0x00FF

#define HIT							1
#define MISS						0

#define get_hash(x) ((x.next_hash1 << 8) + x.next_hash2)

typedef struct dict_node {
	unsigned longer_string:12;	// When we need to look for a longer string, we head to this index.
	unsigned next_hash1:4;		// When we need to look for a different string entry of the same
	unsigned next_hash2:8;		//  length, we look here.
	unsigned char entry;
} Dict_node;

typedef struct undict_node {
	int up;						// Signed with plenty of extra bits.
	unsigned char entry;
	unsigned char undefined;	// To make the structure an even number of bytes.
} Undict_node;

// Holds the output.
unsigned char lzw_output_file_buffer[LZW_BUFFER_SIZE];

// Holds the input.
unsigned char write_buffer[SLZW_BLOCK_SIZE];


unsigned int slzw_decompress(unsigned int in_filesize);
unsigned int slzw_compress(unsigned int in_filesize, unsigned char cache_size);



#endif	// __SLZW_H
