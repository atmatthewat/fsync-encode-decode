/*-
 * fsync_common.c
 * Author: Matthew Kaufman (matthew@eeph.com)
 *
 * Copyright (c) 2012  Matthew Kaufman  All rights reserved.
 * 
 *  This file is part of Matthew Kaufman's fsync Encoder/Decoder Library
 *
 *  The fsync Encoder/Decoder Library is free software; you can
 *  redistribute it and/or modify it under the terms of version 2 of
 *  the GNU General Public License as published by the Free Software
 *  Foundation.
 *
 *  If you cannot comply with the terms of this license, contact
 *  the author for alternative license arrangements or do not use
 *  or redistribute this software.
 *
 *  The fsync Encoder/Decoder Library is distributed in the hope
 *  that it will be useful, but WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 *  USA.
 *
 *  or see http://www.gnu.org/copyleft/gpl.html
 *
-*/





static int _fsync_crc(int word1, int word2)
{

        int paritybit = 0;
        int crcsr = 0;
        int bit;
        int cur;
        int invert;

        for(bit = 0; bit < 48; bit++)
        {
                if(bit < 32)
                {
                    cur = (word1 >> (31-bit)) & 0x01;
                }
                else
                {
                    cur = (word2 >> (31-(bit-32))) & 0x01;
                }
                if(cur)
                    paritybit ^= 1;
                invert = cur ^ (0x01 & (crcsr>>15));
                if(invert)
                {
                        crcsr ^= 0x6815;
                }
                crcsr <<= 1;
        }

		for(bit = 48; bit<63; bit++)
		{
                cur = (word2 >> (31-(bit-32))) & 0x01;
                if(cur)
                    paritybit ^= 1;
		}

        crcsr ^= 0x0002;
        crcsr += paritybit;
        return crcsr & 0xffff;
}



