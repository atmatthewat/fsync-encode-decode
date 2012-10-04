/*-
 * fsync_decode.c
 *   Decodes a specific format of 1200 BPS MSK data burst
 *   from input audio samples.
 *
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

#include <stdlib.h>
#include "fsync_decode.h"
#include "fsync_common.c"

fsync_decoder_t * fsync_decoder_new(int sampleRate)
{
	fsync_decoder_t *decoder;
	fsync_int_t i;

	decoder = (fsync_decoder_t *)malloc(sizeof(fsync_decoder_t));
	if(!decoder)
		return (fsync_decoder_t *) 0L;

	decoder->hyst = 3.0/256.0;
	decoder->incr = (1200.0 * TWOPI) / ((fsync_float_t)sampleRate);
	decoder->actives = 0;
	decoder->level = 0;

	for(i=0; i<FSYNC_ND; i++)
	{
		decoder->th[i] = 0.0 + ( ((fsync_float_t)i) * (TWOPI/(fsync_float_t)FSYNC_ND));
		decoder->zc[i] = 0;
		decoder->xorb[i] = 0;
		decoder->shstate[i] = 0;
		decoder->shcount[i] = 0;
	}

	return decoder;
}

static void _dispatch(fsync_decoder_t *decoder, int x)
{
	unsigned char payload[128];
	int paylen = 0;
	int aflag;
	int fleetflag;
	fsync_u8_t m0, m1;
	fsync_u8_t *msg = decoder->message[x];
	fsync_int_t msglen = decoder->msglen[x];
	fsync_int_t from_fleet, from_unit, to_fleet, to_unit;

	if(msg[0] & 0x01)
		aflag = 1;
	else
		aflag = 0;

	if(msg[1] & 0x01)
		fleetflag = 1;
	else
		fleetflag = 0;

	m0 = msg[0] & 0xfe;
	m1 = msg[1] & 0xfe;

	from_fleet = msg[2] + 99;
	from_unit = (msg[3] << 4) + ((msg[4] >> 4) & 0x0f) + 999;
	to_unit = ((msg[4] << 8) & 0xf00) + msg[5] + 999;

	if(fleetflag)
	{
		if(msglen < 7)
			return;	// cannot dispatch, not enough data
		to_fleet = msg[6] + 99;
	}
	else
	{
		to_fleet = from_fleet;
	}

	if(from_fleet == 99)
		from_fleet = -1;
	if(to_fleet == 99)
		to_fleet = -1;
	if(to_unit == 999)
		to_unit = -1;
	if(from_unit == 999)
		from_unit = -1;

	if(m1 == 0x42)
	{
		int i, offset;

		if(msglen < 11)
			return;	// truncated before length field, cannot dispatch

		paylen = (msg[9] << 8) + msg[10];

		// XXX unsure if shuffle is appropriate for GPS message (subcmd 0xf4)

		for(i=0; i<paylen; i++)
		{
			offset = 11 + (6*((i/6)+1)) - (i%6);
			if(offset >= msglen)
				return;	// truncated message (XXX could instead dispatch what we have but with a truncated flag
			if(i >= sizeof(payload))
				return; // should never happen unless paylen is corrupted

			payload[i] = msg[offset];
		}
	}
	else
	{
		paylen = 0;
	}

	if(decoder->callback)
		(decoder->callback)((int)m1, (int)m0, (int)from_fleet, (int)from_unit, (int)to_fleet, (int)to_unit, (int)aflag, payload, paylen, (unsigned char *)msg, (int)msglen);

}
	

static void _procbits(fsync_decoder_t *decoder, int x)
{
	int crc;


	crc = _fsync_crc(decoder->word1[x], decoder->word2[x]);

	if(crc == (decoder->word2[x] & 0x0000ffff))
	{
		int i;

		decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >> 24) & 0xff;
		decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >> 16) & 0xff;
		decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >> 8) & 0xff;
		decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >> 0) & 0xff;
		decoder->message[x][decoder->msglen[x]++] = (decoder->word2[x] >> 24) & 0xff;
		decoder->message[x][decoder->msglen[x]++] = (decoder->word2[x] >> 16) & 0xff;
		// followed by 15 bits of crc and 1 parity bit (already checked)

		// go get next word, if there is one
		decoder->shstate[x] = 1;
		decoder->shcount[x] = 32;

#if 0
		// and abort rest

		// we no longer abort the rest, as we give each decoder a shot
		// at each component of a multi-part message

		for(i=0; i<FSYNC_ND; i++)
		{
			if(i != x)
			{
				decoder->shstate[i] = 0;
			}
		}
#endif
	}
	else
	{
		decoder->actives--;
		if(decoder->msglen[x] > 0)
		{
			if(decoder->actives)
			{
				// others are working, might decode a longer successful run
				// XXX this approach is a little sensitive to one of them re-syncing on something
			}
			else
			{
				_dispatch(decoder, x);
			}
		}
		decoder->shstate[x] = 0;
	}
}

static int _onebits(fsync_u32_t n)
{
	int i=0;
	while(n)
	{
		++i;
		n &= (n-1);
	}
	return i;
}

static void _shiftin(fsync_decoder_t *decoder, int x)
{
	int bit = decoder->xorb[x];

	decoder->synchigh[x] <<= 1;
	if(decoder->synclow[x] & 0x80000000)
		decoder->synchigh[x] |= 1;

	decoder->synclow[x] <<= 1;
	if(bit)
		decoder->synclow[x] |= 1;

	switch(decoder->shstate[x])
	{
	case 0:
	 // sync 23eb or can also be 0x052B?
		if(_onebits(decoder->synchigh[x]^0xaaaa23eb) < FSYNC_GDTHRESH || _onebits(decoder->synchigh[x]^0xaaaa052b) < FSYNC_GDTHRESH)
		{
			decoder->actives++;
			// note, we do sync on the trailing 32 bits and skip state 1 here to ensure we can catch back-to-back messages with the same decoder phase
			decoder->word1[x] = decoder->synclow[x];
			decoder->shstate[x] = 2;
			decoder->shcount[x] = 32;
			decoder->msglen[x] = 0;
		}
		return;
	case 1:
		if(--decoder->shcount[x]<= 0)
		{
			decoder->word1[x] = decoder->synclow[x];
			decoder->shcount[x] = 32;
			decoder->shstate[x] = 2;
		}

		return;
	case 2:
		if(--decoder->shcount[x]<= 0)
		{
			decoder->word2[x] = decoder->synclow[x];
			_procbits(decoder, x);
		}
		return;

	default:
		decoder->shstate[x] = 0; // should never happen
		return;
	}
}

static void _zcproc(fsync_decoder_t *decoder, int x)
{
	switch(decoder->zc[x])
	{
	case 2:
	case 4:
		decoder->xorb[x] = 1;
		break;
	case 3:
		decoder->xorb[x] = 0;
		break;
	default:
		return;
	}

	_shiftin(decoder, x);
}


int fsync_decoder_process_samples(fsync_decoder_t *decoder,
                                fsync_sample_t *samples,
                                int numSamples)
{
	fsync_int_t i, j, k;
	fsync_sample_t sample;
	fsync_float_t value;
	fsync_float_t delta;

	if(!decoder)
		return -1;

	for(i = 0; i<numSamples; i++)
	{
		sample = samples[i];

#if defined(FSYNC_SAMPLE_FORMAT_U8)
		value = (((fsync_float_t)sample) - 128.0)/256;
#elif defined(FSYNC_SAMPLE_FORMAT_U16)
		value = (((fsync_float_t)sample) - 32768.0)/65536.0;
#elif defined(FSYNC_SAMPLE_FORMAT_S16)
		value = ((fsync_float_t)sample) / 65536.0;
#elif defined(FSYNC_SAMPLE_FORMAT_FLOAT)
		value = sample;
#else
#error "no known sample format set"
#endif

#ifdef ZEROCROSSING

#ifdef DIFFERENTIATOR
		delta = value - decoder->lastvalue;
		decoder->lastvalue = value;

		if(decoder->level == 0)
		{
			if(delta > decoder->hyst)
			{
				for(k=0; k<FSYNC_ND; k++)
					decoder->zc[k]++;
				decoder->level = 1;
			}
		}
		else
		{
			if(delta < (-1 * decoder->hyst))
			{
				for(k=0; k<FSYNC_ND; k++)
					decoder->zc[k]++;
				decoder->level = 0;
			}
		}
#else	/* DIFFERENTIATOR */
		if(decoder->level == 0)
		{
			if(s > decoder->hyst)
			{
				for(k=0; k<FSYNC_ND; k++)
					decoder->zc[k]++;
				decoder->level = 1;
			}
		}
		else
		{
			if(s < (-1.0 * decoder->hyst))
			{
				for(k=0; k<FSYNC_ND; k++)
					decoder->zc[k]++;
				decoder->level = 0;
			}
		}
#endif	/* DIFFERENTIATOR */
		

		for(j=0; j<FSYNC_ND; j++)
		{
			decoder->th[j] += decoder->incr;
			if(decoder->th[j] >= TWOPI)
			{
				_zcproc(decoder, j);
				decoder->th[j] -= TWOPI;
				decoder->zc[j] = 0;
			}
		}
#else	/* ZEROCROSSING */

#error "no alternative to ZEROCROSSING for fsync yet"

#endif
	}

	// XXX callback only -- no longer return 1 if good and have a way to get message. ok?

	return 0;
}

int fsync_decoder_end_samples(fsync_decoder_t *decoder)
{
	if(!decoder)
		return -1;

	int i, j;

	for(i = 0; i < 70; i++)
	{
		for(j = 0; j < FSYNC_ND; j++)
		{
			_shiftin(decoder, j);
		}
	}

	return 0;
}

int fsync_decoder_set_callback(fsync_decoder_t *decoder, fsync_decoder_callback_t callbackFunction)
{
	if(!decoder)
		return -1;

	decoder->callback = callbackFunction;

	return 0;
}

