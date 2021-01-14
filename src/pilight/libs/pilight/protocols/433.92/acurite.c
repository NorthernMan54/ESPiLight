/*********************************************************************************************\
 * This plugin takes care of decoding the protocol used for AcuRite 986 Refrigerator and Freezer sensors
 * This Plugin works at least with: 
 * 
 * Author  (present)  : NorthernMan54
 * Support (present)  : https://github.com/couin3/RFLink 
 * Author  (original) : NorthernMan54 ( Borrowed heavily from rtl_433 )
 * Support (original) : http://sourceforge.net/projects/rflink/
 * License            : This code is free for use in any open source project when this header is included.
 *                      Usage of any parts of this code in a commercial application is prohibited!
 *********************************************************************************************

 * A preamble: 2x of 216 us pulse + 276 us gap, 4x of 1600 us pulse + 1560 us gap
 * 39 bits of data: 220 us pulses with short gap of 520 us or long gap of 880 us
 * A transmission consists of two packets that run into each other.
 * There should be 40 bits of data though. But the last bit can't be detected.
 * 
 * Technical Information:
 * Message Format: (10 nibbles, 40 bits):
 * 
 *     TT II II SS CC
- T - Temperature in Fahrenheit, integer, MSB = sign.
      Encoding is "Sign and magnitude", LSB first
- I - 16 bit sensor ID
      changes at each power up
- S - status/sensor type
      0x01 = Sensor 2
      0x02 = low battery
- C = CRC (CRC-8 poly 0x07, little-endian)
 *
 * Format for Temperature
 *               1            2           3
 *   0123 4567 8901 2345 6789 0123 4567 8901 2345 6789
 *   TTTT TTTT IIII IIII IIII IIII SSSS SSSS CCCC CCCC 
 *   1110 0100 0100 0111 1001 0100 0000 0001 0111 1100
 *   0110 0100 0100 0111 1001 0100 0000 0000 0100 1010
 *   0100 0000 0001 0100 1111 1001 0000 0001 0101 1110
 * 
 *   1101 0010 1001 1010 0001 0111 0000 0000 1111 0011 001
 *   0011 0010 0100 1101 0101 1010 0000 0000 0001 1100 001
 *   0011 0010 0001 1100 1010 1000 0000 0000 0110 0000 001
 *   0011 0010 0001 1110 1000 1110 0000 0000 0110 1011 001
 *   0010 0000 0111 1000 1110 1100 1000 0000 1100 1110 001
 *
 *   T - Temperature in Fahrenheit, integer, MSB = sign. Encoding is "Sign and magnitude"
 *   I - 16 bit sensor ID, changes at each power up
 *   S - status/sensor type, 0x80 = Sensor 2, 0x40 = low battery
 *
 * 20;XX;DEBUG;Pulses=176;Pulses(uSec)=1696,1472,1664,1280,256,416,256,448,256,512,256,832,256,512,256,832,256,416,256,512,256,864,256,800,256,512,256,832,224,512,256,832,256,512,256,832,256,512,256,864,256,896,256,832,224,448,256,512,256,832,256,448,224,480,256,448,256,448,256,448,224,448,224,480,224,480,224,480,224,512,224,832,224,512,224,896,224,832,224,480,224,480,224,544,224,224,224,224,1632,1504,1632,1504,1632,1504,1632,1344,224,480,224,480,224,544,224,832,224,544,224,864,224,480,224,544,224,896,224,864,224,544,224,864,224,544,224,864,224,544,224,864,224,544,224,928,224,928,224,864,224,480,224,544,224,864,224,480,224,480,224,480,224,480,224,480,224,480,224,480,192,480,192,480,192,544,192,864,224,544,224,928,224,864,192,480,192,480,192,480;
 * DEBUG;Pulses=193;Pulses=584,479,252,1503,227,1451,553,524,233,256,208,286,1594,1569,1619,1576,1592,1576,1600,1389,192,536,207,588,204,954,200,913,192,537,193,582,206,909,207,531,182,532,206,526,210,540,198,593,196,894,207,589,210,890,209,573,199,958,206,963,207,889,204,581,220,886,203,582,202,900,208,536,208,535,196,539,178,536,209,535,200,536,182,534,208,535,199,585,199,966,194,911,183,539,198,539,205,536,204,585,202,955,199,961,197,285,206,288,1599,1577,1592,1583,1586,1591,1605,1387,197,518,205,596,206,965,179,912,208,532,179,27,583,203,904,180,536,204,530,205,527,210,516,200,589,199,915,194,588,196,908,206,579,203,975,197,965,179,902,202,596,200,900,205,593,208,892,207,528,209,514,198,537,206,520,205,532,199,539,189,538,206,538,196,588,196,969,203,881,208,527,207,527,207,541,198,593,198,945,204, = 102274
 *  \*********************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ACURITE_HAS_TEMPERATURE
//#define ACURITE_HAS_HUMIDITY
#define ACURITE_HAS_BATTERY
// #define ACURITE_DEBUG

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../protocol.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "acurite.h"

#define PULSE_MULTIPLIER 4
#define MIN_PULSE_LENGTH 150
#define AVG_PULSE_LENGTH 750
#define MAX_PULSE_LENGTH 1100
#define MIN_RAW_LENGTH 170
#define MAX_RAW_LENGTH 200

typedef struct settings_t
{
    double id;
#ifdef ACURITE_HAS_TEMPERATURE
    double temp;
    int isCelsius;
#endif
#ifdef ACURITE_HAS_HUMIDITY
    double humi;
#endif
    struct settings_t *next;
} settings_t;

static struct settings_t *settings = NULL;

static int validate(void)
{
    if (acurite->rawlen >= MIN_RAW_LENGTH && acurite->rawlen <= MAX_RAW_LENGTH)
    {
#ifdef ACURITE_DEBUG
        // logprintf(LOG_DEBUG, "acurite last index [%d] = %d", acurite->rawlen - 2, acurite->raw[acurite->rawlen - 2]);
#endif
        int x = 0, messageTime = 0;
        for (x = 1; x < acurite->rawlen - 1; x += 2)
        {
            messageTime = messageTime + (acurite->raw[x - 1]) + (acurite->raw[x]);
        }
#ifdef ACURITE_DEBUG
        // logprintfLn(LOG_ERR, " = %d", messageTime);
#endif
        if (messageTime > 90000 && messageTime < 110000)
        {
            return 0;
        }
    }

    return -1;
}

static void parseCode(void)
{
    int i = 0, x = 0, binary[MAX_RAW_LENGTH / 2];
#ifdef ACURITE_DEBUG
    char binOut[MAX_RAW_LENGTH / 2];
#endif
    // int check = 0;

    double temp_offset = 0.0;
    double temperature = 0.0;
    double id = 0;
    int unit = 0;
    int battery = 0;
    int start = 0;

    // Find first byte post sync bit

    for (x = 2; x < acurite->rawlen - 1; x++)
    {
        // check = (int)((double)AVG_PULSE_LENGTH*(PULSE_MULTIPLIER*2));
        // logprintfLn(LOG_DEBUG, "acurite code %d < %d && %d > %d", acurite->raw[x] , MAX_PULSE_LENGTH , acurite->raw[x - 1] , MAX_PULSE_LENGTH);
        if (acurite->raw[x] < MAX_PULSE_LENGTH && acurite->raw[x - 1] > MAX_PULSE_LENGTH && acurite->raw[x - 2] > MAX_PULSE_LENGTH)
        {
            start = x;
            x = acurite->rawlen;
        }
    }

    for (x = start + 1; x < acurite->rawlen - 1; x += 2)
    {
        if (acurite->raw[x] > MAX_PULSE_LENGTH)
        {
            x = acurite->rawlen;
        }

        if (acurite->raw[x] > AVG_PULSE_LENGTH)
        {
#ifdef ACURITE_DEBUG
            binOut[i] = '1';
#endif
            binary[i++] = 1;
        }
        else
        {
#ifdef ACURITE_DEBUG
            binOut[i] = '0';
#endif
            binary[i++] = 0;
        }
    }

#ifdef ACURITE_DEBUG
    binOut[i] = '\0';
    logprintfLn(LOG_DEBUG, "acurite code %s", binOut);
#endif

    id = binToDec(binary, 8, 16);
    battery = binary[25]? 1 : 0;
    unit = binary[24];
    id = id + (((float)unit) / 10);

    temperature = ((double)binToSigned(binary, 0, 7));
    temperature = F2C(temperature);

    struct settings_t *tmp = settings;
    while (tmp)
    {
        if (fabs(tmp->id - id) < EPSILON)
        {
            temp_offset = tmp->temp;
            break;
        }
        tmp = tmp->next;
    }

    temperature += temp_offset;
#ifdef ACURITE_DEBUG
    logprintfLn(LOG_DEBUG, "Temperature: %f", temperature);
    logprintfLn(LOG_DEBUG, "battery: %d", battery);
    logprintfLn(LOG_DEBUG, "unit: %d", unit);
    logprintfLn(LOG_DEBUG, "id: %f", id);
#endif

    /*
    byte crcc = crc8le(binary, 32, 0x07, 0);
    logprintfLn(LOG_DEBUG, "crcc: %d", crcc);
    logprintfLn(LOG_DEBUG, "crc: %d", binToDec(binary,32,8)); // const int *binary, int s, int e
    if (crcc != reverse8(bitstream2))
    {
        Serial.println("ERROR: crc failed.");
        Serial.print("crcc le: ");
        Serial.println(crcc);
        Serial.print("crc: ");
        Serial.println(reverse8(bitstream2));
        return false;
    }
    */

    acurite->message = json_mkobject();
    json_append_member(acurite->message, "id", json_mknumber(id, 1));
    json_append_member(acurite->message, "temperature", json_mknumber(temperature, 1));
    json_append_member(acurite->message, "battery", json_mknumber(battery, 0));

    acurite->repeats = 1;   // Kludge for the the signal repeating in the pulse train
    acurite->old_content = json_encode(acurite->message);
}

static int checkValues(struct JsonNode *jvalues)
{
    struct JsonNode *jid = NULL;
    logprintfLn(LOG_DEBUG, "checkValues()");

    if ((jid = json_find_member(jvalues, "id")))
    {
        struct settings_t *snode = NULL;
        struct JsonNode *jchild = NULL;
        struct JsonNode *jchild1 = NULL;
        double id = -1;
        int match = 0;

        jchild = json_first_child(jid);
        while (jchild)
        {
            jchild1 = json_first_child(jchild);
            while (jchild1)
            {
                if (strcmp(jchild1->key, "id") == 0)
                {
                    id = jchild1->number_;
                }
                jchild1 = jchild1->next;
            }
            jchild = jchild->next;
        }

        struct settings_t *tmp = settings;
        while (tmp)
        {
            if (fabs(tmp->id - id) < EPSILON)
            {
                match = 1;
                break;
            }
            tmp = tmp->next;
        }

        if (match == 0)
        {
            if ((snode = MALLOC(sizeof(struct settings_t))) == NULL)
            {
                fprintf(stderr, "out of memory\n");
                exit(EXIT_FAILURE);
            }
            snode->id = id;

#ifdef ACURITE_HAS_TEMPERATURE
            snode->temp = 0;
            json_find_number(jvalues, "temperature-offset", &snode->temp);
#endif
            snode->next = settings;
            settings = snode;
        }
    }
    return 0;
}

static void gc(void)
{
    struct settings_t *tmp = NULL;
    while (settings)
    {
        tmp = settings;
        settings = settings->next;
        FREE(tmp);
    }
    if (settings != NULL)
    {
        FREE(settings);
    }
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void
acuriteInit(void)
{

    protocol_register(&acurite);
    protocol_set_id(acurite, "acurite");
    protocol_device_add(acurite, "acurite", "Acu-Rite 986 Temperature Sensor");
    acurite->devtype = WEATHER;
    acurite->hwtype = RF433;
    acurite->minrawlen = MIN_RAW_LENGTH;
    acurite->maxrawlen = MAX_RAW_LENGTH;
    acurite->maxgaplen = MAX_PULSE_LENGTH * PULSE_DIV;
    acurite->mingaplen = MIN_PULSE_LENGTH * PULSE_DIV;

    options_add(&acurite->options, "t", "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
    options_add(&acurite->options, "i", "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "[0-9]");
    options_add(&acurite->options, "b", "battery", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[01]$");

    // options_add(&acurite->options, "0", "decimals", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
    options_add(&acurite->options, "0", "temperature-offset", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *)0, "[0-9]");
    options_add(&acurite->options, "0", "temperature-decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
    options_add(&acurite->options, "0", "show-temperature", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");
    options_add(&acurite->options, "0", "show-battery", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");

    acurite->parseCode = &parseCode;
    acurite->checkValues = &checkValues;
    acurite->validate = &validate;
    acurite->gc = &gc;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module)
{
    module->name = "acurite";
    module->version = "1.0";
    module->reqversion = "6.0";
    module->reqcommit = "1";
}

void init(void)
{
    acuriteInit();
}

/** @file
    Various utility functions for use by device drivers.

    Copyright (C) 2015 Tommy Vestermark

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

// #include "7_Utils.h"
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>

uint8_t reverse8(uint8_t x)
{
    x = (x & 0xF0) >> 4 | (x & 0x0F) << 4;
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
}

void reflect_bytes(uint8_t message[], unsigned num_bytes)
{
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        message[i] = reverse8(message[i]);
    }
}

uint8_t reflect4(uint8_t x)
{
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2;
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1;
    return x;
}

void reflect_nibbles(uint8_t message[], unsigned num_bytes)
{
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        message[i] = reflect4(message[i]);
    }
}

unsigned extract_nibbles_4b1s(uint8_t *message, unsigned offset_bits, unsigned num_bits, uint8_t *dst)
{
    unsigned ret = 0;

    while (num_bits >= 5)
    {
        uint16_t bits = (message[offset_bits / 8] << 8) | message[(offset_bits / 8) + 1];
        bits >>= 11 - (offset_bits % 8); // align 5 bits to LSB
        if ((bits & 1) != 1)
            break; // stuff-bit error
        *dst++ = (bits >> 1) & 0xf;
        ret += 1;
        offset_bits += 5;
        num_bits -= 5;
    }

    return ret;
}

uint8_t crc4(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    unsigned remainder = init << 4; // LSBs are unused
    unsigned poly = polynomial << 4;
    unsigned bit;

    while (nBytes--)
    {
        remainder ^= *message++;
        for (bit = 0; bit < 8; bit++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1) ^ poly;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder >> 4 & 0x0f; // discard the LSBs
}

uint8_t crc7(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    unsigned remainder = init << 1; // LSB is unused
    unsigned poly = polynomial << 1;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1) ^ poly;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder >> 1 & 0x7f; // discard the LSB
}

uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1) ^ polynomial;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

uint8_t crc8le(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    uint8_t remainder = reverse8(init);
    unsigned byte, bit;
    polynomial = reverse8(polynomial);

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit)
        {
            if (remainder & 1)
            {
                remainder = (remainder >> 1) ^ polynomial;
            }
            else
            {
                remainder = (remainder >> 1);
            }
        }
    }
    return remainder;
}

uint16_t crc16lsb(uint8_t const message[], unsigned nBytes, uint16_t polynomial, uint16_t init)
{
    uint16_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte];
        for (bit = 0; bit < 8; ++bit)
        {
            if (remainder & 1)
            {
                remainder = (remainder >> 1) ^ polynomial;
            }
            else
            {
                remainder = (remainder >> 1);
            }
        }
    }
    return remainder;
}

uint16_t crc16(uint8_t const message[], unsigned nBytes, uint16_t polynomial, uint16_t init)
{
    uint16_t remainder = init;
    unsigned byte, bit;

    for (byte = 0; byte < nBytes; ++byte)
    {
        remainder ^= message[byte] << 8;
        for (bit = 0; bit < 8; ++bit)
        {
            if (remainder & 0x8000)
            {
                remainder = (remainder << 1) ^ polynomial;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

uint8_t lfsr_digest8(uint8_t const message[], unsigned bytes, uint8_t gen, uint8_t key)
{
    uint8_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k)
    {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i)
        {
            // fprintf(stderr, "key is %02x\n", key);
            // XOR key into sum if data bit is set
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}

uint8_t lfsr_digest8_reflect(uint8_t const message[], int bytes, uint8_t gen, uint8_t key)
{
    uint8_t sum = 0;
    // Process message from last byte to first byte (reflected)
    for (int k = bytes - 1; k >= 0; --k)
    {
        uint8_t data = message[k];
        // Process individual bits of each byte (reflected)
        for (int i = 0; i < 8; ++i)
        {
            // fprintf(stderr, "key is %02x\n", key);
            // XOR key into sum if data bit is set
            if ((data >> i) & 1)
            {
                sum ^= key;
            }

            // roll the key left (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 0x80)
                key = (key << 1) ^ gen;
            else
                key = (key << 1);
        }
    }
    return sum;
}

uint16_t lfsr_digest16(uint32_t data, int bits, uint16_t gen, uint16_t key)
{
    uint16_t sum = 0;
    for (int bit = bits - 1; bit >= 0; --bit)
    {
        // fprintf(stderr, "key at bit %d : %04x\n", bit, key);
        // if data bit is set then xor with key
        if ((data >> bit) & 1)
            sum ^= key;

        // roll the key right (actually the lsb is dropped here)
        // and apply the gen (needs to include the dropped lsb as msb)
        if (key & 1)
            key = (key >> 1) ^ gen;
        else
            key = (key >> 1);
    }
    return sum;
}

/*
void lfsr_keys_fwd16(int rounds, uint16_t gen, uint16_t key)
{
    for (int i = 0; i <= rounds; ++i) {
        fprintf(stderr, "key at bit %d : %04x\n", i, key);

        // roll the key right (actually the lsb is dropped here)
        // and apply the gen (needs to include the dropped lsb as msb)
        if (key & 1)
            key = (key >> 1) ^ gen;
        else
            key = (key >> 1);
    }
}

void lfsr_keys_rwd16(int rounds, uint16_t gen, uint16_t key)
{
    for (int i = 0; i <= rounds; ++i) {
        fprintf(stderr, "key at bit -%d : %04x\n", i, key);

        // roll the key left (actually the msb is dropped here)
        // and apply the gen (needs to include the dropped msb as lsb)
        if (key & (1 << 15))
            key = (key << 1) ^ gen;
        else
            key = (key << 1);
    }
}
*/

// we could use popcount intrinsic, but don't actually need the performance
int parity8(uint8_t byte)
{
    byte ^= byte >> 4;
    byte &= 0xf;
    return (0x6996 >> byte) & 1;
}

int parity_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        result ^= parity8(message[i]);
    }
    return result;
}

uint8_t xor_bytes(uint8_t const message[], unsigned num_bytes)
{
    uint8_t result = 0;
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        result ^= message[i];
    }
    return result;
}

int add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        result += message[i];
    }
    return result;
}

int add_nibbles(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i)
    {
        result += (message[i] >> 4) + (message[i] & 0x0f);
    }
    return result;
}

// Unit testing
#ifdef _TEST
int main(int argc, char **argv)
{
    fprintf(stderr, "util:: test\n");

    uint8_t msg[] = {0x08, 0x0a, 0xe8, 0x80};

    fprintf(stderr, "util::crc8(): odd parity:  %02X\n", crc8(msg, 3, 0x80, 0x00));
    fprintf(stderr, "util::crc8(): even parity: %02X\n", crc8(msg, 4, 0x80, 0x00));

    return 0;
}
#endif /* _TEST */

#endif