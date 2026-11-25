/* sane - Scanner Access Now Easy.

   BACKEND canon_lide70

   Copyright (C) 2019-2021 Juergen Ernst and pimvantend.

   This file is part of the SANE package.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.

   This file implements a SANE backend for the Canon CanoScan LiDE 70 and 600 */

#include <errno.h>
#include <fcntl.h>		/* open */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>		/* usleep */
#include <time.h>
#include <math.h>		/* pow() */
#include <signal.h>		/* sig_atomic_t */
#ifdef HAVE_OS2_H
#include <sys/types.h>		/* mode_t */
#endif
#include <sys/stat.h>

#define USB_TYPE_VENDOR   (0x02 << 5)
#define USB_RECIP_DEVICE   0x00
#define USB_DIR_OUT        0x00
#define USB_DIR_IN         0x80

#define MSEC               1000	/* 1ms = 1000us */

/* Assign status and verify a good return code */
#define CHK(A) {if ((status = A) != SANE_STATUS_GOOD) {\
                DBG (1, "Failure on line of %s: %d\n", \
                     __FILE__, __LINE__ ); return status; }}

typedef SANE_Byte byte;

/*****************************************************
           Canon LiDE70 calibration and scan
******************************************************/

/* at 600 dpi */
#define CANON_MAX_WIDTH    5104	/* 8.5in */
/* this may not be right */
#define CANON_MAX_HEIGHT   7300	/* 11.66in */
/* Just for my scanner, or is this universal?  Calibrate? */

/* data structures and constants */
typedef struct CANON_Handle
{
  /* options */
  SANE_Option_Descriptor opt[num_options];
  Option_Value val[num_options];
  SANE_Parameters params;

  SANE_Word graymode;
  char *product;		/* product name */
  int productcode;		/* product code, 0x2224 or 0x2225 */
  int fd;			/* scanner fd */
  int x1, x2, y1, y2;		/* in pixels, at 600 dpi */
  long width, height;		/* at scan resolution */
  unsigned char value_08, value_09;	/* left */
  unsigned char value_0a, value_0b;	/* right */
  unsigned char value_66, value_67, value_68;	/* bottom */
  unsigned char value_51;	/* lamp colors */
  unsigned char value_90;	/* motor mode */
  int resolution;		/* dpi */
  char *fname;			/* output file name */
  FILE *fp;			/* output file pointer (for reading) */
  unsigned char absolute_threshold;
  double table_gamma;
  double table_gamma_blue;
  unsigned char highlight_red_enhanced;
  unsigned char highlight_blue_reduced;
  unsigned char highlight_other;
  /* shading calibration (see make_shading) */
  SANE_Bool linear;		/* identity gamma tables: linear sensor values */
  int afe_gain;			/* analog front end gain code, 0: default */
  unsigned short *shading;	/* per byte of a line: gain, 8.8 fixed point */
  unsigned char shading_gamma[256];	/* applied after the gain */
  float *strip;			/* last strip reading, strip_columns per color */
  int strip_columns;
  int strip_dpi;
  int strip_gain;
  time_t strip_time;
}
CANON_Handle;

/*****************************************************
            CP2155 communication primitives
   Provides I/O routines to Philips CP2155BE chip
******************************************************/

typedef int CP2155_Register;

/* Write single byte to CP2155 register */
static SANE_Status
cp2155_set (int fd, CP2155_Register reg, byte data)
{
  SANE_Status status;
  byte cmd_buffer[5];
  size_t count = 5 /* = sizeof(cmd_buffer) */ ;

  cmd_buffer[0] = (reg >> 8) & 0xff;
  cmd_buffer[1] = (reg) & 0xff;
  cmd_buffer[2] = 0x01;
  cmd_buffer[3] = 0x00;
  cmd_buffer[4] = data;

/*  if (cmd_buffer[0]==0 && cmd_buffer[1]>0x21 && cmd_buffer[1]<0x44)
    { */
  DBG (1, "cp2155_set %02x %02x %02x %02x %02x\n",
       cmd_buffer[0], cmd_buffer[1], cmd_buffer[2],
       cmd_buffer[3], cmd_buffer[4]);
/*    } */
/* */
  usleep (0.0 * MSEC);
/* */
  status = sanei_usb_write_bulk (fd, cmd_buffer, &count);

  if (status != SANE_STATUS_GOOD)
    {
      DBG (1, "cp2155_set: sanei_usb_write_bulk error\n");
/*      exit(0); */
    }

  return status;
}

/* Read single byte from CP2155 register */
static SANE_Status
cp2155_get (int fd, CP2155_Register reg, byte * data)
{
  SANE_Status status;
  byte cmd_buffer[4];
  size_t count = 4;		/* = sizeof(cmd_buffer) */

  cmd_buffer[0] = 0x01;
  cmd_buffer[1] = (reg) & 0xff;
  cmd_buffer[2] = 0x01;
  cmd_buffer[3] = 0x00;

  status = sanei_usb_write_bulk (fd, cmd_buffer, &count);

  if (status != SANE_STATUS_GOOD)
    {
      DBG (1, "cp2155_get: sanei_usb_write_bulk error\n");
      return status;
    }

  usleep (1 * MSEC);

  count = 1;
  status = sanei_usb_read_bulk (fd, data, &count);

  if (status != SANE_STATUS_GOOD)
    {
      DBG (1, "cp2155_get: sanei_usb_read_bulk error\n");
    }

  return status;
}

/* Read a block of data from CP2155 chip */
static SANE_Status
cp2155_read (int fd, byte * data, size_t size)
{
  SANE_Status status;
  byte cmd_buffer[4];
  size_t count = 4;		/* = sizeof(cmd_buffer) */

  cmd_buffer[0] = 0x05;
  cmd_buffer[1] = 0x70;
  cmd_buffer[2] = (size) & 0xff;
  cmd_buffer[3] = (size >> 8) & 0xff;

  status = sanei_usb_write_bulk (fd, cmd_buffer, &count);

  if (status != SANE_STATUS_GOOD)
    {
      DBG (1, "cp2155_read: sanei_usb_write_bulk error\n");
      return status;
    }

  usleep (1 * MSEC);

  count = size;
  status = sanei_usb_read_bulk (fd, data, &count);
/*
  if (status != SANE_STATUS_GOOD)
    {
      DBG (1, "cp2155_read: sanei_usb_read_bulk error %lu\n", (u_long) count);
    }
*/
  return status;
}

/*****************************************************/

static void
cp2155_write_gamma_block (int fd, unsigned int addr, byte * data)
{
  byte value_71 = 0x16;
  size_t count = 0x100;

  while ((count & 0x0f) != 0)
    {
      count++;
    }

  byte pgLO = (count) & 0xff;
  byte pgHI = (count >> 8) & 0xff;
/*
  DBG (1, "cp2155_write_gamma_block %06x %02x %04lx %04lx\n", addr, v001, (u_long) size,
       (u_long) count);
*/
  cp2155_set (fd, 0x71, 0x01);
  cp2155_set (fd, 0x0230, 0x11);
  cp2155_set (fd, 0x71, value_71);
  cp2155_set (fd, 0x72, pgHI);
  cp2155_set (fd, 0x73, pgLO);
  cp2155_set (fd, 0x74, (addr >> 16) & 0xff);
  cp2155_set (fd, 0x75, (addr >> 8) & 0xff);
  cp2155_set (fd, 0x76, (addr) & 0xff);
  cp2155_set (fd, 0x0239, 0x40);
  cp2155_set (fd, 0x0238, 0x89);
  cp2155_set (fd, 0x023c, 0x2f);
  cp2155_set (fd, 0x0264, 0x20);

  count = count + 4;
  sanei_usb_write_bulk (fd, data, &count);
}

void
makegammatable (double gamma, int highlight, unsigned char *buf)
{
  int maxin = 255;		/* 8 bit gamma input */
  int maxout = 255;		/* 8 bit gamma output */
  int in = 0;
  int out;

  buf[0] = 0x04;
  buf[1] = 0x70;
  buf[2] = 0x00;
  buf[3] = 0x01;

  while (in < highlight)
    {
      out = maxout * pow ((double) in / highlight, (1.0 / gamma));
      buf[in + 4] = (unsigned char) out;
      in++;
    }

  while (in <= maxin)
    {
      buf[in + 4] = maxout;
      in++;
    }

  return;
}

/* Identity gamma table: the scanner returns linear sensor values.

   buf (260 bytes) is sent as is by cp2155_write_gamma_block, in the same
   layout as makegammatable's:
     buf[0..1]    0x04 0x70: the CP2155 register the table is written to
     buf[2..3]    0x00 0x01: its length, little-endian (0x0100 = 256 bytes)
     buf[4..259]  the table: the output for each 8-bit input 0..255, so
                  entry `in` is at buf[in + 4] */
static void
make_linear_gamma_table (unsigned char *buf)
{
  int in;

  buf[0] = 0x04;
  buf[1] = 0x70;
  buf[2] = 0x00;
  buf[3] = 0x01;
  for (in = 0; in <= 255; in++)
    {
      buf[in + 4] = (unsigned char) in;
    }
}

static void
cp2155_set_linear_gamma (int fd)
{
  unsigned char buf[260];

  DBG (1, "cp2155_set_linear_gamma\n");
  make_linear_gamma_table (buf);
  cp2155_write_gamma_block (fd, 0x000, buf);
  cp2155_write_gamma_block (fd, 0x100, buf);
  cp2155_write_gamma_block (fd, 0x200, buf);
}

static void
cp2155_set_gamma (int fd, CANON_Handle * chndl)
{
  DBG (1, "cp2155_set_gamma\n");
  unsigned char buf[260];
  if (chndl->linear)
    {
      cp2155_set_linear_gamma (fd);
      return;
    }
/* gamma tables */
  makegammatable (chndl->table_gamma, chndl->highlight_other, buf);
  cp2155_write_gamma_block (fd, 0x000, buf);
  cp2155_write_gamma_block (fd, 0x100, buf);
  cp2155_write_gamma_block (fd, 0x200, buf);
}

static void
cp2155_set_gamma_red_enhanced (int fd, CANON_Handle * chndl)
{
  DBG (1, "cp2155_set_gamma\n");
  unsigned char buf[260];
  if (chndl->linear)
    {
      cp2155_set_linear_gamma (fd);
      return;
    }
/* gamma tables */
  makegammatable (chndl->table_gamma, chndl->highlight_red_enhanced, buf);
  cp2155_write_gamma_block (fd, 0x000, buf);
  makegammatable (chndl->table_gamma, chndl->highlight_other, buf);
  cp2155_write_gamma_block (fd, 0x100, buf);
  makegammatable (chndl->table_gamma_blue, chndl->highlight_blue_reduced,
		  buf);
  cp2155_write_gamma_block (fd, 0x200, buf);
}

void
make_descending_slope (size_t start_descent, double coefficient,
		       unsigned char *buf)
{
  size_t count, position;
  int top_value;
  int value;
  unsigned char value_lo, value_hi;
  DBG (1, "start_descent = %lx\n", start_descent);
  top_value = buf[start_descent - 2] + 256 * buf[start_descent - 1];
  DBG (1, "buf[start_descent-2] = %02x buf[start_descent-1] = %02x\n",
       buf[start_descent - 2], buf[start_descent - 1]);
  count = buf[2] + 256 * buf[3];
  position = start_descent;
  DBG (1, "count = %ld top_value = %d\n", count, top_value);
  while (position < count + 4)
    {
      value =
	(int) (top_value /
	       (1.0 + coefficient * (position + 2 - start_descent)));
      value_lo = value & 0xff;
      value_hi = (value >> 8) & 0xff;
      buf[position] = value_lo;
      buf[position + 1] = value_hi;
      DBG (1,
	   "position = %03lx  buf[position]= %02x buf[position+1] = %02x\n",
	   position, buf[position], buf[position + 1]);
      position += 2;
    }
}

void
make_constant_buf (size_t count, unsigned int hiword, unsigned int loword,
		   unsigned char *buf)
{
  size_t i = 4;
  unsigned char hihi = (hiword >> 8) & 0xff;
  unsigned char hilo = (hiword) & 0xff;
  unsigned char lohi = (loword >> 8) & 0xff;
  unsigned char lolo = (loword) & 0xff;
  buf[0] = 0x04;
  buf[1] = 0x70;
  buf[2] = (count - 4) & 0xff;
  buf[3] = ((count - 4) >> 8) & 0xff;
  while (i < count)
    {
      buf[i] = hilo;
      i++;
      buf[i] = hihi;
      i++;
      buf[i] = lolo;
      i++;
      buf[i] = lohi;
      i++;
    }
}

void
make_slope_table (size_t count, unsigned int word, size_t start_descent,
		  double coefficient, unsigned char *buf)
{
  size_t i = 4;
  unsigned char hi = (word >> 8) & 0xff;
  unsigned char lo = (word) & 0xff;
  buf[0] = 0x04;
  buf[1] = 0x70;
  buf[2] = (count - 4) & 0xff;
  buf[3] = ((count - 4) >> 8) & 0xff;
  while (i < start_descent)
    {
      buf[i] = lo;
      i++;
      buf[i] = hi;
      i++;
    }
  make_descending_slope (start_descent, coefficient, buf);
}

void
write_buf (int fd, size_t count, unsigned char *buf,
	   unsigned char value_74, unsigned char value_75)
{
  unsigned char value_72, value_73;
  value_72 = ((count - 4) >> 8) & 0xff;
  value_73 = (count - 4) & 0xff;
  cp2155_set (fd, 0x71, 0x01);
  cp2155_set (fd, 0x0230, 0x11);
  cp2155_set (fd, 0x71, 0x14);
  cp2155_set (fd, 0x72, value_72);
  cp2155_set (fd, 0x73, value_73);
  cp2155_set (fd, 0x74, value_74);
  cp2155_set (fd, 0x75, value_75);
  cp2155_set (fd, 0x76, 0x00);
  cp2155_set (fd, 0x0239, 0x40);
  cp2155_set (fd, 0x0238, 0x89);
  cp2155_set (fd, 0x023c, 0x2f);
  cp2155_set (fd, 0x0264, 0x20);
  sanei_usb_write_bulk (fd, buf, &count);
}

void
big_write (int fd, size_t count, unsigned char *buf)
{
  make_constant_buf (count, 62756, 20918, buf);
  write_buf (fd, count, buf, 0x00, 0x00);
  write_buf (fd, count, buf, 0x00, 0xb0);
  write_buf (fd, count, buf, 0x01, 0x60);
  write_buf (fd, count, buf, 0x02, 0x10);
}

void
big_write_2224 (int fd, size_t count, unsigned char *buf)
{
  make_constant_buf (count, 62756, 30918, buf);
  write_buf (fd, count, buf, 0x00, 0x00);
  write_buf (fd, count, buf, 0x00, 0xb0);
  write_buf (fd, count, buf, 0x01, 0x60);
  write_buf (fd, count, buf, 0x02, 0x10);
}

void
big_write_film (int fd, size_t count, unsigned char *buf)
{
  make_constant_buf (count, 62756, 20918, buf);
  write_buf (fd, count, buf, 0x00, 0x00);
  write_buf (fd, count, buf, 0x02, 0x00);
  write_buf (fd, count, buf, 0x04, 0x00);
  write_buf (fd, count, buf, 0x06, 0x00);
}

void
general_motor_2225 (int fd)
{
  cp2155_set (fd, 0x9b, 0x02);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x91);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x03, 0x01);
  cp2155_set (fd, 0x71, 0x01);
  cp2155_set (fd, 0x0230, 0x11);
  cp2155_set (fd, 0x71, 0x18);
  cp2155_set (fd, 0x72, 0x00);
  cp2155_set (fd, 0x73, 0x10);
  cp2155_set (fd, 0x0239, 0x40);
  cp2155_set (fd, 0x0238, 0x89);
  cp2155_set (fd, 0x023c, 0x2f);
  cp2155_set (fd, 0x0264, 0x20);
}

void
general_motor_2224 (int fd)
{
  cp2155_set (fd, 0x90, 0xfa);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x91);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x03, 0x01);
  cp2155_set (fd, 0x71, 0x01);
  cp2155_set (fd, 0x0230, 0x11);
  cp2155_set (fd, 0x71, 0x18);
  cp2155_set (fd, 0x72, 0x00);
  cp2155_set (fd, 0x73, 0x10);
  cp2155_set (fd, 0x0239, 0x40);
  cp2155_set (fd, 0x0238, 0x89);
  cp2155_set (fd, 0x023c, 0x2f);
  cp2155_set (fd, 0x0264, 0x20);
}

void
register_table (int fd, unsigned char register_value, unsigned char *buf)
{
  cp2155_set (fd, 0x1a, 0x00);
  cp2155_set (fd, 0x1b, 0x00);
  cp2155_set (fd, 0x1c, 0x02);
  cp2155_set (fd, 0x15, 0x80);
  cp2155_set (fd, 0x14, 0x7c);
  cp2155_set (fd, 0x17, 0x01);
  cp2155_set (fd, 0x43, 0x1c);
  cp2155_set (fd, 0x44, 0x9c);
  cp2155_set (fd, 0x45, 0x38);
  if (register_value > 0)
    {
      unsigned char register_number = 0x23;
      while (register_number < 0x43)
	{
	  cp2155_set (fd, register_number, register_value);
	  register_number++;
	}
    }
  else
    {
      int buffer_index = 0;
      cp2155_set (fd, 0x23 + buffer_index, buf[buffer_index]);
      cp2155_set (fd, 0x33 + buffer_index, buf[buffer_index]);
      buffer_index++;
      while (buffer_index < 9)
	{
	  cp2155_set (fd, 0x23 + buffer_index, buf[buffer_index]);
	  cp2155_set (fd, 0x33 + buffer_index, buf[buffer_index]);
	  cp2155_set (fd, 0x43 - buffer_index, buf[buffer_index]);
	  cp2155_set (fd, 0x33 - buffer_index, buf[buffer_index]);
	  buffer_index++;
	}
    }

  cp2155_set (fd, 0xca, 0x00);
  cp2155_set (fd, 0xca, 0x00);
  cp2155_set (fd, 0xca, 0x00);

}

void
register_table_4800 (int fd, unsigned char register_value, unsigned char *buf)
{
  cp2155_set (fd, 0x1a, 0x00);
  cp2155_set (fd, 0x1b, 0x00);
  cp2155_set (fd, 0x1c, 0x02);
  cp2155_set (fd, 0x15, 0x80);
  cp2155_set (fd, 0x14, 0x7a);
  cp2155_set (fd, 0x17, 0x02);
  cp2155_set (fd, 0x43, 0x1c);
  cp2155_set (fd, 0x44, 0x9c);
  cp2155_set (fd, 0x45, 0x38);
  if (register_value > 0)
    {
      unsigned char register_number = 0x23;
      while (register_number < 0x43)
	{
	  cp2155_set (fd, register_number, register_value);
	  register_number++;
	}
    }
  else
    {
      int buffer_index = 0;
      cp2155_set (fd, 0x23 + buffer_index, buf[buffer_index]);
      cp2155_set (fd, 0x33 + buffer_index, buf[buffer_index]);
      buffer_index++;
      while (buffer_index < 9)
	{
	  cp2155_set (fd, 0x23 + buffer_index, buf[buffer_index]);
	  cp2155_set (fd, 0x33 + buffer_index, buf[buffer_index]);
	  cp2155_set (fd, 0x43 - buffer_index, buf[buffer_index]);
	  cp2155_set (fd, 0x33 - buffer_index, buf[buffer_index]);
	  buffer_index++;
	}
    }

  cp2155_set (fd, 0xca, 0x00);
  cp2155_set (fd, 0xca, 0x00);
  cp2155_set (fd, 0xca, 0x00);

}

void
startblob_2225_0075 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;
  ratio 320 decimal

*/
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0xb0, 0x03);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x06);
  cp2155_set (fd, 0xa3, 0x70);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0x32);
  cp2155_set (fd, 0x5b, 0x32);
  cp2155_set (fd, 0x5c, 0x32);
  cp2155_set (fd, 0x5d, 0x32);
  cp2155_set (fd, 0x52, 0x09);
  cp2155_set (fd, 0x53, 0x5a);
  cp2155_set (fd, 0x54, 0x06);
  cp2155_set (fd, 0x55, 0x08);
  cp2155_set (fd, 0x56, 0x05);
  cp2155_set (fd, 0x57, 0x5f);
  cp2155_set (fd, 0x58, 0xa9);
  cp2155_set (fd, 0x59, 0xce);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x83, 0x02);
  cp2155_set (fd, 0x84, 0x06);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x0b);

  big_write (fd, 0x5174, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x9b, 0x03);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x40);
  cp2155_set (fd, 0x13, 0x40);
  cp2155_set (fd, 0x16, 0x40);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x40);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0xf0);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  memcpy (buf, "\x28\x27\x25\x21\x1c\x16\x0f\x08\x00", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 260;
  make_slope_table (count, top_value, 0x6a, 0.021739, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.15217, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2225 (fd);
}

void
startblob_2225_0150 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x1e;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x1e;
  ratio 320 decimal
*/
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0xb0, 0x02);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x0c);
  cp2155_set (fd, 0xa3, 0xd0);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0xa0);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0x32);
  cp2155_set (fd, 0x5b, 0x32);
  cp2155_set (fd, 0x5c, 0x32);
  cp2155_set (fd, 0x5d, 0x32);
  cp2155_set (fd, 0x52, 0x09);
  cp2155_set (fd, 0x53, 0x5a);
  cp2155_set (fd, 0x54, 0x06);
  cp2155_set (fd, 0x55, 0x08);
  cp2155_set (fd, 0x56, 0x05);
  cp2155_set (fd, 0x57, 0x5f);
  cp2155_set (fd, 0x58, 0xa9);
  cp2155_set (fd, 0x59, 0xce);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x83, 0x02);
  cp2155_set (fd, 0x84, 0x06);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x0a);

  big_write (fd, 0x5174, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x9b, 0x03);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x40);
  cp2155_set (fd, 0x13, 0x40);
  cp2155_set (fd, 0x16, 0x40);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x40);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  memcpy (buf, "\x28\x27\x25\x21\x1c\x16\x0f\x08\x00", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 260;
  make_slope_table (count, top_value, 0x06, 0.0089185, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.102968, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2225 (fd);
}

void
startblob_2225_0300 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2a;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2a;
  ratio 228 decimal
*/
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0xb0, 0x01);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x19);
  cp2155_set (fd, 0xa3, 0x30);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x80);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0x32);
  cp2155_set (fd, 0x5b, 0x32);
  cp2155_set (fd, 0x5c, 0x32);
  cp2155_set (fd, 0x5d, 0x32);
  cp2155_set (fd, 0x52, 0x09);
  cp2155_set (fd, 0x53, 0x5a);
  cp2155_set (fd, 0x54, 0x06);
  cp2155_set (fd, 0x55, 0x08);
  cp2155_set (fd, 0x56, 0x05);
  cp2155_set (fd, 0x57, 0x5f);
  cp2155_set (fd, 0x58, 0xa9);
  cp2155_set (fd, 0x59, 0xce);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x83, 0x02);
  cp2155_set (fd, 0x84, 0x06);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x09);

  big_write (fd, 0x5174, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x9b, 0x01);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x0c);
  cp2155_set (fd, 0x13, 0x0c);
  cp2155_set (fd, 0x16, 0x0c);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x0c);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table (fd, 0x14, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 52;
  make_slope_table (count, top_value, 0x06, 0.0038363, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0080213, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2225 (fd);
}

void
startblob_2225_0600 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x15;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x15;
  ratio 457 decimal
*/
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0x90, 0xd8);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x77);
  cp2155_set (fd, 0xa3, 0xb0);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0xe0);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0x32);
  cp2155_set (fd, 0x5b, 0x32);
  cp2155_set (fd, 0x5c, 0x32);
  cp2155_set (fd, 0x5d, 0x32);
  cp2155_set (fd, 0x52, 0x07);
  cp2155_set (fd, 0x53, 0xd0);
  cp2155_set (fd, 0x54, 0x07);
  cp2155_set (fd, 0x55, 0xd0);
  cp2155_set (fd, 0x56, 0x07);
  cp2155_set (fd, 0x57, 0xd0);
  cp2155_set (fd, 0x58, 0x00);
  cp2155_set (fd, 0x59, 0x01);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x83, 0x02);
  cp2155_set (fd, 0x84, 0x06);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x9b, 0x01);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0xc3);
  cp2155_set (fd, 0x11, 0xc3);
  cp2155_set (fd, 0x11, 0xc3);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x12, 0x12);
  cp2155_set (fd, 0x13, 0x00);
  cp2155_set (fd, 0x16, 0x12);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x12);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table (fd, 0x14, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 84;
  make_slope_table (count, top_value, 0x06, 0.0020408, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0064935, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2225 (fd);
}

void
startblob_2225_1200 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0xff00;
  unsigned char value_62 = 0xaa;

/* original:
  unsigned int top_value = 0xff00;
  unsigned char value_62 = 0xaa;
*/
  cp2155_set (fd, 0x90, 0xc8);
  cp2155_set (fd, 0x90, 0xe8);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x63);
  cp2155_set (fd, 0xa3, 0xd0);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0x32);
  cp2155_set (fd, 0x5b, 0x32);
  cp2155_set (fd, 0x5c, 0x32);
  cp2155_set (fd, 0x5d, 0x32);
  cp2155_set (fd, 0x52, 0x11);
  cp2155_set (fd, 0x53, 0x50);
  cp2155_set (fd, 0x54, 0x0c);
  cp2155_set (fd, 0x55, 0x01);
  cp2155_set (fd, 0x56, 0x0a);
  cp2155_set (fd, 0x57, 0xae);
  cp2155_set (fd, 0x58, 0xa9);
  cp2155_set (fd, 0x59, 0xce);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x83, 0x02);
  cp2155_set (fd, 0x84, 0x06);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x08);

  big_write (fd, 0xa1a4, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x9b, 0x01);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x06);
  cp2155_set (fd, 0x13, 0x06);
  cp2155_set (fd, 0x16, 0x06);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x06);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  memcpy (buf, "\x14\x14\x12\x11\x0e\x0b\x08\x04\x00", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x01);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);
  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2225 (fd);
}

void
startblob_2224_0075 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;
  ratio 208 decimal
*/
  cp2155_set (fd, 0x90, 0xe8);
  cp2155_set (fd, 0x9b, 0x06);
  cp2155_set (fd, 0x9b, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0xb0, 0x03);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x06);
  cp2155_set (fd, 0xa3, 0x70);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x0c);
  cp2155_set (fd, 0x53, 0xda);
  cp2155_set (fd, 0x54, 0x0c);
  cp2155_set (fd, 0x55, 0x44);
  cp2155_set (fd, 0x56, 0x08);
  cp2155_set (fd, 0x57, 0xbb);
  cp2155_set (fd, 0x58, 0x1d);
  cp2155_set (fd, 0x59, 0xa1);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x0b);

  big_write (fd, 0x5694, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x7d);
  cp2155_set (fd, 0x13, 0x7d);
  cp2155_set (fd, 0x16, 0x7d);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x7d);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x71);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table (fd, 0x0f, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 516;
  make_slope_table (count, top_value, 0x6a, 0.0084116, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.15217, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_0150 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x1e;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x1e;
  ratio 320 decimal
*/
  cp2155_set (fd, 0x90, 0xe8);
  cp2155_set (fd, 0x9b, 0x06);
  cp2155_set (fd, 0x9b, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0xb0, 0x02);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x0c);
  cp2155_set (fd, 0xa3, 0xd0);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0xa0);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x0c);
  cp2155_set (fd, 0x53, 0xda);
  cp2155_set (fd, 0x54, 0x0c);
  cp2155_set (fd, 0x55, 0x44);
  cp2155_set (fd, 0x56, 0x08);
  cp2155_set (fd, 0x57, 0xbb);
  cp2155_set (fd, 0x58, 0x1d);
  cp2155_set (fd, 0x59, 0xa1);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x0a);

  big_write (fd, 0x5694, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x40);
  cp2155_set (fd, 0x13, 0x40);
  cp2155_set (fd, 0x16, 0x40);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x40);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table (fd, 0x0d, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 260;
  make_slope_table (count, top_value, 0x86, 0.017979, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.102968, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_0300 (CANON_Handle * chndl, unsigned char *buf)
{
  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x3200;
  unsigned char value_62 = 0x15;

/* original:
  unsigned int top_value = 0x3200;
  unsigned char value_62 = 0x15;
  ratio 609.52 decimal
*/
  cp2155_set (fd, 0x90, 0xe8);
  cp2155_set (fd, 0x9b, 0x06);
  cp2155_set (fd, 0x9b, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0xb0, 0x01);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x03);
  cp2155_set (fd, 0xa3, 0x10);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0xe0);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x0a);
  cp2155_set (fd, 0x53, 0xf0);
  cp2155_set (fd, 0x54, 0x0a);
  cp2155_set (fd, 0x55, 0xf0);
  cp2155_set (fd, 0x56, 0x0a);
  cp2155_set (fd, 0x57, 0xf0);
  cp2155_set (fd, 0x58, 0x00);
  cp2155_set (fd, 0x59, 0x01);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x01);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0xc3);
  cp2155_set (fd, 0x11, 0xc3);
  cp2155_set (fd, 0x11, 0xc3);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x11, 0xc1);
  cp2155_set (fd, 0x12, 0x40);
  cp2155_set (fd, 0x13, 0x00);
  cp2155_set (fd, 0x16, 0x40);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x40);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table (fd, 0x0a, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 260;
  make_slope_table (count, top_value, 0x66, 0.0129596, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.09307359, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_0600 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x19;

/* original:
  unsigned int top_value = 0x7f80;
  unsigned char value_62 = 0x55;
  ratio 384 decimal
*/
  cp2155_set (fd, 0x90, 0xe8);
  cp2155_set (fd, 0x9b, 0x06);
  cp2155_set (fd, 0x9b, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x31);
  cp2155_set (fd, 0xa3, 0xf0);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x0c);
  cp2155_set (fd, 0x53, 0xda);
  cp2155_set (fd, 0x54, 0x0c);
  cp2155_set (fd, 0x55, 0x44);
  cp2155_set (fd, 0x56, 0x08);
  cp2155_set (fd, 0x57, 0xbb);
  cp2155_set (fd, 0x58, 0x1d);
  cp2155_set (fd, 0x59, 0xa1);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x08);

  big_write_2224 (fd, 0x5694, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x06);
  cp2155_set (fd, 0x13, 0x06);
  cp2155_set (fd, 0x16, 0x06);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x06);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table (fd, 0x0c, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);
  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_1200 (CANON_Handle * chndl, unsigned char *buf)
{
/*
  chndl->value_51 = 0x0f;
*/
  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x7c71;
  unsigned char value_62 = 0x29;

/*
  unsigned int top_value = 0x3fc7;
  unsigned char value_62 = 0x15;
  ratio 777 decimal
*/

  cp2155_set (fd, 0x90, 0xe0);	/* e8 */

  double n_msec = 10.0;
  int n_9b = 10;		/* 2 */
  while (n_9b > 0)
    {
      cp2155_set (fd, 0x9b, 0x06);
      usleep (n_msec * MSEC);
      cp2155_set (fd, 0x9b, 0x04);
      usleep (n_msec * MSEC);
      n_9b--;
    }

  cp2155_set (fd, 0x90, 0xf0);	/* f8 */
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x63);
  cp2155_set (fd, 0xa3, 0xd0);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x19);
  cp2155_set (fd, 0x53, 0x5a);
  cp2155_set (fd, 0x54, 0x17);
  cp2155_set (fd, 0x55, 0x98);
  cp2155_set (fd, 0x56, 0x11);
  cp2155_set (fd, 0x57, 0xae);
  cp2155_set (fd, 0x58, 0xa9);
  cp2155_set (fd, 0x59, 0x01);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x08);

  big_write (fd, 0xa1a4, buf);
/*  big_write_film (fd, 0xf004, buf); */

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x50);
  cp2155_set (fd, 0x13, 0x50);
  cp2155_set (fd, 0x16, 0x50);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x50);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  memcpy (buf, "\x01\x03\x05\x07\x09\x0a\x0b\x0c\x0c", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 324;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_2400 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x5555;	/* was 0x7c71 */
  unsigned char value_62 = 0x0e;	/* at 0x15 ratio 1517 decimal, value_62 was 0x29 */

  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, 0x83);
/*
  unsigned int top_value = 0x3fc7;
  unsigned char value_62 = 0x15;
  ratio 777 decimal

  cp2155_set (fd, 0x01, 0x2b);
  cp2155_set (fd, 0x04, 0x08);
  cp2155_set (fd, 0x05, 0x00);
  cp2155_set (fd, 0x06, 0x00);
*/
  cp2155_set (fd, 0x90, 0xe0);

  double n_msec = 10.0;
  int n_9b = 11;
  while (n_9b > 0)
    {
      cp2155_set (fd, 0x9b, 0x06);
      usleep (n_msec * MSEC);
      cp2155_set (fd, 0x9b, 0x04);
      usleep (n_msec * MSEC);
      n_9b--;
    }

  cp2155_set (fd, 0x90, 0xf0);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x25);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x92);
  cp2155_set (fd, 0xa3, 0x10);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf1);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x47);
  cp2155_set (fd, 0x53, 0x3d);
  cp2155_set (fd, 0x54, 0x2b);
  cp2155_set (fd, 0x55, 0xd1);
  cp2155_set (fd, 0x56, 0x20);
  cp2155_set (fd, 0x57, 0x3d);
  cp2155_set (fd, 0x58, 0x13);
  cp2155_set (fd, 0x59, 0x25);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);	/* 0x29); = darker */
  cp2155_set (fd, 0x81, 0x31);	/* 0x29); */
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x08);

  big_write (fd, 0xa1a4, buf);
  big_write_film (fd, 0xf004, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x50);
  cp2155_set (fd, 0x13, 0x50);
  cp2155_set (fd, 0x16, 0x50);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x50);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, chndl->value_66);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  memcpy (buf, "\x02\x04\x04\x06\x06\x08\x08\x0a\x0a", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 324;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_4800 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x3fc7;	/* was 0x7c71 */
  unsigned char value_62 = 0x15;	/* at 0x15 ratio 1517 decimal, value_62 was 0x29 */

  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, 0x83);
/*
  unsigned int top_value = 0x3fc7;
  unsigned char value_62 = 0x15;
  ratio 777 decimal

  cp2155_set (fd, 0x01, 0x2b);
  cp2155_set (fd, 0x04, 0x08);
  cp2155_set (fd, 0x05, 0x00);
  cp2155_set (fd, 0x06, 0x00);
*/
  cp2155_set (fd, 0x90, 0xe0);

  double n_msec = 10.0;
  int n_9b = 12;
  while (n_9b > 0)
    {
      cp2155_set (fd, 0x9b, 0x06);
      usleep (n_msec * MSEC);
      cp2155_set (fd, 0x9b, 0x04);
      usleep (n_msec * MSEC);
      n_9b--;
    }

  cp2155_set (fd, 0x90, 0xf0);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x25);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x92);
  cp2155_set (fd, 0xa3, 0x10);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf1);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x47);
  cp2155_set (fd, 0x53, 0x3d);
  cp2155_set (fd, 0x54, 0x2b);
  cp2155_set (fd, 0x55, 0xd1);
  cp2155_set (fd, 0x56, 0x20);
  cp2155_set (fd, 0x57, 0x3d);
  cp2155_set (fd, 0x58, 0x13);
  cp2155_set (fd, 0x59, 0x25);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x31);	/* 0x29); = darker */
  cp2155_set (fd, 0x81, 0x31);	/* 0x29); */
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x08);

  big_write (fd, 0xa1a4, buf);
  big_write_film (fd, 0xf004, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x50);
  cp2155_set (fd, 0x13, 0x50);
  cp2155_set (fd, 0x16, 0x50);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x50);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, chndl->value_66);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  memcpy (buf, "\x02\x04\x04\x06\x06\x08\x08\x0a\x0a", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 324;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
startblob_2224_4799 (CANON_Handle * chndl, unsigned char *buf)
{

  int fd;
  fd = chndl->fd;
  size_t count;

  unsigned int top_value = 0x1400;	/* was 0x7c71 */
  unsigned char value_62 = 0x14;	/* at 0x15 ratio 1517 decimal, value_62 was 0x29 */

  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, 0x83);

/*
  unsigned int top_value = 0x3fc7;
  unsigned char value_62 = 0x15;
  ratio 777 decimal

  cp2155_set (fd, 0x01, 0x2b);
  cp2155_set (fd, 0x04, 0x08);
  cp2155_set (fd, 0x05, 0x00);
  cp2155_set (fd, 0x06, 0x00);
*/
  cp2155_set (fd, 0x90, 0xe0);

  double n_msec = 10.0;
  int n_9b = 12;
  while (n_9b > 0)
    {
      cp2155_set (fd, 0x9b, 0x06);
      usleep (n_msec * MSEC);
      cp2155_set (fd, 0x9b, 0x04);
      usleep (n_msec * MSEC);
      n_9b--;
    }

  cp2155_set (fd, 0x90, 0xf0);
  cp2155_set (fd, 0xb0, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, chndl->value_08);
  cp2155_set (fd, 0x09, chndl->value_09);
  cp2155_set (fd, 0x0a, chndl->value_0a);
  cp2155_set (fd, 0x0b, chndl->value_0b);
  cp2155_set (fd, 0xa0, 0x25);
  cp2155_set (fd, 0xa1, 0x01);
  cp2155_set (fd, 0xa2, 0x23);
  cp2155_set (fd, 0xa3, 0x10);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x90, 0xf1);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x92);
  cp2155_set (fd, 0x53, 0xa0);
  cp2155_set (fd, 0x54, 0x58);
  cp2155_set (fd, 0x55, 0x29);
  cp2155_set (fd, 0x56, 0x40);
  cp2155_set (fd, 0x57, 0x08);
  cp2155_set (fd, 0x58, 0x27);
  cp2155_set (fd, 0x59, 0xc7);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, chndl->value_51);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x08);

  big_write (fd, 0xa1a4, buf);
  big_write_film (fd, 0xf004, buf);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x83);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x11, 0x81);
  cp2155_set (fd, 0x12, 0x50);
  cp2155_set (fd, 0x13, 0x50);
  cp2155_set (fd, 0x16, 0x50);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x50);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x00);
  cp2155_set (fd, 0x1f, 0x04);
  cp2155_set (fd, 0x66, chndl->value_66);
  cp2155_set (fd, 0x67, chndl->value_67);
  cp2155_set (fd, 0x68, chndl->value_68);

  register_table_4800 (fd, 0x05, buf);
  cp2155_set (fd, 0x18, 0x02);

  count = 324;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.0, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  general_motor_2224 (fd);

}

void
send_start_blob (CANON_Handle * chndl)
{
  unsigned char buf[0xfff0];

  int fd;
  fd = chndl->fd;

/* value_51: lamp colors
   bit 0 set: red on, bit 1 set: green on, bit 2 set: blue on,
   bit 3 set: infrared on
   all bits off: no scan is made
*/
  chndl->value_51 = 0x07;
  chndl->value_66 = 0x00;

  switch (chndl->val[opt_resolution].w)
    {
    case 75:
      chndl->value_67 = 0x0a;	/* 3*7300/8 */
      chndl->value_68 = 0xb1;
      break;
    case 150:
      chndl->value_67 = 0x15;	/* 3*7300/4 */
      chndl->value_68 = 0x63;
      break;
    case 300:
      chndl->value_67 = 0x2a;	/* 3*7300/2 */
      chndl->value_68 = 0xc6;
      break;
    case 600:
      chndl->value_67 = 0x55;	/* 3*7300 */
      chndl->value_68 = 0x8c;
      break;
    case 1200:
      chndl->value_67 = 0xab;	/* 6*7300 */
      chndl->value_68 = 0x18;
      break;
    case 2400:
      chndl->value_66 = 0x01;
      chndl->value_67 = 0x56;	/* 12*7300 */
      chndl->value_68 = 0x30;
      break;
    case 4800:
      chndl->value_66 = 0x02;
      chndl->value_67 = 0xac;	/* 24*7300 */
      chndl->value_68 = 0x60;
    }

  unsigned char value_11 = 0xc1;	/* 0x00; */

  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, value_11);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, value_11);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x11, value_11);
  cp2155_set (fd, 0x01, 0x29);
  cp2155_set (fd, 0x04, 0x0c);
  cp2155_set (fd, 0x05, 0x00);
  cp2155_set (fd, 0x06, 0x00);
  cp2155_set (fd, 0x01, 0x29);
  cp2155_set_gamma (fd, chndl);

  switch (chndl->val[opt_resolution].w)
    {
    case 75:
      if (chndl->productcode == 0x2225)
	{
	  startblob_2225_0075 (chndl, buf);
	}
      else
	{
	  startblob_2224_0075 (chndl, buf);
	}
      break;
    case 150:
      if (chndl->productcode == 0x2225)
	{
	  startblob_2225_0150 (chndl, buf);
	}
      else
	{
	  startblob_2224_0150 (chndl, buf);
	}
      break;
    case 300:
      if (chndl->productcode == 0x2225)
	{
	  startblob_2225_0300 (chndl, buf);
	}
      else
	{
	  cp2155_set_gamma_red_enhanced (fd, chndl);
	  startblob_2224_0300 (chndl, buf);
	}
      break;
    case 600:
      if (chndl->productcode == 0x2225)
	{
	  cp2155_set_gamma_red_enhanced (fd, chndl);
	  startblob_2225_0600 (chndl, buf);
/*
          startblob_2225_0600_extra (chndl, buf);
*/
	}
      else
	{
	  startblob_2224_0600 (chndl, buf);
	}
      break;
    case 1200:
      if (chndl->productcode == 0x2225)
	{
	  startblob_2225_1200 (chndl, buf);
	}
      else
	{
	  startblob_2224_1200 (chndl, buf);
	}
      break;
    case 2400:
      if (chndl->productcode == 0x2225)
	{
	  startblob_2225_1200 (chndl, buf);
	}
      else
	{
	  startblob_2224_2400 (chndl, buf);
	}
      break;
    case 4800:
      if (chndl->productcode == 0x2225)
	{
	  startblob_2225_1200 (chndl, buf);
	}
      else
	{
	  startblob_2224_4800 (chndl, buf);
	}
      break;
    }
}

/* Wait until data ready */
static long
wait_for_data (CANON_Handle * chndl)
{
  int fd;
  fd = chndl->fd;
  time_t start_time = time (NULL);
  long size;
  byte value;

  DBG (12, "waiting...\n");

  while (1)
    {
      size = 0;
      cp2155_get (fd, 0x46, &value);
      DBG (1, "home sensor: %02x\n", value);
      if (value == 0)
	{
	  send_start_blob (chndl);
	  cp2155_get (fd, 0x46, &value);
	  DBG (1, "home sensor: %02x\n", value);
	}

      if (cp2155_get (fd, 0xa5, &value) != SANE_STATUS_GOOD)
	{
	  return -1;
	}

      size += value;

      if (cp2155_get (fd, 0xa6, &value) != SANE_STATUS_GOOD)
	{
	  return -1;
	}

      size <<= 8;
      size += value;

      if (cp2155_get (fd, 0xa7, &value) != SANE_STATUS_GOOD)
	{
	  return -1;
	}

      size <<= 8;
      size += value;

      if (size != 0)
	{
	  return 2 * size;
	}

      /* Give it 5 seconds */
      if ((time (NULL) - start_time) > 5)
	{
	  DBG (1, "wait_for_data: timed out (%ld)\n", size);
	  return -1;
	}

      usleep (1 * MSEC);
    }
}

static int
init_2225 (CANON_Handle * chndl)
{
  int fd = chndl->fd;
  byte value;
  int result = 0;

  cp2155_get (fd, 0xd0, &value);
  /* Detect if scanner is plugged in */
  if (value != 0x81 && value != 0x40)
    {
      DBG (1, "INIT: unexpected value: %x\n", value);
    }

  if (value == 0x00)
    {
      return -1;
    }

  cp2155_set (fd, 0x02, 0x01);
  cp2155_set (fd, 0x02, 0x00);
  cp2155_set (fd, 0x01, 0x00);
  cp2155_set (fd, 0x01, 0x28);
  cp2155_set (fd, 0x90, 0x4f);
  cp2155_set (fd, 0x92, 0xff);
  cp2155_set (fd, 0x93, 0x00);
  cp2155_set (fd, 0x91, 0x1f);
  cp2155_set (fd, 0x95, 0x1f);
  cp2155_set (fd, 0x97, 0x1f);
  cp2155_set (fd, 0x9b, 0x00);
  cp2155_set (fd, 0x9c, 0x07);
  cp2155_set (fd, 0x90, 0x4d);
  cp2155_set (fd, 0x90, 0xcd);
  cp2155_set (fd, 0x90, 0xcc);
  cp2155_set (fd, 0x9b, 0x01);
  cp2155_set (fd, 0xa0, 0x04);
  cp2155_set (fd, 0xa0, 0x05);
  cp2155_set (fd, 0x01, 0x28);
  cp2155_set (fd, 0x04, 0x0c);
  cp2155_set (fd, 0x05, 0x00);
  cp2155_set (fd, 0x06, 0x00);
  cp2155_set (fd, 0x98, 0x00);
  cp2155_set (fd, 0x98, 0x00);
  cp2155_set (fd, 0x98, 0x02);
  cp2155_set (fd, 0x99, 0x28);
  cp2155_set (fd, 0x9a, 0x03);
  cp2155_set (fd, 0x80, 0x10);
  cp2155_set (fd, 0x8d, 0x00);
  cp2155_set (fd, 0x8d, 0x04);

  cp2155_set (fd, 0x85, 0x00);
  cp2155_set (fd, 0x87, 0x00);
  cp2155_set (fd, 0x88, 0x70);

  /* Registers 0x85 (address), 0x87 (data bit 8) and 0x88 (data bits 7-0)
     write to the HT82V36 analog front end. Its register 3 is the PGA gain
     code; calibrated scans use a lower gain (see make_shading). */
  cp2155_set (fd, 0x85, 0x03);
  cp2155_set (fd, 0x87, 0x00);
  cp2155_set (fd, 0x88, chndl->afe_gain ? chndl->afe_gain : 0x28);

  cp2155_set (fd, 0x85, 0x06);
  cp2155_set (fd, 0x87, 0x00);
  cp2155_set (fd, 0x88, 0x28);


  DBG (1, "INIT state: %0d\n", result);
  return result;
}

static int
init_2224 (CANON_Handle * chndl)
{
  int fd = chndl->fd;
  byte value;
  int result = 0;

  cp2155_get (fd, 0xd0, &value);
  /* Detect if scanner is plugged in */
  if (value != 0x81 && value != 0x40)
    {
      DBG (1, "INIT: unexpected value: %x\n", value);
    }

  if (value == 0x00)
    {
      return -1;
    }

  cp2155_set (fd, 0x02, 0x01);
  cp2155_set (fd, 0x02, 0x00);
  cp2155_set (fd, 0x01, 0x00);
  cp2155_set (fd, 0x01, 0x28);
  cp2155_set (fd, 0xa0, 0x04);
  cp2155_set (fd, 0xa0, 0x05);
  cp2155_set (fd, 0x01, 0x28);
  cp2155_set (fd, 0x04, 0x0c);
  cp2155_set (fd, 0x05, 0x00);
  cp2155_set (fd, 0x06, 0x00);
  cp2155_set (fd, 0x90, 0x27);
  cp2155_set (fd, 0x92, 0xf7);
  cp2155_set (fd, 0x94, 0xf7);
  cp2155_set (fd, 0x93, 0x00);
  cp2155_set (fd, 0x91, 0x1f);
  cp2155_set (fd, 0x95, 0x0f);
  cp2155_set (fd, 0x97, 0x0f);
  cp2155_set (fd, 0x9b, 0x00);
  cp2155_set (fd, 0x9c, 0x07);
  cp2155_set (fd, 0x90, 0xf0);
  cp2155_set (fd, 0x9b, 0x04);
  cp2155_set (fd, 0x98, 0x00);
  cp2155_set (fd, 0x98, 0x00);
  cp2155_set (fd, 0x98, 0x02);
  cp2155_set (fd, 0x99, 0x3b);
  cp2155_set (fd, 0x9a, 0x03);
  cp2155_set (fd, 0x80, 0x10);
  cp2155_set (fd, 0x8d, 0x00);
  cp2155_set (fd, 0x8d, 0x04);

  DBG (1, "INIT state: %0d\n", result);

  return result;
}

static int
init (CANON_Handle * chndl)
{
  int result;
  if (chndl->productcode == 0x2225)
    {
      chndl->table_gamma = 2.2;
      chndl->table_gamma_blue = 2.2;
      chndl->highlight_red_enhanced = 190;
      chndl->highlight_other = 240;
      chndl->highlight_blue_reduced = 240;
      result = init_2225 (chndl);
    }
  else
    {
      chndl->table_gamma = 2.2;
      chndl->table_gamma_blue = 1.95;
      chndl->highlight_red_enhanced = 190;
      chndl->highlight_other = 215;
      chndl->highlight_blue_reduced = 255;
      result = init_2224 (chndl);
    }
  return result;
}

void
back2225 (int fd, unsigned char *buf)
{
  size_t count;
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;
  ratio 320 decimal
*/
  cp2155_set (fd, 0x90, 0xc8);
  cp2155_set (fd, 0x90, 0xc8);
  cp2155_set (fd, 0xb0, 0x03);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, 0x00);
  cp2155_set (fd, 0x09, 0x69);
  cp2155_set (fd, 0x0a, 0x00);
  cp2155_set (fd, 0x0b, 0xe8);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x00);
  cp2155_set (fd, 0xa3, 0x70);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, 0x07);
  cp2155_set (fd, 0x5a, 0x32);
  cp2155_set (fd, 0x5b, 0x32);
  cp2155_set (fd, 0x5c, 0x32);
  cp2155_set (fd, 0x5d, 0x32);
  cp2155_set (fd, 0x52, 0x00);
  cp2155_set (fd, 0x53, 0x01);
  cp2155_set (fd, 0x54, 0x00);
  cp2155_set (fd, 0x55, 0x01);
  cp2155_set (fd, 0x56, 0x00);
  cp2155_set (fd, 0x57, 0x01);
  cp2155_set (fd, 0x58, 0x00);
  cp2155_set (fd, 0x59, 0x01);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, 0x07);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x81, 0x29);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x82, 0x09);
  cp2155_set (fd, 0x83, 0x02);
  cp2155_set (fd, 0x84, 0x06);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x03);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x9b, 0x03);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x41);
  cp2155_set (fd, 0x11, 0x61);
  cp2155_set (fd, 0x11, 0x21);
  cp2155_set (fd, 0x11, 0x21);
  cp2155_set (fd, 0x11, 0x25);
  cp2155_set (fd, 0x11, 0x25);
  cp2155_set (fd, 0x11, 0x25);
  cp2155_set (fd, 0x12, 0x40);
  cp2155_set (fd, 0x13, 0x40);
  cp2155_set (fd, 0x16, 0x40);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x40);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x36);
  cp2155_set (fd, 0x1f, 0xd0);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, 0x00);
  cp2155_set (fd, 0x68, 0x06);

  memcpy (buf, "\x28\x27\x25\x21\x1c\x16\x0f\x08\x00", 9);
  register_table (fd, 0, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 260;
  make_slope_table (count, top_value, 0x6a, 0.021739, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.15217, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x35);
  cp2155_set (fd, 0x60, 0x15);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x03, 0x01);

}

void
back2224 (int fd, unsigned char *buf)
{
  size_t count;

  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;

/* original:
  unsigned int top_value = 0x2580;
  unsigned char value_62 = 0x2e;
  ratio 320 decimal
*/
  cp2155_set (fd, 0x90, 0xe8);
  cp2155_set (fd, 0x9b, 0x06);
  cp2155_set (fd, 0x9b, 0x04);
  cp2155_set (fd, 0x90, 0xf8);
  cp2155_set (fd, 0xb0, 0x03);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x07, 0x00);
  cp2155_set (fd, 0x08, 0x01);
  cp2155_set (fd, 0x09, 0xb3);
  cp2155_set (fd, 0x0a, 0x02);
  cp2155_set (fd, 0x0b, 0x32);
  cp2155_set (fd, 0xa0, 0x1d);
  cp2155_set (fd, 0xa1, 0x00);
  cp2155_set (fd, 0xa2, 0x00);
  cp2155_set (fd, 0xa3, 0x70);
  cp2155_set (fd, 0x64, 0x00);
  cp2155_set (fd, 0x65, 0x00);
  cp2155_set (fd, 0x61, 0x00);
  cp2155_set (fd, 0x62, value_62);
  cp2155_set (fd, 0x63, 0x00);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x50, 0x04);
/*  cp2155_set (fd, 0x90, 0xf8); */
  cp2155_set (fd, 0x51, 0x07);
  cp2155_set (fd, 0x5a, 0xff);
  cp2155_set (fd, 0x5b, 0xff);
  cp2155_set (fd, 0x5c, 0xff);
  cp2155_set (fd, 0x5d, 0xff);
  cp2155_set (fd, 0x52, 0x00);
  cp2155_set (fd, 0x53, 0x01);
  cp2155_set (fd, 0x54, 0x00);
  cp2155_set (fd, 0x55, 0x01);
  cp2155_set (fd, 0x56, 0x00);
  cp2155_set (fd, 0x57, 0x01);
  cp2155_set (fd, 0x58, 0x00);
  cp2155_set (fd, 0x59, 0x01);
  cp2155_set (fd, 0x5e, 0x02);
  cp2155_set (fd, 0x5f, 0x00);
  cp2155_set (fd, 0x5f, 0x03);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x50, 0x04);
  cp2155_set (fd, 0x51, 0x07);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x81, 0x31);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x82, 0x11);
  cp2155_set (fd, 0x83, 0x01);
  cp2155_set (fd, 0x84, 0x05);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0xb0, 0x03);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x41);
  cp2155_set (fd, 0x11, 0x61);
  cp2155_set (fd, 0x11, 0x21);
  cp2155_set (fd, 0x11, 0x21);
  cp2155_set (fd, 0x11, 0x25);
  cp2155_set (fd, 0x11, 0x25);
  cp2155_set (fd, 0x11, 0x25);
  cp2155_set (fd, 0x12, 0x7d);
  cp2155_set (fd, 0x13, 0x7d);
  cp2155_set (fd, 0x16, 0x7d);
  cp2155_set (fd, 0x21, 0x06);
  cp2155_set (fd, 0x22, 0x7d);
  cp2155_set (fd, 0x20, 0x06);
  cp2155_set (fd, 0x1d, 0x00);
  cp2155_set (fd, 0x1e, 0x36);
  cp2155_set (fd, 0x1f, 0xd0);
  cp2155_set (fd, 0x66, 0x00);
  cp2155_set (fd, 0x67, 0x00);
  cp2155_set (fd, 0x68, 0x06);

  register_table (fd, 0x0d, buf);
  cp2155_set (fd, 0x18, 0x00);

  count = 516;
  make_slope_table (count, top_value, 0x06, 0.0067225, buf);

  write_buf (fd, count, buf, 0x03, 0x00);
  write_buf (fd, count, buf, 0x03, 0x02);
  write_buf (fd, count, buf, 0x03, 0x06);

  count = 36;
  make_slope_table (count, top_value, 0x06, 0.15217, buf);

  write_buf (fd, count, buf, 0x03, 0x04);
  write_buf (fd, count, buf, 0x03, 0x08);

  cp2155_set (fd, 0x10, 0x05);
  cp2155_set (fd, 0x11, 0x35);
  cp2155_set (fd, 0x60, 0x01);
  cp2155_set (fd, 0x80, 0x12);
  cp2155_set (fd, 0x03, 0x01);

}

/* Shading calibration (LiDE 70).

   Each column of the contact image sensor has its own sensitivity and the
   three colors differ: uncorrected, a white page shows vertical bands of
   about 5% (up to 12%) and a red or pink tint. The scanner has a white strip
   just before the glass, from about 5.5 to 2 mm before the top of the scan
   area at 300 dpi and up, inside the 7 mm top_edge of
   CANON_set_scan_parameters. Before a calibrated scan, a short linear scan of
   the strip (strip_band) gives, per sensor column and color, how bright
   white reads. The strip is a little brighter than white on the glass, so
   the reading is scaled by fixed glass/strip factors per color, measured once
   against a white surface on the glass. The scan itself is made with
   identity gamma tables; each column is scaled so that white becomes 255,
   and the gamma is applied afterwards (apply_shading).

   The analog front end's gain (HT82V36 PGA, register 3; gain = 5.85 / (1 +
   4.85 * (63 - code) / 63)) is lowered from 0x28 (about 2.11x) for the strip
   and the scan, as white paper and the strip saturate in many red and blue
   columns at 0x28. The scanner's response has been seen to change by a few
   percent between sessions, so the strip is read again for a scan unless
   the last reading at the same resolution is recent. */

#define STRIP_READING_LIFETIME 600	/* seconds a strip reading is reused */
#define STRIP_MIN_WHITE 48.0	/* darkest white trusted (edges of the bed) */
#define STRIP_MAX_SATURATED 0.15	/* largest fraction of saturated readings */
#define STRIP_LEVEL_MIN 0.6	/* plausible strip level, relative to the */
#define STRIP_LEVEL_MAX 1.4	/* level measured with the factors */
#define SHADING_GAMMA 2.2

typedef struct
{
  int dpi;			/* scan resolution */
  int afe_gain;			/* gain code for the strip reading and the scan */
  float factor[3];		/* white on the glass / strip; red, green, blue */
  float level[3];		/* mean strip reading when the factors were measured */
}
Strip_Calibration;

/* Measured on one LiDE 70, with its white lid closed on the clear glass. The
   600 dpi setup reads red weakly and blue strongly, hence its lower gain. */
static const Strip_Calibration strip_calibrations[] = {
  {75, 32, {0.8876, 0.9076, 0.9101}, {206.3, 179.8, 201.1}},
  {150, 32, {0.8834, 0.9020, 0.9039}, {206.3, 179.8, 201.1}},
  {300, 32, {0.8860, 0.9054, 0.9080}, {206.3, 179.8, 201.1}},
  {600, 26, {0.9073, 0.9247, 0.9278}, {126.6, 167.3, 207.5}},
  {1200, 32, {0.8762, 0.8954, 0.8965}, {201.5, 188.6, 212.4}},
};

/* The calibration for a scan at dpi, or NULL if there is none. */
static const Strip_Calibration *
strip_calibration_for (CANON_Handle * chndl, int dpi)
{
  size_t i;

  if (chndl->productcode != 0x2225)
    {
      return NULL;
    }
  for (i = 0; i < sizeof (strip_calibrations) / sizeof (strip_calibrations[0]);
       i++)
    {
      if (strip_calibrations[i].dpi == dpi)
	{
	  return &strip_calibrations[i];
	}
    }
  return NULL;
}

/* Resolution of the strip reading for a scan at dpi: below 300 dpi the scan
   area starts on the glass, so the strip is read at 300 dpi. */
static int
strip_dpi_for (int dpi)
{
  return dpi < 300 ? 300 : dpi;
}

/* The band of the strip that is read, in mm from the top of the scan area.
   The slider moves slowly at 1200 dpi, so that band is narrower. The factors
   above were measured with these bands. */
static void
strip_band (int strip_dpi, double *top, double *bottom)
{
  if (strip_dpi >= 1200)
    {
      *top = -4.5;
      *bottom = -3.3;
    }
  else
    {
      *top = -5.0;
      *bottom = -2.5;
    }
}

static int
compare_bytes (const void *a, const void *b)
{
  return *(const unsigned char *) a - *(const unsigned char *) b;
}

/* Keeps the per-column medians of a strip scan (rows of interleaved RGB, 3
   bytes per column) as chndl->strip, so that dust does not count. */
static SANE_Status
store_strip_reading (CANON_Handle * chndl, const unsigned char *data,
		     int columns, int rows, int strip_dpi, int afe_gain)
{
  float *strip;
  unsigned char *samples;
  int x, c, r;

  strip = malloc (3 * columns * sizeof (float));
  samples = malloc (rows);
  if (!strip || !samples)
    {
      free (strip);
      free (samples);
      return SANE_STATUS_NO_MEM;
    }

  for (c = 0; c < 3; c++)
    {
      for (x = 0; x < columns; x++)
	{
	  for (r = 0; r < rows; r++)
	    {
	      samples[r] = data[(long) r * 3 * columns + 3 * x + c];
	    }
	  qsort (samples, rows, 1, compare_bytes);
	  strip[c * columns + x] = samples[rows / 2];
	}
    }
  free (samples);

  free (chndl->strip);
  chndl->strip = strip;
  chndl->strip_columns = columns;
  chndl->strip_dpi = strip_dpi;
  chndl->strip_gain = afe_gain;
  chndl->strip_time = time (NULL);
  return SANE_STATUS_GOOD;
}

/* A stored strip reading that can be used for a scan at dpi. */
static SANE_Bool
strip_reading_is_current (CANON_Handle * chndl,
			  const Strip_Calibration * cal, int dpi)
{
  return chndl->strip && chndl->strip_dpi == strip_dpi_for (dpi)
    && chndl->strip_gain == cal->afe_gain
    && time (NULL) - chndl->strip_time < STRIP_READING_LIFETIME;
}

/* Saturated readings (255) only say "at least this bright": interpolate
   between the nearest unsaturated readings on either side. */
static void
fill_saturated (float *v, int n)
{
  int x = 0, end, i;
  float left, right;

  while (x < n)
    {
      if (v[x] < 255)
	{
	  x++;
	  continue;
	}
      end = x;
      while (end < n && v[end] >= 255)
	{
	  end++;
	}
      left = x > 0 ? v[x - 1] : -1;
      right = end < n ? v[end] : -1;
      for (i = x; i < end; i++)
	{
	  if (left >= 0 && right >= 0)
	    {
	      v[i] = left + (right - left) * (i - x + 1) / (end - x + 1);
	    }
	  else if (left >= 0)
	    {
	      v[i] = left;
	    }
	  else if (right >= 0)
	    {
	      v[i] = right;
	    }
	}
      x = end;
    }
}

/* Builds the shading correction for the scan set up in chndl (resolution,
   x1, width) from the stored strip reading. Fails, with the reason in the
   debug output, if the reading does not look like the strip; the scan is
   then made uncalibrated. */
static SANE_Status
make_shading (CANON_Handle * chndl, const Strip_Calibration * cal)
{
  int dpi = chndl->val[opt_resolution].w;
  int n = chndl->strip_columns;
  int step = chndl->strip_dpi / dpi;
  int columns, first, width, x, c, i, saturated, filled = 0;
  float *v, level[3];
  double sum;
  unsigned short *shading;

  if (step < 1)
    {
      step = 1;
    }
  if (!chndl->strip || n <= 10)
    {
      DBG (1, "make_shading: no strip reading\n");
      return SANE_STATUS_INVAL;
    }
  v = malloc (3 * n * sizeof (float));
  width = chndl->params.pixels_per_line;	/* chndl->width once scanning */
  shading = malloc (3 * width * sizeof (unsigned short));
  if (!v || !shading)
    {
      free (v);
      free (shading);
      return SANE_STATUS_NO_MEM;
    }
  memcpy (v, chndl->strip, 3 * n * sizeof (float));

  for (c = 0; c < 3; c++)
    {
      float *col = v + c * n;
      saturated = 0;
      for (x = 0; x < n; x++)
	{
	  if (col[x] >= 255)
	    {
	      saturated++;
	    }
	}
      if (saturated > STRIP_MAX_SATURATED * n)
	{
	  DBG (1, "make_shading: %d of %d strip readings saturated\n",
	       saturated, n);
	  free (v);
	  free (shading);
	  return SANE_STATUS_INVAL;
	}
      filled += saturated;
      fill_saturated (col, n);

      sum = 0;
      for (x = n / 50; x < n - n / 50; x++)
	{
	  sum += col[x];
	}
      level[c] = sum / (n - 2 * (n / 50)) / cal->level[c];
      if (level[c] < STRIP_LEVEL_MIN || level[c] > STRIP_LEVEL_MAX)
	{
	  DBG (1, "make_shading: strip level %.2f (color %d) is implausible\n",
	       level[c], c);
	  free (v);
	  free (shading);
	  return SANE_STATUS_INVAL;
	}
    }
  DBG (1, "make_shading: strip level %.3f/%.3f/%.3f of the calibrated "
       "scanner's, %d saturated readings filled in\n",
       level[0], level[1], level[2], filled);

  /* Column x of the scan is sensor column first + x at the scan resolution;
     a column at a lower resolution than the strip's averages step columns
     of the strip. */
  columns = (n + step - 1) / step;
  first = (chndl->x1 * dpi + 300) / 600;
  for (x = 0; x < width; x++)
    {
      int column = first + x;
      int start, end;

      if (column > columns - 1)
	{
	  column = columns - 1;
	}
      start = column * step;
      end = start + step < n ? start + step : n;
      for (c = 0; c < 3; c++)
	{
	  double white = 0;
	  for (i = start; i < end; i++)
	    {
	      white += v[c * n + i];
	    }
	  white = white / (end - start) * cal->factor[c];
	  if (white < STRIP_MIN_WHITE)
	    {
	      white = STRIP_MIN_WHITE;
	    }
	  shading[3 * x + c] = (unsigned short) (255.0 / white * 256 + 0.5);
	}
    }
  free (v);

  for (i = 0; i < 256; i++)
    {
      chndl->shading_gamma[i] =
	(unsigned char) (255 * pow (i / 255.0, 1 / SHADING_GAMMA) + 0.5);
    }
  free (chndl->shading);
  chndl->shading = shading;
  return SANE_STATUS_GOOD;
}

/* Corrects one line of interleaved RGB from a linear scan in place. */
static void
apply_shading (CANON_Handle * chndl, unsigned char *line)
{
  long i, n = 3 * chndl->width;
  unsigned int v;

  for (i = 0; i < n; i++)
    {
      v = (line[i] * (unsigned int) chndl->shading[i]) >> 8;
      line[i] = chndl->shading_gamma[v > 255 ? 255 : v];
    }
}

/* Drops the shading correction: the next scan is uncalibrated unless it
   builds a new one. */
static void
clear_shading (CANON_Handle * chndl)
{
  free (chndl->shading);
  chndl->shading = NULL;
  chndl->linear = SANE_FALSE;
  chndl->afe_gain = 0;
}

static void
go_home_without_wait (CANON_Handle * chndl)
{
  unsigned char buf[0x400];
  int fd = chndl->fd;
  byte value;
  cp2155_get (fd, 0x46, &value);
  if (value == 0x08)
    {
      return;
    }

  DBG (1, "go_home_without_wait: product code: %x\n", chndl->productcode);
  if (chndl->productcode == 0x2225)
    {
      back2225 (fd, buf);
    }
  else
    {
      back2224 (fd, buf);
    }
}


/* Set by sane_cancel(), which may be called from a signal handler or from
   another thread while sane_start() is scanning; cleared by sane_start(). */
static volatile sig_atomic_t cancel_requested = 0;

/* Longest wait for the slider to reach its home position. A return from the
   far end of the bed takes a few seconds. */
#define GO_HOME_TIMEOUT 30	/* seconds */

/* Returns 0 when the slider is home, -1 if the scanner does not answer, the
   slider does not get home in time, or the scan is cancelled. */
static int
go_home (CANON_Handle * chndl)
{
  int fd = chndl->fd;
  byte value;
  time_t start_time;

  if (cp2155_get (fd, 0x46, &value) != SANE_STATUS_GOOD)
    {
      return -1;
    }
  DBG (1, "state sensor: %02x\n", value);
  if (value == 0x08)
    {
      return 0;
    }

  go_home_without_wait (chndl);
  start_time = time (NULL);

  while (1)
    {
      usleep (200 * MSEC);
      if (cp2155_get (fd, 0x46, &value) != SANE_STATUS_GOOD)
	{
	  return -1;
	}
      DBG (1, "state sensor: %02x\n", value);

      if (value == 0x08)
	{
	  break;
	}

      if (cancel_requested)
	{
	  DBG (1, "go_home: cancelled\n");
	  return -1;
	}

      if (time (NULL) - start_time > GO_HOME_TIMEOUT)
	{
	  DBG (1, "go_home: not home after %d seconds (state sensor: %02x)\n",
	       GO_HOME_TIMEOUT, value);
	  return -1;
	}
    }
  return 0;
}

/* Scan and save the resulting image as r,g,b non-interleaved PPM file */
static SANE_Status
preread (CANON_Handle * chndl, SANE_Byte * data, FILE * fp)
{
  SANE_Status status = SANE_STATUS_GOOD;

  static byte linebuf[0x40000];
  byte readbuf[0xf000];
  int fd = chndl->fd;
  long width = chndl->params.pixels_per_line;
  /* set width to next multiple of 0x10 */
  while ((width % 0x10) != 0xf)
    {
      width++;
    }

  width++;

  byte *srcptr = readbuf;
  static byte *dstptr = linebuf;
  byte *endptr = linebuf + 3 * width;	/* Red line + Green line + Blue line */
  long datasize = 0;
  static long line = 0;
  size_t offset = 0;
  size_t bytes_written;
  static byte slot = 0;

  /* Data coming back is "width" bytes Red data, width bytes Green,
     width bytes Blue, repeat for "height" lines. */
/*  while (line < height)  process one buffer from the scanner */
  long startline = line;

  /* stop between two buffers from the scanner; do_scan then sends the
     slider home */
  if (cancel_requested)
    {
      DBG (1, "scan cancelled at line %ld\n", line);
      init (chndl);
      status = SANE_STATUS_CANCELLED;
      goto stop;
    }

  if (line >= (chndl->y1) * chndl->val[opt_resolution].w / 600
      + chndl->params.lines)
    {
      status = SANE_STATUS_EOF;
      init (chndl);
      line = 0;
      slot = 0;
      dstptr = linebuf;
      return status;
    }
  datasize = wait_for_data (chndl);

  /* The end of the image is detected above, so no data here means that the
     scanner stopped answering (or was disconnected) partway through. */
  if (datasize < 0)
    {
      DBG (1, "no data at line %ld\n", line);
      status = SANE_STATUS_IO_ERROR;
      goto stop;
    }

  if (datasize > 0xf000)
    {
      datasize = 0xf000;
    }

  DBG (12, "scan line %ld %ld\n", line, datasize);

  cp2155_set (fd, 0x72, (datasize >> 8) & 0xff);
  cp2155_set (fd, 0x73, (datasize) & 0xff);

  status = cp2155_read (fd, readbuf, datasize);

  if (status != SANE_STATUS_GOOD)
    {
      DBG (1, "read error at line %ld\n", line);
      status = SANE_STATUS_IO_ERROR;
      goto stop;
    }

  /* Contorsions to convert data from line-by-line RGB to byte-by-byte RGB,
     without reading in the whole buffer first.  One image line is
     constructed in buffer linebuf and written to temp file if complete. */
  int idx = 0;
  srcptr = readbuf;

  while (idx < datasize)
    {
      *dstptr = (byte) * srcptr;
      idx++;
      srcptr += 1;
      dstptr += 3;

      if (dstptr >= endptr)	/* line of one color complete */
	{
	  slot++;		/* next color for this line */
	  dstptr = linebuf + slot;	/* restart shortly after beginning */
	  if (slot == 3)	/* all colors done */
	    {
	      slot = 0;		/* back to first color */
	      dstptr = linebuf;	/* back to beginning of line */
	      line++;		/* number of line just completed */
	      /* use scanner->width instead of width to remove pad bytes */
	      if (line > (chndl->y1) * chndl->val[opt_resolution].w / 600)
		{
		  if (chndl->shading)
		    {
		      apply_shading (chndl, linebuf);
		    }
		  if (chndl->params.format == SANE_FRAME_RGB)
		    {
		      memcpy (data + offset, linebuf, 3 * chndl->width);
		      offset += 3 * chndl->width;
		    }
		  else
		    {
		      int grayvalue;
		      int lineelement = 0;
		      while (lineelement < chndl->width)
			{
			  grayvalue = linebuf[3 * lineelement] +
			    linebuf[3 * lineelement + 1] +
			    linebuf[3 * lineelement + 2];
			  grayvalue /= 3;
			  if (chndl->params.depth == 8)	/* gray */
			    {
			      data[offset + lineelement] = (byte) grayvalue;
			    }
			  else	/* lineart */
			    {
			      if (lineelement % 8 == 0)
				{
				  data[offset + (lineelement >> 3)] = 0;
				}
			      if ((byte) grayvalue <
				  chndl->absolute_threshold)
				{
				  data[offset + (lineelement >> 3)] |=
				    (1 << (7 - lineelement % 8));
				}
			    }
			  lineelement++;
			}
		      offset += chndl->params.bytes_per_line;
		    }
		  DBG (6, "line %ld written...\n", line);
		}

	      if (line == (chndl->y1) * chndl->val[opt_resolution].w / 600
		  + chndl->params.lines)
		{
		  break;
		}

	    }
	}
    }				/* one readbuf processed */
  bytes_written = fwrite (data, 1, offset, fp);
  DBG (6, "%ld bytes written\n", bytes_written);
  if (bytes_written != offset)
    {
      status = SANE_STATUS_IO_ERROR;
    }
  DBG (6, "%ld lines from readbuf\n", line - startline);
  if (status == SANE_STATUS_GOOD)
    {
      return status;		/*  to escape from this loop
				   after processing only one data buffer */
    }

stop:
  /* the scan ends here: start the next one from the first line */
  line = 0;
  slot = 0;
  dstptr = linebuf;
  return status;
}

/* Scan and save the resulting image as r,g,b non-interleaved PPM file */
static SANE_Status
do_scan (CANON_Handle * chndl)
{
  SANE_Status status = SANE_STATUS_GOOD;
  SANE_Byte outbuf[0x40000];
  FILE *fp;
  fp = fopen (chndl->fname, "w");
  if (!fp)
    {
      DBG (1, "err:%s when opening %s\n", strerror (errno), chndl->fname);
      return SANE_STATUS_IO_ERROR;
    }
  long width = chndl->params.pixels_per_line;
  if (chndl->val[opt_resolution].w < 600)
    {
      width = width * 600 / chndl->val[opt_resolution].w;
    }
  /* set width to next multiple of 0x10 */
  while ((width % 0x10) != 0xf)
    {
      width++;
    }

  long x_start;
  long x_end;
  long left_edge = 0x69;
  switch (chndl->val[opt_resolution].w)
    {
    case 75:
    case 150:
    case 300:
    case 600:
      if (chndl->productcode == 0x2224)
	{
	  left_edge = 0x1b3;
	}
      else
	{
	  left_edge = 0x69;
	}
      break;
    case 1200:
      if (chndl->productcode == 0x2224)
	{
	  left_edge = 0x1b2;
	}
      else
	{
	  left_edge = 0x87;
	}
      break;
    case 2400:
      if (chndl->productcode == 0x2224)
	{
	  left_edge = 0x287;	/* 0x2eb; */
	}
      else
	{
	  left_edge = 0x87;
	}
      break;
    case 4800:
      if (chndl->productcode == 0x2224)
	{
	  left_edge = 0x2e3;	/* should be adjusted; 0x23e; 0x2eb; */
	}
      else
	{
	  left_edge = 0x87;
	}
    }
  x_start = left_edge + chndl->x1 * chndl->val[opt_resolution].w / 600;
  if (chndl->val[opt_resolution].w < 600)
    {
      x_start = left_edge + chndl->x1;
    }
  x_end = x_start + (width);
  width++;

  chndl->value_08 = (x_start >> 8) & 0xff;
  chndl->value_09 = (x_start) & 0xff;
  chndl->value_0a = (x_end >> 8) & 0xff;
  chndl->value_0b = (x_end) & 0xff;

  DBG (3, "val_08: %02x\n", chndl->value_08);
  DBG (3, "val_09: %02x\n", chndl->value_09);
  DBG (3, "val_0a: %02x\n", chndl->value_0a);
  DBG (3, "val_0b: %02x\n", chndl->value_0b);
  DBG (3, "chndl->width: %04lx\n", chndl->width);

  send_start_blob (chndl);

  while (status == SANE_STATUS_GOOD)
    {
      status = preread (chndl, outbuf, fp);
    }
  go_home_without_wait (chndl);

  if (status == SANE_STATUS_EOF)
    {
      status = SANE_STATUS_GOOD;
    }

  fclose (fp);
  DBG (6, "created scan file %s\n", chndl->fname);

  return status;
}

/* Scan sequence */
/* resolution is 75,150,300,600,1200,2400,4800
   scan coordinates in 600-dpi pixels */

static SANE_Status
scan (CANON_Handle * chndl)
{
  SANE_Status status = SANE_STATUS_GOOD;
  /* Resolution: dpi 75, 150, 300, 600, 1200, 2400, 4800 */
  switch (chndl->val[opt_resolution].w)
    {
    case 75:
    case 150:
    case 300:
    case 600:
    case 1200:
    case 2400:
    case 4800:
      break;
    default:
      chndl->val[opt_resolution].w = 600;
    }

  chndl->width = chndl->params.pixels_per_line;
  chndl->height =
    (chndl->y2 - chndl->y1) * chndl->val[opt_resolution].w / 600;
  DBG (1, "dpi=%d\n", chndl->val[opt_resolution].w);
  DBG (1, "x1=%d y1=%d\n", chndl->x1, chndl->y1);
  DBG (1, "x2=%d y2=%d\n", chndl->x2, chndl->y2);
  DBG (1, "width=%ld height=%ld\n", chndl->width, chndl->height);

  CHK (do_scan (chndl));
  return status;
}


static SANE_Status
CANON_set_scan_parameters (CANON_Handle * chndl)
{
  int left;
  int top;
  int right;
  int bottom;

  double leftf;
  double rightf;
  double topf;
  double bottomf;

  double widthf;
  double heightf;
  int widthi;
  int heighti;

  int top_edge = 7;		/* in mm */
  if (chndl->val[opt_resolution].w < 300)
    {
      top_edge = 0;
    }
  if (chndl->val[opt_resolution].w == 300 && chndl->productcode == 0x2224)
    {
      top_edge = 0;
    }

  left = SANE_UNFIX (chndl->val[opt_tl_x].w) / MM_IN_INCH * 600;
  top = (top_edge + SANE_UNFIX (chndl->val[opt_tl_y].w)) / MM_IN_INCH * 600;
  right = SANE_UNFIX (chndl->val[opt_br_x].w) / MM_IN_INCH * 600;
  bottom =
    (top_edge + SANE_UNFIX (chndl->val[opt_br_y].w)) / MM_IN_INCH * 600;

  leftf = SANE_UNFIX (chndl->val[opt_tl_x].w);
  rightf = SANE_UNFIX (chndl->val[opt_br_x].w);
  topf = SANE_UNFIX (chndl->val[opt_tl_y].w);
  bottomf = SANE_UNFIX (chndl->val[opt_br_y].w);

  widthf = (rightf - leftf) / MM_PER_INCH * 600;
  widthi = (int) widthf;
  heightf = (bottomf - topf) / MM_PER_INCH * 600;
  heighti = (int) heightf;

  DBG (2, "CANON_set_scan_parameters:\n");
  DBG (2, "widthf = %f\n", widthf);
  DBG (2, "widthi = %d\n", widthi);
  DBG (2, "in 600dpi pixels:\n");
  DBG (2, "left  = %d, top    = %d\n", left, top);
  DBG (2, "right = %d, bottom = %d\n", right, bottom);

  /* Validate the input parameters */
  if ((left < 0) || (right > CANON_MAX_WIDTH))
    {
      return SANE_STATUS_INVAL;
    }

  if ((top < 0) || (bottom > CANON_MAX_HEIGHT))
    {
      return SANE_STATUS_INVAL;
    }

  if (((right - left) < 10) || ((bottom - top) < 10))
    {
      return SANE_STATUS_INVAL;
    }

  if ((chndl->val[opt_resolution].w != 75) &&
      (chndl->val[opt_resolution].w != 150) &&
      (chndl->val[opt_resolution].w != 300) &&
      (chndl->val[opt_resolution].w != 600) &&
      (chndl->val[opt_resolution].w != 1200) &&
      (chndl->val[opt_resolution].w != 2400) &&
      (chndl->val[opt_resolution].w != 4800))
    {
      return SANE_STATUS_INVAL;
    }

  /* Store params */
  chndl->x1 = left;
  chndl->x2 = left + widthi;
  chndl->y1 = top;
  chndl->y2 = top + heighti;
  chndl->absolute_threshold = (chndl->val[opt_threshold].w * 255) / 100;
  return SANE_STATUS_GOOD;
}


static SANE_Status
CANON_close_device (CANON_Handle * scan)
{
  DBG (3, "CANON_close_device:\n");
  sanei_usb_close (scan->fd);
  return SANE_STATUS_GOOD;
}


static SANE_Status
CANON_open_device (CANON_Handle * scan, const char *dev)
{
  SANE_Word vendor;
  SANE_Word product;
  SANE_Status res;

  DBG (3, "CANON_open_device: `%s'\n", dev);

  scan->fname = NULL;
  scan->fp = NULL;

  res = sanei_usb_open (dev, &scan->fd);

  if (res != SANE_STATUS_GOOD)
    {
      DBG (1, "CANON_open_device: couldn't open device `%s': %s\n", dev,
	   sane_strstatus (res));
      return res;
    }

  scan->product = "unknown";

#ifndef NO_AUTODETECT
  /* We have opened the device. Check that it is a USB scanner. */
  if (sanei_usb_get_vendor_product (scan->fd, &vendor, &product) !=
      SANE_STATUS_GOOD)
    {
      DBG (1, "CANON_open_device: sanei_usb_get_vendor_product failed\n");
      /* This is not a USB scanner, or SANE or the OS doesn't support it. */
      sanei_usb_close (scan->fd);
      scan->fd = -1;
      return SANE_STATUS_UNSUPPORTED;
    }

  /* Make sure we have a CANON scanner */
  if (vendor == 0x04a9)
    {
      scan->product = "Canon";
      scan->productcode = product;
      if (product == 0x2224)
	{
	  scan->product = "CanoScan LiDE 600F";
	}
      else if (product == 0x2225)
	{
	  scan->product = "CanoScan LiDE 70";
	}
      else
	{
	  DBG (1, "CANON_open_device: incorrect vendor/product (0x%x/0x%x)\n",
	       vendor, product);
	  sanei_usb_close (scan->fd);
	  scan->fd = -1;
	  return SANE_STATUS_UNSUPPORTED;
	}
    }
#endif

  return SANE_STATUS_GOOD;
}


static const char *
CANON_get_device_name (CANON_Handle * chndl)
{
  return chndl->product;
}


static SANE_Status
CANON_finish_scan (CANON_Handle * chndl)
{
  DBG (3, "CANON_finish_scan:\n");

  if (chndl->fp)
    {
      fclose (chndl->fp);
    }

  chndl->fp = NULL;

  /* remove temp file */
  if (chndl->fname)
    {
      DBG (4, "removing temp file %s\n", chndl->fname);
      unlink (chndl->fname);
      free (chndl->fname);
    }

  chndl->fname = NULL;
  return SANE_STATUS_GOOD;
}


static SANE_Status
CANON_start_scan (CANON_Handle * chndl)
{
  SANE_Status status;
  int result;
  int fd;
  DBG (3, "CANON_start_scan called\n");

  /* a previous scan that was not read to the end still has its temp file */
  CANON_finish_scan (chndl);

  /* choose a temp file name for scan data */
  chndl->fname = strdup ("/tmp/scan.XXXXXX");
  fd = mkstemp (chndl->fname);

  if (fd == -1)
    {
      return SANE_STATUS_IO_ERROR;
    }

  close (fd);

  /* check if calibration needed */
  result = init (chndl);

  if (result < 0)
    {
      DBG (1, "Can't talk on USB.\n");
      CANON_finish_scan (chndl);
      return SANE_STATUS_IO_ERROR;
    }

  if (go_home (chndl) < 0)
    {
      CANON_finish_scan (chndl);
      if (cancel_requested)
	{
	  return SANE_STATUS_CANCELLED;
	}
      DBG (1, "Can't move the slider home.\n");
      return SANE_STATUS_IO_ERROR;
    }

  /* scan */
  if ((status = scan (chndl)) != SANE_STATUS_GOOD)
    {
      CANON_finish_scan (chndl);
      return status;
    }

  /* prepare for reading the temp file back out */
  chndl->fp = fopen (chndl->fname, "r");
  DBG (4, "reading %s\n", chndl->fname);

  if (!chndl->fp)
    {
      DBG (1, "open %s", chndl->fname);
      CANON_finish_scan (chndl);
      return SANE_STATUS_IO_ERROR;
    }

  return SANE_STATUS_GOOD;
}


static SANE_Status
CANON_read (CANON_Handle * chndl, SANE_Byte * data,
	    SANE_Int max_length, SANE_Int * length)
{
  SANE_Status status;
  int read_len;

  DBG (5, "CANON_read called\n");

  /* cancelled while the frontend reads the scanned image */
  if (cancel_requested && chndl->fp)
    {
      DBG (4, "CANON_read: cancelled\n");
      *length = 0;
      CANON_finish_scan (chndl);
      return SANE_STATUS_CANCELLED;
    }

  if (!chndl->fp)
    {
      return SANE_STATUS_INVAL;
    }

  read_len = fread (data, 1, max_length, chndl->fp);
  /* return some data */
  if (read_len > 0)
    {
      *length = read_len;
      DBG (5, "CANON_read returned (%d/%d)\n", *length, max_length);
      return SANE_STATUS_GOOD;
    }

  /* EOF or file err */
  *length = 0;

  if (feof (chndl->fp))
    {
      DBG (4, "EOF\n");
      status = SANE_STATUS_EOF;
    }
  else
    {
      DBG (4, "IO ERR\n");
      status = SANE_STATUS_IO_ERROR;
    }

  CANON_finish_scan (chndl);
  DBG (5, "CANON_read returned (%d/%d)\n", *length, max_length);
  return status;
}
