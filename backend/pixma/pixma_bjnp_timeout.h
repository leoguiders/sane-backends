/* SANE - Scanner Access Now Easy.

   Copyright (C) 2008 2012 by Louis Lagendijk
   Copyright (C) 2026 SANE Project

   This file is part of the SANE package.

   SANE is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   SANE is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
   License for more details.

   You should have received a copy of the GNU General Public License
   along with sane; see the file COPYING.
   If not, see <https://www.gnu.org/licenses/>.

   As a special exception, the authors of SANE give permission for
   additional uses of the libraries contained in this release of SANE.

   The exception is that, if you link a SANE library with other files
   to produce an executable, this does not by itself cause the
   resulting executable to be covered by the GNU General Public
   License.  Your use of that executable is in no way restricted on
   account of linking the SANE library code into it.

   This exception does not, however, invalidate any other reasons why
   the executable file might be covered by the GNU General Public
   License.

   If you submit changes to SANE to the maintainers to be included in
   a subsequent release, you agree by submitting the changes that
   those changes may be distributed with this exception intact.

   If you write modifications of your own for SANE, it is your choice
   whether to permit this exception to apply to your modifications.
   If you do not wish that, delete this exception notice.
*/

#ifndef PIXMA_BJNP_TIMEOUT_H
#define PIXMA_BJNP_TIMEOUT_H

/* Convert a nonnegative timeout in milliseconds for select().
   The caller provides the platform definition of struct timeval. */
static void
bjnp_timeout_to_timeval (int milliseconds, struct timeval *timeout)
{
  timeout->tv_sec = milliseconds / 1000;
  timeout->tv_usec = (milliseconds % 1000) * 1000;
}

#endif /* PIXMA_BJNP_TIMEOUT_H */
