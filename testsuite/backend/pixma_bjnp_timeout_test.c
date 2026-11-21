/* sane - Scanner Access Now Easy.

   Copyright (C) 2026 SANE Project

   This file is part of the SANE package and is distributed under the
   terms of the GNU General Public License version 3 or later.

   Pure BJNP timeout conversion tests: no scanner or socket access. */

#include "sane/config.h"

#ifdef HAVE_SYS_TYPES_H
#include <sys/types.h>
#endif
#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif
#ifdef HAVE_SYS_SELECT_H
#include <sys/select.h>
#endif
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../backend/pixma/pixma_bjnp_timeout.h"

int
main (void)
{
  static const struct
  {
    int milliseconds;
    long seconds;
    long microseconds;
  } cases[] = {
    { 0, 0, 0 },
    { 1, 0, 1000 },
    { 10, 0, 10000 },
    { 500, 0, 500000 },
    { 999, 0, 999000 },
    { 1000, 1, 0 },
    { 1001, 1, 1000 },
    { 1500, 1, 500000 },
    { 1999, 1, 999000 },
    { 2000, 2, 0 },
    { 2001, 2, 1000 },
    { 5000, 5, 0 },
    { 10000, 10, 0 },
    { INT_MAX, INT_MAX / 1000, (INT_MAX % 1000) * 1000 }
  };
  size_t i;
  int failures = 0;

  for (i = 0; i < sizeof (cases) / sizeof (cases[0]); i++)
    {
      struct timeval timeout;

      bjnp_timeout_to_timeval (cases[i].milliseconds, &timeout);
      if (timeout.tv_sec != cases[i].seconds
          || timeout.tv_usec != cases[i].microseconds
          || timeout.tv_usec < 0 || timeout.tv_usec >= 1000000)
        {
          fprintf (stderr, "%d ms: expected %ld s, %ld us; got %ld s, %ld us\n",
                   cases[i].milliseconds, cases[i].seconds,
                   cases[i].microseconds, (long) timeout.tv_sec,
                   (long) timeout.tv_usec);
          failures++;
        }
    }

  printf ("BJNP timeout: %lu cases, %d failures\n",
          (unsigned long) (sizeof (cases) / sizeof (cases[0])), failures);
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
