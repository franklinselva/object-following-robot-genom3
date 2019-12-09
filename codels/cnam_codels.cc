#include <stdio.h>
#include "accnam.h"

#include "cnam_c_types.h"

/* --- Function Stop ---------------------------------------------------- */

/** Codel StopTrack of function Stop.
 *
 * Returns genom_ok.
 */
genom_event
StopTrack(int32_t verbose, const genom_context self)
{
  // nothing, just a place holder for an example
  
  if (verbose > 0) printf("This codel does nothing. But the Stop function service interrupts the ColorTrack activity.\n");
  return genom_ok;
}
