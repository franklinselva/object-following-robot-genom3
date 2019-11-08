#include "accnam.h"

#include "cnam_c_types.h"


/* --- Activity ColorTrack ---------------------------------------------- */

/** Validation codel ValidateArgs of activity ColorTrack.
 *
 * Returns genom_ok.
 * Throws cnam_bad_cmd_port, cnam_bad_image_port, cnam_opencv_error.
 */
genom_event
ValidateArgs(const genom_context self)
{
  // nothing, just a place holder for an example
  return genom_ok;
}


/* --- Function Stop ---------------------------------------------------- */

/** Codel StopTrack of function Stop.
 *
 * Returns genom_ok.
 */
genom_event
StopTrack(int32_t verbose, const genom_context self)
{
  // nothing, just a place holder for an example
  return genom_ok;
}
