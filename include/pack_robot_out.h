#ifndef PACKROBOTOUT_H
#define PACKROBOTOUT_H

#include <stdbool.h>
#include "cassiemujoco.h"
#include "dairlib_lcmt_robot_output.h"
#include "cassie_out_t.h"

#ifdef __cplusplus
extern "C" {
#endif


/// Pack the position, velocity, and effort vectors of the input message using
/// the simulation pointer. Takes the true values for position and velocity, 
/// without noise. The efforts are those reported by the simulated motors.
/// This DOES NOT change the utime, length, or name fields of the message
/// @p message the output message
/// @p sim the simulation object (used to get q and v)
/// @param cassie_out struct (used to get efforts)
/// @p hold_pelvis True if the pelvis is fixed, false if floating base
void pack_robot_out_vectors(dairlib_lcmt_robot_output* message,
    cassie_sim_t *sim, cassie_out_t* cassie_out, bool hold_pelvis);

#ifdef __cplusplus
}
#endif

#endif // PACKROBOTOUT_H
