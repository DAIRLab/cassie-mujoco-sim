#include "pack_robot_out.h"

/*******************************************************************************
 * Drive macros from cassiemujoco.c
 ******************************************************************************/

#define DRIVE_LIST                              \
    X(leftLeg.hipRollDrive)                     \
    X(leftLeg.hipYawDrive)                      \
    X(leftLeg.hipPitchDrive)                    \
    X(leftLeg.kneeDrive)                        \
    X(leftLeg.footDrive)                        \
    X(rightLeg.hipRollDrive)                    \
    X(rightLeg.hipYawDrive)                     \
    X(rightLeg.hipPitchDrive)                   \
    X(rightLeg.kneeDrive)                       \
    X(rightLeg.footDrive)

// Requires that the vector (pointer) already be set, with memory allocated
// elsewhere
void pack_robot_out_vectors(dairlib_lcmt_robot_output* message,
    cassie_sim_t *sim, cassie_out_t* cassie_out, bool hold_pelvis) {

  // If we are not in a floating base, need to skip the first 7 postions
  // (x,y,z,quat) and the first 6 velocities (linear and ang. velocity)
  int qoffset = 0;
  int voffset = 0;
  if (!hold_pelvis) {
    qoffset = 7;
    voffset = 6;
  }

  // Get positions
  double* q = cassie_sim_qpos(sim);

  // See cassiemujoco.h cassie_sim_qpos for ordering of q
  // q starts with floating base coordinates and left hip,  identical to
  // the joint ordering chosen in cassiesim.c for the message
  // The offsetting here is to (1) get the proper length and (2) skip floating
  // base coordinates if necessary
  memcpy(message->position, &q[7 - qoffset], (3 + qoffset) * sizeof(double));

  // remainder of left leg
  message->position[3 + qoffset] = q[14];  // knee
  message->position[4 + qoffset] = q[20];  // foot
  message->position[5 + qoffset] = q[15];  // shin
  message->position[6 + qoffset] = q[16];  // tarsus
  message->position[7 + qoffset] = q[17];  // heel spring

  // right hip, also in identical order
  memcpy(&message->position[8 + qoffset], &q[21], 3 * sizeof(double));

  // remainder of right leg
  message->position[11 + qoffset] = q[28];  // knee
  message->position[12 + qoffset] = q[34];  // foot
  message->position[13 + qoffset] = q[29];  // shin
  message->position[14 + qoffset] = q[30];  // tarsus
  message->position[15 + qoffset] = q[31];  // heel spring

  // get velocities
  double* v = cassie_sim_qvel(sim);
  // See cassiemujoco.h cassie_sim_qvel for ordering of v
  // floating base and left hip
  memcpy(message->velocity, &v[6 - voffset], (3 + voffset) * sizeof(double));

  // remainder of left legqoffsetqoffset
  message->velocity[3 + voffset] = v[12];  // knee
  message->velocity[4 + voffset] = v[18];  // foot
  message->velocity[5 + voffset] = v[13];  // shin
  message->velocity[6 + voffset] = v[14];  // tarsus
  message->velocity[7 + voffset] = v[15];  // heel spring

  // right hip
  memcpy(&message->velocity[8 + voffset], &v[19], 3 * sizeof(double));

  // remainder of right leg
  message->velocity[11 + voffset] = v[25];  // knee
  message->velocity[12 + voffset] = v[31];  // foot
  message->velocity[13 + voffset] = v[26];  // shin
  message->velocity[14 + voffset] = v[27];  // tarsus
  message->velocity[15 + voffset] = v[28];  // heel spring

  // Efforts
  // Ordered list of drive_out_t addresses
  elmo_out_t *drives[10] = {
  #define X(drive) &cassie_out->drive,
    DRIVE_LIST
  #undef X
  };
  message->effort[0] = drives[0]->torque;
  message->effort[1] = drives[1]->torque;
  message->effort[2] = drives[2]->torque;
  message->effort[3] = drives[3]->torque;
  message->effort[4] = drives[4]->torque;
  message->effort[5] = drives[5]->torque;
  message->effort[6] = drives[6]->torque;
  message->effort[7] = drives[7]->torque;
  message->effort[8] = drives[8]->torque;
  message->effort[9] = drives[9]->torque;
}
