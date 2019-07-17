#include "udp_lcm_translator.h"


void copy_elmo(elmo_out_t* input, dairlib_lcmt_elmo_out* output) {
  output->statusWord = input->statusWord;
  output->position = input->position;
  output->velocity = input->velocity;
  output->torque = input->torque;
  output->driveTemperature = input->driveTemperature;
  output->dcLinkVoltage = input->dcLinkVoltage;
  output->torqueLimit = input->torqueLimit;
  output->gearRatio = input->gearRatio;
}

void copy_leg(cassie_leg_out_t* input, dairlib_lcmt_cassie_leg_out* output) {
  copy_elmo(&input->hipRollDrive, &output->hipRollDrive);
  copy_elmo(&input->hipYawDrive, &output->hipYawDrive);
  copy_elmo(&input->hipPitchDrive, &output->hipPitchDrive);
  copy_elmo(&input->kneeDrive, &output->kneeDrive);
  copy_elmo(&input->footDrive, &output->footDrive);
  output->shinJoint.position = input->shinJoint.position;
  output->shinJoint.velocity = input->shinJoint.velocity;
  output->tarsusJoint.position = input->tarsusJoint.position;
  output->tarsusJoint.velocity = input->tarsusJoint.velocity;
  output->footJoint.position = input->footJoint.position;
  output->footJoint.velocity = input->footJoint.velocity;
  output->medullaCounter = input->medullaCounter;
  output->medullaCpuLoad = input->medullaCpuLoad;
  output->reedSwitchState = input->reedSwitchState;
}

void copy_vector_int16(int16_t* input, int16_t* output, int size) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i];
  }
}

void copy_vector_int32(int32_t* input, int32_t* output, int size) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i];
  }
}

void copy_vector_double(double* input, double* output, int size) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i];
  }
}

void cassieOutToLcm(cassie_out_t* cassie_out, double time_seconds,
    dairlib_lcmt_cassie_out* message) {
  message->utime = time_seconds * 1e6;

  // copy pelvis
  copy_vector_int32(cassie_out->pelvis.targetPc.etherCatStatus,
              message->pelvis.targetPc.etherCatStatus, 6);
  copy_vector_int32(cassie_out->pelvis.targetPc.etherCatNotifications,
              message->pelvis.targetPc.etherCatNotifications, 21);

  message->pelvis.targetPc.taskExecutionTime =
      cassie_out->pelvis.targetPc.taskExecutionTime;
  message->pelvis.targetPc.overloadCounter =
      cassie_out->pelvis.targetPc.overloadCounter;

  message->pelvis.targetPc.cpuTemperature =
      cassie_out->pelvis.targetPc.cpuTemperature;

  message->pelvis.battery.dataGood =
      cassie_out->pelvis.battery.dataGood;
  message->pelvis.battery.stateOfCharge =
      cassie_out->pelvis.battery.stateOfCharge;
  message->pelvis.battery.current =
      cassie_out->pelvis.battery.current;
  copy_vector_double(cassie_out->pelvis.battery.voltage,
              message->pelvis.battery.voltage, 12);
  copy_vector_double(cassie_out->pelvis.battery.temperature,
              message->pelvis.battery.temperature, 4);

  message->pelvis.radio.radioReceiverSignalGood =
      cassie_out->pelvis.radio.radioReceiverSignalGood;
  message->pelvis.radio.receiverMedullaSignalGood =
      cassie_out->pelvis.radio.receiverMedullaSignalGood;
  copy_vector_double(cassie_out->pelvis.radio.channel,
              message->pelvis.radio.channel, 16);

  message->pelvis.vectorNav.dataGood =
      cassie_out->pelvis.vectorNav.dataGood;
  message->pelvis.vectorNav.vpeStatus =
      cassie_out->pelvis.vectorNav.vpeStatus;
  message->pelvis.vectorNav.pressure =
      cassie_out->pelvis.vectorNav.pressure;
  message->pelvis.vectorNav.temperature =
      cassie_out->pelvis.vectorNav.temperature;
  copy_vector_double(cassie_out->pelvis.vectorNav.magneticField,
              message->pelvis.vectorNav.magneticField, 3);
  copy_vector_double(cassie_out->pelvis.vectorNav.angularVelocity,
              message->pelvis.vectorNav.angularVelocity, 3);
  copy_vector_double(cassie_out->pelvis.vectorNav.linearAcceleration,
              message->pelvis.vectorNav.linearAcceleration, 3);
  copy_vector_double(cassie_out->pelvis.vectorNav.orientation,
              message->pelvis.vectorNav.orientation, 4);
  message->pelvis.medullaCounter = cassie_out->pelvis.medullaCounter;
  message->pelvis.medullaCpuLoad = cassie_out->pelvis.medullaCpuLoad;
  message->pelvis.bleederState = cassie_out->pelvis.bleederState;
  message->pelvis.leftReedSwitchState = cassie_out->pelvis.leftReedSwitchState;
  message->pelvis.rightReedSwitchState = cassie_out->pelvis.rightReedSwitchState;
  message->pelvis.vtmTemperature = cassie_out->pelvis.vtmTemperature;

  copy_leg(&cassie_out->leftLeg, &message->leftLeg);
  copy_leg(&cassie_out->rightLeg, &message->rightLeg);

  copy_vector_int16(cassie_out->messages,
              message->messages, 4);

  message->isCalibrated = cassie_out->isCalibrated;
}
