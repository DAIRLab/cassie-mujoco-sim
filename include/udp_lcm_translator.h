#pragma once

#include "cassie_out_t.h"
#include "cassie_user_in_t.h"
#include "dairlib_lcmt_cassie_out.h"
#include "dairlib_lcmt_cassie_in.h"

// Convert from Agility cassie_out_t struct to LCM message,
// dairlib::lcmt_cassie_out. Since the struct does not include time, time (s)
// is a required additional input
void cassieOutToLcm(cassie_out_t* cassie_out, double time_seconds,
    dairlib_lcmt_cassie_out* message);

