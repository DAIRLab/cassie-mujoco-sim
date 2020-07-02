/*
 * Copyright (c) 2018 Dynamic Robotics Laboratory
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <lcm/lcm.h>
#include "cassiemujoco.h"
#include "udp.h"
#include "dairlib_lcmt_cassie_out.h"
#include "dairlib_lcmt_robot_output.h"
#include "dairlib_lcmt_contact_mujoco.h"
#include "udp_lcm_translator.h"
#include "pack_robot_out.h"


enum mode {
    MODE_STANDARD,
    MODE_PD
};


#ifdef _WIN32
#include <windows.h>

static long long get_microseconds(void)
{
    LARGE_INTEGER count, frequency;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&count);
    return (count.QuadPart * 1000000) / frequency.QuadPart;
}

#else

static long long get_microseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

#endif // _WIN32


int main(int argc, char *argv[])
{
    // LCM
    lcm_t *lcm = lcm_create(NULL);

    // Option variables and flags
    char *iface_addr_str = "0.0.0.0";
    char *iface_port_str = "25000";
    bool realtime = false;
    bool visualize = false;
    bool hold = false;
    bool state_pub = false;
    char *log_file_path = NULL;
    char *qlog_file_path = NULL;
    int mode = MODE_STANDARD;

    // Parse arguments
    int c;
    while ((c = getopt(argc, argv, "a:p:rvhsl:q:x")) != -1) {
        switch (c) {
        case 'a':
            // Inteface address to bind
            iface_addr_str = optarg;
            break;
        case 'p':
            // Port to bind
            iface_port_str = optarg;
            break;
        case 'r':
            // Enable real-time mode
            realtime = true;
            break;
        case 'v':
            // Visualize simulation
            visualize = true;
            break;
        case 'h':
            // Hold pelvis
            hold = true;
            break;
        case 'l':
            // Log data
            log_file_path = optarg;
            break;
        case 'q':
            // Log simulator state data
            qlog_file_path = optarg;
            break;
        case 'x':
            // Run in PD mode
            mode = MODE_PD;
            break;
        case 's':
            // Publish state via LCM
            state_pub = true;
            break;
        default:
            // Print usage
            printf(
"Usage: cassiesim [OPTION]...\n"
"Simulates the Cassie robot communicating over UDP.\n"
"\n"
"  -a [ADDRESS]   Specify the local interface to bind to.\n"
"  -p [PORT]      Specify the port to listen on.\n"
"  -r             Run simulation continuously instead of in lockstep.\n"
"  -v             Show a visualization of the running simulation.\n"
"  -h             Hold the pelvis in place.\n"
"  -l [FILENAME]  Log the input and output UDP data to a file.\n"
"  -q [FILENAME]  Log simulation time, qpos, and qvel to a file.\n"
"  -s             Publish the full state via LCM on CASSIE_STATE channel. \n"
"  -x             Run in PD mode, taking PD targets and sending state estimates.\n"
"\n"
"By default, the simulator listens on all IPv4 interfaces at port %s.\n"
"If the option -r is not given, the simulator waits for input after sending\n"
"each sensor data packet. With -r, the simulator runs in real-time, reusing\n"
"old input if new data is not yet ready and waiting a minimum amount of time\n"
"between sending sensor data packets.\n\n", iface_port_str);
            exit(EXIT_SUCCESS);
        }
    }

    // Bind to network interface
    int sock = udp_init_host(iface_addr_str, iface_port_str);
    if (-1 == sock)
        exit(EXIT_FAILURE);

    // Create packet input/output buffers
    int dinlen, doutlen;
    switch (mode) {
    case MODE_PD:
        dinlen = PD_IN_T_PACKED_LEN;
        doutlen = STATE_OUT_T_PACKED_LEN;
        break;
    default:
        dinlen = CASSIE_USER_IN_T_PACKED_LEN;
        doutlen = CASSIE_OUT_T_PACKED_LEN;
    }
    const int recvlen = PACKET_HEADER_LEN + dinlen;
    const int sendlen = PACKET_HEADER_LEN + doutlen;
    unsigned char *recvbuf = malloc(recvlen);
    unsigned char *sendbuf = malloc(sendlen);

    // Separate input/output buffers into header and payload
    const unsigned char *header_in = recvbuf;
    const unsigned char *data_in = &recvbuf[PACKET_HEADER_LEN];
    unsigned char *header_out = sendbuf;
    unsigned char *data_out = &sendbuf[PACKET_HEADER_LEN];

    // Create standard input/output structs
    cassie_user_in_t cassie_user_in = {0};
    cassie_out_t cassie_out;

    // Create PD input/output structs
    pd_in_t pd_in = {0};
    state_out_t state_out;

    // Create header information struct
    packet_header_info_t header_info = {0};

    // Address to send sensor data packets to
    struct sockaddr_storage src_addr = {0};
    socklen_t addrlen = sizeof src_addr;

    struct sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    remoteaddr.sin_port = htons(25001);

    // Create cassie simulation
    const char modelfile[] = "../model/cassie.xml";
    // const char modelfile[] = "/home/yangwill/workspace/dairlib/examples/Cassie/urdf/cassie_v2.urdf";
    cassie_sim_t *sim = cassie_sim_init(modelfile);
    cassie_vis_t *vis;
    if (visualize)
        vis = cassie_vis_init(sim, modelfile);
    if (hold)
        cassie_sim_hold(sim);

    // Cassie input/output log file
    FILE *log_file = NULL;
    if (log_file_path)
        log_file = fopen(log_file_path, "wb");

    // SImulator state log file
    FILE *qlog_file = NULL;
    if (qlog_file_path)
        qlog_file = fopen(qlog_file_path, "wb");

    // Manage simulation loop
    unsigned long long loop_counter = 0;
    bool run_sim = false;
    double realtime_rate = 0.5;
    const long long cycle_usec = 1000000 / (2000 * realtime_rate);
    const long long timeout_usec = mode == MODE_PD ? 100000 : 10000;
    long long send_time = get_microseconds();
    long long recv_time = get_microseconds();

    // Create robot_output lcm message
    dairlib_lcmt_robot_output robot_output_message;
    double position[23];
    robot_output_message.position = position;
    robot_output_message.position_names =
        (char**) lcm_malloc(sizeof(char*) * 23);
    double velocity[22];
    robot_output_message.velocity = velocity;
    robot_output_message.velocity_names =
        (char**) lcm_malloc(sizeof(char*) * 22);
    double effort[10];
    robot_output_message.effort = effort;
    robot_output_message.effort_names =
        (char**) lcm_malloc(sizeof(char*) * 10);

    int qoffset = 0;
    int voffset = 0;

    // Create contact_force lcm message
    dairlib_lcmt_contact_mujoco cassie_mujoco_contact;
    double contact_forces[12];
    cassie_mujoco_contact.contact_forces = contact_forces;
    cassie_mujoco_contact.num_contact_forces = 12;

    if(!hold) {
        // Although this is different than the ordering of MBP, state receiver will rearrange accordingly
        robot_output_message.position_names[0] = "base_x";
        robot_output_message.position_names[1] = "base_y";
        robot_output_message.position_names[2] = "base_z";        
        robot_output_message.position_names[3] = "base_qw";
        robot_output_message.position_names[4] = "base_qx";
        robot_output_message.position_names[5] = "base_qy";
        robot_output_message.position_names[6] = "base_qz";

        robot_output_message.velocity_names[0] = "base_vx";
        robot_output_message.velocity_names[1] = "base_vy";
        robot_output_message.velocity_names[2] = "base_vz";
        robot_output_message.velocity_names[3] = "base_wx";
        robot_output_message.velocity_names[4] = "base_wy";
        robot_output_message.velocity_names[5] = "base_wz";

        qoffset = 7;
        voffset = 6;
    }
    robot_output_message.num_positions = 16 + qoffset;
    robot_output_message.num_velocities = 16 + voffset;

    robot_output_message.position_names[qoffset] = "hip_roll_left";
    robot_output_message.position_names[qoffset + 1] = "hip_yaw_left";
    robot_output_message.position_names[qoffset + 2] = "hip_pitch_left";
    robot_output_message.position_names[qoffset + 3] = "knee_left";
    robot_output_message.position_names[qoffset + 4] = "toe_left";
    robot_output_message.position_names[qoffset + 5] = "knee_joint_left";
    robot_output_message.position_names[qoffset + 6] = "ankle_joint_left";
    robot_output_message.position_names[qoffset + 7] =
            "ankle_spring_joint_left";
    robot_output_message.position_names[qoffset + 8] = "hip_roll_right";
    robot_output_message.position_names[qoffset + 9] = "hip_yaw_right";
    robot_output_message.position_names[qoffset + 10] = "hip_pitch_right";
    robot_output_message.position_names[qoffset + 11] = "knee_right";
    robot_output_message.position_names[qoffset + 12] = "toe_right";
    robot_output_message.position_names[qoffset + 13] = "knee_joint_right";
    robot_output_message.position_names[qoffset + 14] = "ankle_joint_right";
    robot_output_message.position_names[qoffset + 15] =
            "ankle_spring_joint_right";

    robot_output_message.velocity_names[voffset] = "hip_roll_leftdot";
    robot_output_message.velocity_names[voffset + 1] = "hip_yaw_leftdot";
    robot_output_message.velocity_names[voffset + 2] = "hip_pitch_leftdot";
    robot_output_message.velocity_names[voffset + 3] = "knee_leftdot";
    robot_output_message.velocity_names[voffset + 4] = "toe_leftdot";
    robot_output_message.velocity_names[voffset + 5] = "knee_joint_leftdot";
    robot_output_message.velocity_names[voffset + 6] = "ankle_joint_leftdot";
    robot_output_message.velocity_names[voffset + 7] =
            "ankle_spring_joint_leftdot";
    robot_output_message.velocity_names[voffset + 8] = "hip_roll_rightdot";
    robot_output_message.velocity_names[voffset + 9] = "hip_yaw_rightdot";
    robot_output_message.velocity_names[voffset + 10] = "hip_pitch_rightdot";
    robot_output_message.velocity_names[voffset + 11] = "knee_rightdot";
    robot_output_message.velocity_names[voffset + 12] = "toe_rightdot";
    robot_output_message.velocity_names[voffset + 13] = "knee_joint_rightdot";
    robot_output_message.velocity_names[voffset + 14] = "ankle_joint_rightdot";
    robot_output_message.velocity_names[voffset + 15] =
            "ankle_spring_joint_rightdot";

    robot_output_message.num_efforts = 10;
    robot_output_message.effort_names[0] = "hip_roll_left_motor";
    robot_output_message.effort_names[1] = "hip_yaw_left_motor";
    robot_output_message.effort_names[2] = "hip_pitch_left_motor";
    robot_output_message.effort_names[3] = "knee_left_motor";
    robot_output_message.effort_names[4] = "toe_left_motor";
    robot_output_message.effort_names[5] = "hip_roll_right_motor";
    robot_output_message.effort_names[6] = "hip_yaw_right_motor";
    robot_output_message.effort_names[7] = "hip_pitch_right_motor";
    robot_output_message.effort_names[8] = "knee_right_motor";
    robot_output_message.effort_names[9] = "toe_right_motor";
    run_sim = true;

    // Listen/respond loop
    while (true) {
        // Try to get a new packet
        ssize_t nbytes;
        if (realtime) {
            // Get newest packet, or return -1 if no new packets are available
            nbytes = get_newest_packet(sock, recvbuf, recvlen,
                                       (struct sockaddr *) &src_addr, &addrlen);
        } else {
            // If not in real-time mode, wait until a new packet comes in
            nbytes = wait_for_packet(sock, recvbuf, recvlen,
                                     (struct sockaddr *) &src_addr, &addrlen);
        }

        // If a new packet was received, process and unpack it
        if (recvlen == nbytes) {
            // Process incoming header and write outgoing header
            process_packet_header(&header_info, header_in, header_out);
            // printf("\033[F\033[Jdelay: %d, diff: %d\n",
                   // header_info.delay, header_info.seq_num_in_diff);

            // Unpack received data into cassie user input struct
            switch (mode) {
            case MODE_PD:
                unpack_pd_in_t(data_in, &pd_in);
                break;
            default:
                unpack_cassie_user_in_t(data_in, &cassie_user_in);
            }

            // Update packet received timestamp
            recv_time = get_microseconds();

            // Start the simulation after the first valid packet is received
            run_sim = true;
        }

        if (run_sim) {
            // Run simulator and pack output struct into outgoing packet
            switch (mode) {
            case MODE_PD:
                cassie_sim_step_pd(sim, &state_out, &pd_in);
                pack_state_out_t(&state_out, data_out);
                cassie_sim_foot_forces(sim, contact_forces);
              break;
            default:
                cassie_sim_step(sim, &cassie_out, &cassie_user_in);
                pack_cassie_out_t(&cassie_out, data_out);
                cassie_sim_foot_forces(sim, contact_forces);
            }

            // print_inertia_matrix(sim);

            // Log Cassie input/output
            if (log_file) {
                fwrite(data_out, doutlen, 1, log_file);
                fwrite(data_in, dinlen, 1, log_file);
            }

            // Log simulation state
            if (qlog_file) {
                fwrite(cassie_sim_time(sim), sizeof (double), 1, qlog_file);
                fwrite(cassie_sim_qpos(sim), sizeof (double), 35, qlog_file);
                fwrite(cassie_sim_qvel(sim), sizeof (double), 32, qlog_file);
            }

            if (realtime) {
                // Wait at least cycle_usec between sending simulator updates
                while (get_microseconds() - send_time < cycle_usec) {}
                send_time = get_microseconds();

                // Zero input data if no new packets have been
                // received in a while
                if (get_microseconds() - recv_time > timeout_usec) {
                    memset(&cassie_user_in, 0, sizeof cassie_user_in);
                    memset(&pd_in, 0, sizeof pd_in);
                }
            }

            // Send response
            send_packet(sock, sendbuf, sendlen,
                        (struct sockaddr *) &remoteaddr, addrlen);
            // Send LCM response
            dairlib_lcmt_cassie_out message;
            double time = *cassie_sim_time(sim);
            cassieOutToLcm(&cassie_out, time, &message);
            dairlib_lcmt_cassie_out_publish(lcm, "CASSIE_OUTPUT", &message);

            if (state_pub) {
                pack_robot_out_vectors(&robot_output_message, sim, &cassie_out,
                    hold);
                robot_output_message.utime = (int64_t) (time * 1.0e6);
                cassie_mujoco_contact.utime = (int64_t) (time * 1.0e6);
                dairlib_lcmt_robot_output_publish(lcm, "CASSIE_STATE_SIMULATION",
                        &robot_output_message);
                dairlib_lcmt_contact_mujoco_publish(lcm, "CASSIE_CONTACT_MUJOCO",
                                                         &cassie_mujoco_contact);
            }
        }

        // Draw no more then once every 33 simulation steps
        if (visualize && loop_counter % 33 == 0)
            cassie_vis_draw(vis, sim);

        // Increment loop counter
        ++loop_counter;
    }
}
