/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
   This is the APMrover2 firmware. It was originally derived from
   ArduPlane by Jean-Louis Naudin (JLN), and then rewritten after the
   AP_HAL merge by Andrew Tridgell

   Maintainer: Grant Morphett

   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Andrew Tridgell, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Jean-Louis Naudin, Grant Morphett

   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier 

   APMrover alpha version tester: Franco Borasio, Daniel Chapelat...

   Please contribute your ideas! See http://dev.ardupilot.org for details
*/

#include "Rover.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Rover rover;

/*
  scheduler table - all regular tasks should be listed here, along
  with how often they should be called (in 20ms units) and the maximum
  time they are expected to take (in microseconds)
*/
AP_Task Rover::scheduler_tasks[] = {
    //         Function name,          Hz,     us,
    make_task("read_radio",             &rover, &Rover::read_radio,             50,   1000),
    make_task("ahrs_update",            &rover, &Rover::ahrs_update,            50,   6400),
    make_task("read_rangefinders",      &rover, &Rover::read_rangefinders,      50,   2000),
    make_task("update_current_mode",    &rover, &Rover::update_current_mode,    50,   1500),
    make_task("set_servos",             &rover, &Rover::set_servos,             50,   1500),
    make_task("update_GPS_50Hz",        &rover, &Rover::update_GPS_50Hz,        50,   2500),
    make_task("update_GPS_10Hz",        &rover, &Rover::update_GPS_10Hz,        10,   2500),
    make_task("update_alt",             &rover, &Rover::update_alt,             10,   3400),
    make_task("AP_Beacon::update",      &rover.g2.beacon, &AP_Beacon::update,   50,   50),
    make_task("AP_Proximity::update",   &rover.g2.proximity, &AP_Proximity::update,           50,   50),
    make_task("update_visual_odom",     &rover, &Rover::update_visual_odom,     50,     50),
    make_task("update_wheel_encoder",   &rover, &Rover::update_wheel_encoder,   20,     50),
    make_task("update_compass",         &rover, &Rover::update_compass,         10,   2000),
    make_task("update_mission",         &rover, &Rover::update_mission,         10,   1000),
    make_task("update_logging1",        &rover, &Rover::update_logging1,        10,   1000),
    make_task("update_logging2",        &rover, &Rover::update_logging2,        10,   1000),
    make_task("gcs_retry_deferred",     &rover, &Rover::gcs_retry_deferred,     50,   1000),
    make_task("gcs_update",             &rover, &Rover::gcs_update,             50,   1700),
    make_task("gcs_data_stream_send",   &rover, &Rover::gcs_data_stream_send,   50,   3000),
    make_task("read_control_switch",    &rover, &Rover::read_control_switch,     7,   1000),
    make_task("read_aux_switch",        &rover, &Rover::read_aux_switch,        10,    100),
    make_task("AP_BattMonitor::read",   &rover.battery, &AP_BattMonitor::read,           10, 1000),
    make_task("read_receiver_rssi",     &rover, &Rover::read_receiver_rssi,     10,   1000),
    make_task("AP_ServoRelayEvents::update_events", &rover.ServoRelayEvents, &AP_ServoRelayEvents::update_events,  50, 1000),
    make_task("check_usb_mux",          &rover, &Rover::check_usb_mux,           3,   1000),
#if MOUNT == ENABLED
    make_task("AP_Mount::update",       &rover.camera_mount, &AP_Mount::update, 50,  600),
#endif
#if CAMERA == ENABLED
    make_task("AP_Camera::update_trigger", &rover.camera, &AP_Camera::update_trigger, 50,  600),
#endif
    make_task("one_second_loop",        &rover, &Rover::one_second_loop,         1,   3000),
    make_task("compass_cal_update",     &rover, &Rover::compass_cal_update,     50,    100),
    make_task("ModeSmartRTL::save_position", &rover.mode_smartrtl, &ModeSmartRTL::save_position,   3,  100),
    make_task("AP_Notify::save_position", &rover.notify, &AP_Notify::update,    50,  300),
    make_task("stats_update",           &rover, &Rover::stats_update,            1,    100),
    make_task("DataFlash_Class::periodic_tasks", &rover.DataFlash, &DataFlash_Class::periodic_tasks, 50,  300),
    make_task("AP_InertialSensor::periodic",   &rover.ins, &AP_InertialSensor::periodic,       50,   50),
    make_task("AP_Scheduler::update_logging",  &rover.scheduler, &AP_Scheduler::update_logging, 0.1,  75),
    make_task("AP_Button::update_logging", &rover.button, &AP_Button::update,          5,  100),
#if ADVANCED_FAILSAFE == ENABLED
    make_task("afs_fs_check",           &rover, &Rover::afs_fs_check,           10,    100),
#endif
};

/*
  update AP_Stats
*/
void Rover::stats_update(void)
{
    g2.stats.set_flying(motor_active());
    g2.stats.update();
}

/*
  setup is called when the sketch starts
 */
void Rover::setup()
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
}

/*
  loop() is called rapidly while the sketch is running
 */
void Rover::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_last_loop_time_s();
}

void Rover::update_soft_armed()
{
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    DataFlash.set_vehicle_armed(hal.util->get_soft_armed());
}

// update AHRS system
void Rover::ahrs_update()
{
    update_soft_armed();

#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before AHRS update
    gcs_update();
#endif

    // when in reverse we need to tell AHRS not to assume we are a
    // 'fly forward' vehicle, otherwise it will see a large
    // discrepancy between the mag and the GPS heading and will try to
    // correct for it, leading to a large yaw error
    ahrs.set_fly_forward(!in_reverse);

    ahrs.update();

    // update home from EKF if necessary
    update_home_from_EKF();

    // if using the EKF get a speed update now (from accelerometers)
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = norm(velocity.x, velocity.y);
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        ground_speed = ahrs.groundspeed();
    }

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_IMU(ins);
    }
}

void Rover::update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
}

/*
  check for GCS failsafe - 10Hz
 */
void Rover::gcs_failsafe_check(void)
{
    if (g.fs_gcs_enabled) {
        failsafe_trigger(FAILSAFE_EVENT_GCS, last_heartbeat_ms != 0 && (millis() - last_heartbeat_ms) > 2000);
    }
}

/*
  check for new compass data - 10Hz
 */
void Rover::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        if (should_log(MASK_LOG_COMPASS)) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_THR)) {
        Log_Write_Throttle();
        Log_Write_Beacon();
        Log_Write_Proximity();
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
    }
}

/*
  log some key data - 10Hz
 */
void Rover::update_logging2(void)
{
    if (should_log(MASK_LOG_STEERING)) {
        Log_Write_Steering();
    }

    if (should_log(MASK_LOG_RC)) {
        Log_Write_RC();
        Log_Write_WheelEncoder();
    }

    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_Vibration(ins);
    }
}


/*
  update aux servo mappings
 */
void Rover::update_aux(void)
{
    SRV_Channels::enable_aux_servos();
}

/*
  once a second events
 */
void Rover::one_second_loop(void)
{
    // send a heartbeat
    gcs().send_message(MSG_HEARTBEAT);

    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    set_control_channels();

    // cope with changes to aux functions
    update_aux();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

    // cope with changes to mavlink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    static uint8_t counter;

    counter++;

    // save compass offsets once a minute
    if (counter >= 60) {
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        counter = 0;
    }

    // update home position if not soft armed and gps position has
    // changed. Update every 1s at most
    if (!hal.util->get_soft_armed() &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        update_home();
    }

    // update error mask of sensors and subsystems. The mask uses the
    // MAV_SYS_STATUS_* values from mavlink. If a bit is set then it
    // indicates that the sensor or subsystem is present but not
    // functioning correctly
    update_sensor_status_flags();
}

void Rover::update_GPS_50Hz(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
    gps.update();

    for (uint8_t i=0; i < gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i);
            }
        }
    }
}


void Rover::update_GPS_10Hz(void)
{
    have_position = ahrs.get_position(current_loc);

    if (gps.last_message_time_ms() != last_gps_msg_ms) {
        last_gps_msg_ms = gps.last_message_time_ms();

        // set system time if necessary
        set_system_time_from_GPS();
#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Rover::update_current_mode(void)
{
    control_mode->update();
}

AP_HAL_MAIN_CALLBACKS(&rover);
