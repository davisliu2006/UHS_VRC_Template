#pragma once

#include "../globals.hpp"
#include "sensing.hpp"

namespace auton {
    // DEFINITIONS

    const double TURN_MINDIFF = 5; // changes turn tolerence (minimum angle diff)
    const double TURN_MAXDIFF = 100; // changes turn scaling upper bound angle

    // SIMPLE MOVEMENT

    // simple move
    inline void advance(int vel) {
        flmotor.move_velocity(vel);
        frmotor.move_velocity(vel);
        rlmotor.move_velocity(vel);
        rrmotor.move_velocity(vel);
    }
    inline void move(double lvel, double rvel) {
        flmotor.move_velocity(lvel);
        frmotor.move_velocity(rvel);
        rlmotor.move_velocity(lvel);
        rrmotor.move_velocity(rvel);
    }

    // simple turn
    inline void turn(int rotvel) {
        flmotor.move_velocity(rotvel);
        frmotor.move_velocity(-rotvel);
        rlmotor.move_velocity(rotvel);
        rrmotor.move_velocity(-rotvel);
    }

    // stop
    inline void stop() {
        flmotor.move_velocity(0);
        frmotor.move_velocity(0);
        rlmotor.move_velocity(0);
        rrmotor.move_velocity(0);
    }

    // wait with background processing
    inline void wait(double dt) {
        sens::update();
        while (dt > 0) {
            sens::update();
            dt -= sens::dt;
        }
    }
    inline void wait_until(function<bool()> func) {
        sens::update();
        while (!func()) {
            sens::update();
        }
    }

    // MEASURED MOVEMENT

    // move distance
    inline void advance_time(double vel, double dt) {
        advance(vel);
        wait(dt);
        stop();
    }
    inline void advance_straight(double vel, double dt, double corr = 1) {
        sens::update();
        double rot = sens::rot;
        while (dt > 0) {
            sens::update();
            double rotdiff = angl_180(rot-sens::rot)/TURN_MAXDIFF;
            flmotor.move_velocity(vel+rotdiff*corr);
            frmotor.move_velocity(vel-rotdiff*corr);
            rlmotor.move_velocity(vel+rotdiff*corr);
            rrmotor.move_velocity(vel-rotdiff*corr);
            dt -= sens::dt;
        }
        stop();
    }
    inline void advance_dist(double dist, double vel) {
        double ang = dist/WHEEL_C;
        flmotor.move_relative(ang, vel);
        frmotor.move_relative(ang, vel);
        rlmotor.move_relative(ang, vel);
        rrmotor.move_relative(ang, vel);
        while (abs(flmotor.get_target_velocity()) > 1) {
            sens::update();
        }
        stop();
    }

    // turn angle
    inline void turn_to(double heading, double mult = 1) {
        sens::update();
        heading = angl_360(heading);
        while (abs(sens::rot-heading) > TURN_MINDIFF) {
            sens::update();
            double rotdiff = angl_180(heading-sens::rot)/TURN_MAXDIFF;
            rotdiff = min(1.0, rotdiff);
            turn(rotdiff*WHEEL_RPM*mult);
        }
        stop();
    }
    inline void turn_angl(double angle) {
        sens::update();
        turn_to(angl_360(sens::rot+angle));
    }

    // INITIALIZE
    inline bool did_init = false;
    inline bool need_sens_reset = false;
    /*
    Runs at the beginning of autonomous before any route.
    For any initializations that cannot occur during initialize(),
    such as calibrating moving parts.
    */
    inline void init() {
        did_init = true;
        // sensing
        if (need_sens_reset && pros::competition::is_autonomous()) {
            sens::reset();
        }
        need_sens_reset = false;
    }
}