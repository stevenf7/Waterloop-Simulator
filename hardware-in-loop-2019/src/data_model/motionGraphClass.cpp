#include <math.h>

#include "motionGraphClass.hpp"
#include "motionGraphDef.hpp"

/**
 * Default Constructor
 */
MotionGraph::MotionGraph() : acceleration{0}, distance{0}, old_speed{0} {}

/**
 * set_steep_mid is a mutator function to set the values for
 *  steep and mid
 * @param steep: how dramatic the sigmoid function looks like
 * @param mid: the curve in the sigmoid graph
 */
void MotionGraph::set_steep_mid (double steep, double mid){
    k = steep;
    a = mid;
}

/**
 * sig_vel calculates the velocity based on the sigmoid function
 * @param time: the difference in time between functions call time
 *              and present time
 * @param forward: direction for the pod
 * @return change in velocity of the pod
 */
double MotionGraph::sig_vel(double time, bool forward) {
    if (!forward) {
        downSig();
    }
    double velocity  = l / (1 + exp(-k * (time - a)));
    upSig();

    return velocity;
}

/**
 * SIMPSON's RULE: area underneath velocity/time graph = distance
 * t1 and t2 are the time bounds needed for simpson's rule
 *
 * sig_dist calculates the distance underneath the curve
 * @param t1: initial time
 * @param t2: end time
 * @param forward: direction of the pod
 * @return distance
 */
double MotionGraph::sig_dist(double t1, double t2, bool forward) {
    //simpson's rule:
    double oddSum{0}, evenSum{0}, n{1000};
    double x{(t2 - t1) / n};

    for (int i = 1; i < n; i += 2) {
        if ((i + 1) * x + t1 != t2) {
            evenSum += 2 * sig_vel(((i + 1) * x) + t1, forward);
        }

        oddSum += 4 * sig_vel(i * x + t1, forward);
    }

    return (x / 3) * (evenSum + oddSum + sig_vel(t1, forward) + sig_vel(t2, forward));
}

/**
 * cruise method to update the distance of the pod
 * @param time_diff: the difference in time from last update
 */
void MotionGraph::cruise(double time_diff){
    distance += speed * time_diff;
    old_speed = speed;
    acceleration = 0;
}

/**
 * aceel is a method to calculate acceleration
 * @param start_time: time of the function call
 * @param time_diff: time difference from the last update
 * @param t: current time
 * @param increase: true for acceleration, false otherwise
 */
void MotionGraph::accel(double start_time,double time_diff, double t, bool increase){
    old_speed=speed;
    speed+=sig_vel(t-start_time, increase);
    acceleration=(speed-old_speed)/time_diff;
    distance+=sig_dist(t-time_diff,t, increase);
}

/**
 * Distance does not change
 *
 * stop method stops the pod
 * @param time_diff: the difference from the last update
 */
void MotionGraph::stop(double time_diff){
    old_speed = speed;
    speed = 0;
    acceleration = (speed - old_speed)/time_diff;
}

/**
 * downSig forces the steep to be negative
 */
void MotionGraph::downSig() {
    k = -fabs(k);
    a = SIG_A2;
}

/**
 * upSig forces the steep to be positive
 */
void MotionGraph::upSig() {
    k = fabs(k);
    a = SIG_A1;
}

/**
 * accessor for the acceleration
 * @return acceleration for the pod
 */
double MotionGraph::getAccel() {
    return acceleration;
}

/**
 * accessor for the distance
 * @return distance travelled by the pod
 */
double MotionGraph::getDistance() {
    return distance;
}

/**
 * accessor for the speed
 * @return speed of the pod
 */
double MotionGraph::getSpeed() {
    return speed;
}