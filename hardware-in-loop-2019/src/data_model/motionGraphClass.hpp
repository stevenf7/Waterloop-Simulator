#ifndef MOTION_GRAPH_CLASS_H
#define MOTION_GRAPH_CLASS_H

class MotionGraph {
private:
    double l, k, a;
    double distance, acceleration, speed;
    double old_speed;

    double sig_vel(double t, bool up);

    double sig_dist(double t1, double t2, bool up);

    void downSig();

    void upSig();


public:
    MotionGraph();

    void set_steep_mid (double steep, double mid);

    void stop (double time_diff);

    void accel (double start_time,double time_diff, double t, bool increase);

    void cruise (double time_diff); // time difference between each loop occurrence

    double getSpeed();

    double getAccel();

    double getDistance();

};

#endif

