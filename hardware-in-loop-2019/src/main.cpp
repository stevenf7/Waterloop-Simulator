#include "data_model/motionGraphClass.hpp"
#include "data_model/motionGraphDef.hpp"
#include "math.h"
#include "Cosa/Trace.hh"
#include "Cosa/UART.hh"

#include <CanPacket.h>
#include <CanPacketDriver.h>
#include <MCP2515.h>
#include <Cosa/MCP2515.h>

// Pre-defined Constants for the POD
int const START{10};
int const ACCELERATION{12};
int const CRUISE{13};
int const DECELERATION{14};
int const STOP{15};
int const STRIP_ID{29};
int const SPEED_ID{30};
int const TIME_ID{31};


int const MAX_ACCEL{15}; //TODO: determine maximum acceleration
using namespace wlp;

static cosa::MCP2515 base;
static MCP2515 bus(&base);
static uint8_t buf[8];

int oldID{};
int data{};
double time{0.0};
double const timeDiff{0.1};


/**
 * UART trace setup
 * Packet setup
 * bus setup
 */
void setup() {
    uart.begin(9600);
    trace.begin(&uart);
    while (bus.begin(CAN_500KBPS, MCP_16MHz) != Result::OK) {
        delay(100);
    }
    trace << "Receiver started\n";
    trace << "Sender started\n";

    wlp::Packet p{};
    wlp::packet::send(bus, p);
    wlp::packet::read(bus, p);
}





/**
 * compares the previous id with the current id
 * @param id: the current id
 * @param old_id: previous id
 * @return boolean value, true id different and false otherwise
 */
bool diffCommand(int id, int prevID) {

    if (id == prevID) {//same ID
        return false;
    }
    return true;
}

/**
 * Console Display for the pod parameters
 * @param time: time travelled since the beginning
 * @param model: the motion graph used to determine the
 *              speed, acceleration and distance
 */
void display(double time, MotionGraph model) {
    trace << "Time: " << time << "\n";
    trace << "Dist: " << model.getDistance() << "\n";
    trace << "Accel: " << model.getAccel() << "\n";
    trace << "Speed: " << model.getSpeed() << "\n\n";
}

/**
 * Outputs the stripe data based on the distance &
 *        outputs speed and time
 *
 * @param distance: the pod has travelled
 * @param speed: the speed of the pod
 * @param time: the time travelled since it started
 */

//TODO:need to do output
void output(double distance, double speed, double time) {

    if (static_cast<int>(distance) % 200 == 0) {
        trace << "Stripe \n";
        wlp::Packet strip(STRIP_ID);
        wlp::packet::send(bus, strip);
    }
        wlp::Packet velocity(SPEED_ID, (float) speed);
        wlp::packet::send(bus, velocity);
    // Send the velocity, time back to RPi
        wlp::Packet second(TIME_ID, (float) time );
        wlp::packet::send(bus, second);

}



void loop() {

    int id{};

    double startTime{};

    trace << "SIMULATION STARTED..." << endl;

    // TODO: Update the input from the canbus packet
    wlp::Packet packet{};

    do {
        wlp::packet::read(bus, &packet);
        id = (int)packet.id();
        data = (int)packet.data();
    } while (id != START);

    oldID = START;

    // TODO Actual values for steep and mid
    double const STEEP{5.3};
    double const MID{1.1};
    MotionGraph model{};

    model.set_steep_mid(STEEP, MID);

    double speed = model.getSpeed();
    double distance = model.getDistance();
    double acceleration = model.getAccel();

    while (true) {
        if (bus.get_message_status() == MessageState::MessagePending) {
            wlp::packet::read(bus, &packet);
            id = (int)packet.id();
            data = (int)packet.data();
            if (diffCommand(id, oldID)) {
                startTime = time;
            }

            switch(id) {
                case ACCELERATION: model.accel(startTime, timeDiff, time, true);
                    break;
                case CRUISE: model.cruise(timeDiff);
                    break;
                case DECELERATION: model.accel(startTime, timeDiff, time, false);
                    break;
                case STOP: model.stop(timeDiff);
                    break;
                default: trace << "Input must be any of ACCELERATION, CRUISE, DECELERATION, STOP";
                    break;
            }

            display(time, model);

            time += timeDiff;

            int const TIMEOUT{10};

            if (time > TIMEOUT) {
                break;
            } else if (distance > TRACK_LENGTH) {
                trace << "POD CRASHED \n";
                break;
            } else if (speed > MAX_VEL) {
                trace << "WARNING: POD OVERSPEEDING \n";
            }
            if (fabs(acceleration)> MAX_ACCEL){
                trace << "WARNING: OVER ACCELERATION \n";
            }

            output(distance, speed, time);
            oldID = id; //update

        } else {
            id = oldID; //default to previous ID

            switch(id) {
                case ACCELERATION: model.accel(startTime, timeDiff, time, true);
                    break;
                case CRUISE: model.cruise(timeDiff);
                    break;
                case DECELERATION: model.accel(startTime, timeDiff, time, false);
                    break;
                case STOP: model.stop(timeDiff);
                    break;
                default: trace << "Input must be any of ACCELERATION, CRUISE, DECELERATION, STOP";
                    break;
            }

            time += timeDiff;
            trace << "WARNING: MISSED SIGNAL FROM CONTROLLER \n";

            display(time, model);
            output(distance, speed, time);

            int const TIMEOUT{10};

            if (time > TIMEOUT) {
                break;
            } else if (distance > TRACK_LENGTH) {
                trace << "POD CRASHED \n";
                break;
            } else if (speed > MAX_VEL) {
                trace << "WARNING: POD OVERSPEEDING \n";
            }
            if (fabs(acceleration)> MAX_ACCEL){
                trace << "WARNING: OVER ACCELERATION \n";
            }

        }
    }
}
