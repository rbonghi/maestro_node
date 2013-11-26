/* 
 * File:   ROSPololuController.cpp
 * Author: raffaello
 * 
 * Created on 25 November 2013, 10:14
 */

#include "PololuController.h"

using namespace std;
using namespace boost;

#define MIN_ANGLE_RAD -M_PI/2
#define MIN_VALUE 3712
#define MIN_VALUE_PULSE 992
#define MAX_ANGLE_RAD M_PI/2
#define MAX_VALUE 8000
#define MAX_VALUE_PULSE 2000
#define MEAN_VALUE_PULSE (((MAX_VALUE_PULSE - MIN_VALUE_PULSE)/2 + MIN_VALUE_PULSE))

PololuController::PololuController(TimeoutSerial* serial) : serial(serial) {
}

PololuController::~PololuController() {
}

void PololuController::maestroSetAngle(int channel, double target) {
    double value = target * ((MAX_VALUE_PULSE - MEAN_VALUE_PULSE) / ((double) MAX_ANGLE_RAD)) + MEAN_VALUE_PULSE;
    maestroSetTarget(channel, value);
}

double PololuController::maestroGetAngle(int channel) {
    double return_value = maestroGetTarget(channel);
    return (return_value - MEAN_VALUE_PULSE) * (MAX_ANGLE_RAD / ((double) (MAX_VALUE_PULSE - MEAN_VALUE_PULSE)));
}

void PololuController::maestroSetTarget(int channel, double target) {
    lock_guard<mutex> l(pololuMutex);
    vector<char> list;
    int convert_target = target * (MAX_VALUE / ((double) MAX_VALUE_PULSE));
    list.push_back(0x84); // Command byte: Set Target.
    list.push_back(channel); // First data byte holds channel number.
    list.push_back(convert_target & 0x7F); // Second byte holds the lower 7 bits of target.
    list.push_back((convert_target >> 7) & 0x7F); // Third data byte holds the bits 7-13 of target.
    serial->write(list);
}

double PololuController::maestroGetTarget(int channel) {
    lock_guard<mutex> l(pololuMutex);
    vector<char> list;
    list.push_back(0x90); // Command byte: Set Target.
    list.push_back(channel); // First data byte holds channel number.
    serial->write(list);
    //Response data
    list = serial->read(2);
    double value = list[0] + 256 * list[1];
    return value * (MAX_VALUE_PULSE / ((double) MAX_VALUE));
}