#ifndef MyRobotSAFETYPROPERTIES_HPP_
#define MyRobotSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include "ControlSystem.hpp"

class MyRobotSafetyProperties : public eeros::safety::SafetyProperties
{
public:
    MyRobotSafetyProperties(ControlSystem &cs, double dt);

    // Define all possible events
    eeros::safety::SafetyEvent doSystemOff;
    eeros::safety::SafetyEvent doSystemOn;
    eeros::safety::SafetyEvent abort;
    eeros::safety::SafetyEvent shutdown;
    eeros::safety::SafetyEvent systemStarted;
    eeros::safety::SafetyEvent motorsHalted;
    eeros::safety::SafetyEvent startMoving;
    eeros::safety::SafetyEvent stopMoving;
    eeros::safety::SafetyEvent emergency;
    eeros::safety::SafetyEvent resetEmergency;
    eeros::safety::SafetyEvent powerOff;
    eeros::safety::SafetyEvent powerOn;


    // Define all possible levels
    eeros::safety::SafetyLevel slSystemOff;
    eeros::safety::SafetyLevel slShuttingDown;
    eeros::safety::SafetyLevel slBraking;
    eeros::safety::SafetyLevel slStartingUp;
    eeros::safety::SafetyLevel slEmergency;
    eeros::safety::SafetyLevel slEmergencyBraking;
    eeros::safety::SafetyLevel slSystemOn;
    eeros::safety::SafetyLevel slMotorPowerOn;
    eeros::safety::SafetyLevel slSystemMoving;

private:
    // Define all critical outputs
    // eeros::hal::Output<bool>* ...;
    eeros::hal::Output<bool>* greenLED;
    eeros::hal::Output<bool>* redLED;

    // Define all critical inputs
    // eeros::hal::Input<bool>* ...;
    eeros::hal::Input<bool>* buttonPause;
    eeros::hal::Input<bool>* buttonMode;

    ControlSystem &cs;
};

#endif // MyRobotSAFETYPROPERTIES_HPP_
