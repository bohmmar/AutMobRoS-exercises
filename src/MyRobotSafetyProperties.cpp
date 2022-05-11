#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),
    
      slSystemOff("System is offline"),
      slSystemOn("System is online"),
      slShuttingDown("System shutting down"),
      slBraking("System braking"),
      slStartingUp("System starting up"),
      slEmergency("Emergency"),
      slEmergencyBraking("System braking"),
      slMotorPowerOn("Motors powered"),
      slSystemMoving("System moving"),


      doSystemOn("Startup the system"),
      doSystemOff("Shutdown the system"),
      abort("Abort"),
      shutdown("Shutdown"),
      systemStarted("System started"),
      motorsHalted("Motors halted"),
      startMoving("Start moving"),
      stopMoving("Stop moving"),
      emergency("Emergency"),
      resetEmergency("Reset emergency"),
      powerOff("Power off"),
      powerOn("Power on"),

{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    // ... = hal.getLogicOutput("...");
    greenLED = hal.getLogicOutput("onBoardLEDgreen");
    redLED = hal.getLogicOutput("onBoardLEDred");

    // criticalOutputs = { ... };
    criticalOutputs = {greenLED, redLED};

    // Declare and add critical inputs
    // ... = eeros::hal::HAL::instance().getLogicInput("...", ...);
    buttonPause = eeros::hal::HAL::instance().getLogicInput("onBoardButtonPause");
    buttonMode = eeros::hal::HAL::instance().getLogicInput("onBoardButtonMode");


    // criticalInputs = { ... };
    criticalInputs = {buttonPause, buttonMode};

    // Add all safety levels to the safety system
    addLevel(slSystemOff);
    addLevel(slShuttingDown);
    addLevel(slBraking);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBraking);
    addLevel(slSystemOn);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slSystemOn, kPublicEvent);
    slShuttingDown.addEvent(shutdown, slSystemOff , kPrivateEvent);
    slBraking.addEvent(motorsHalted, slShuttingDown, kPrivateEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPrivateEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPrivateEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPrivateEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slSystemMoving.addEvent(abort, slBraking, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergency, kPublicEvent);

    // Add events to multiple safety levels
    // addEventToAllLevelsBetween(lowerLevel, upperLevel, event, targetLevel, kPublicEvent/kPrivateEvent);
    addEventToAllLevelsBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelsBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);

    // Define input actions for all levels
    // level.setInputActions({ ... });
    slSystemOff.setInputActions({                ignore(buttonPause),                       ignore(buttonMode) });
    slShuttingDown.setInputActions({             ignore(buttonPause),                       ignore(buttonMode) });
    slBraking.setInputActions({                  ignore(buttonPause),                       ignore(buttonMode) });
    slStartingUp.setInputActions({               ignore(buttonPause),                       ignore(buttonMode) });
    slEmergency.setInputActions({                ignore(buttonPause),                       check(buttonMode, false, resetEmergency) });
    slEmergencyBraking.setInputActions({         ignore(buttonPause),                       ignore(buttonMode) });
    slSystemOn.setInputActions({                 check(buttonPause, false, emergency),      ignore(buttonMode) });
    slSystemMoving.setInputActions({             check(buttonPause, false, emergency),      ignore(buttonMode) });
    slSystemOff.setInputActions({                check(buttonPause, false, emergency),      ignore(buttonMode) });


    // Define output actions for all levels
    // level.setOutputActions({ ... });
    slSystemOff.setOutputActions({                  set(greenLED, false),       set(redLED, false) });
    slShuttingDown.setOutputActions({               set(greenLED, false),       set(redLED, true) });
    slBraking.setOutputActions({                    set(greenLED, false),       set(redLED, true) });
    slStartingUp.setOutputActions({                 set(greenLED, true),        set(redLED, false) });
    slEmergency.setOutputActions({                  set(greenLED, true),        set(redLED, true) });
    slEmergencyBraking.setOutputActions({           set(greenLED, true),        set(redLED, true) });
    slSystemOn.setOutputActions({                   set(greenLED, true),        set(redLED, false) });
    slSystemMoving.setOutputActions({               set(greenLED, true),        set(redLED, false) });
    slSystemOff.setOutputActions({                  set(greenLED, true),        set(redLED, false) });

    // Define and add level actions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.stop();
        eeros::Executor::stop();
    });

    slShuttingDown.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.stop();
        privateContext->triggerEvent(shutdown);
    });

    slBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing sill
        privateContext->triggerEvent(motorsHalted);
    });

    slStartingUp.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.start();
        privateContext->triggerEvent(systemStarted);
    });

    slEmergency.setLevelAction([&](SafetyContext *privateContext) {
        
    });

    slEmergencyBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing still
        privateContext->triggerEvent(motorsHalted);
    });

    slSystemOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemOn.getNofActivations()*dt >= 1)   // wait 1 sec
        {
            privateContext->triggerEvent(powerOn);
        }
    });

    slMotorPowerOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slMotorPowerOn.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(startMoving);
        }
    });

    slSystemMoving.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemMoving.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(stopMoving);
        }
    });

    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(abort);
    });
}
