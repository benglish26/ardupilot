#include "btolRegulator.h"
#include <math.h>
//#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>  //for the warning debug.
//extern const AP_HAL::HAL& hal;
#include <stdio.h>

void BTOL_Regulator::throwWarning(int warningIdentifier)
{
    char text[3];
    snprintf(text,3,"W%d",warningIdentifier);
    gcs().send_text(MAV_SEVERITY_CRITICAL, text);
}

void BTOL_Regulator::setIntegratorValue(float newIntegratorValue)
{
    if(isfinite(newIntegratorValue))
    {
        _integratorValue = newIntegratorValue;
    }else{
        throwWarning(4);
    }
}
float BTOL_Regulator::getIntegratorValue(void)
{
    return _integratorValue;
}

//integratorMax must be positive!
float BTOL_Regulator::getTorqueDemand(float targetRate, float measuredRate, float deltaTime, float PCoef, float ICoef, float DCoef, float integratorMax, float rateDampingCoef, float momentOfInertia)
{
   //https://www.learncpp.com/cpp-tutorial/8-5a-constructor-member-initializer-lists/

    if (!isfinite(targetRate) || !isfinite(measuredRate) || !isfinite(deltaTime) || !isfinite(PCoef) || !isfinite(ICoef) || !isfinite(DCoef) || !isfinite(integratorMax) || !isfinite(rateDampingCoef) || !isfinite(momentOfInertia)) {
        //AP::internalerror().error(AP_InternalError::error_t::constraining_nan);
        //TODO: Throw warning.
        throwWarning(1);
        return 0.0f;
    }

    //Sanitize input values.
    if(deltaTime < _minimumDeltaTime)
    {
        deltaTime = _minimumDeltaTime;
        throwWarning(2);
    }

    if(deltaTime > _maximumDeltaTime)
    {
        deltaTime = _maximumDeltaTime;
        throwWarning(3);
    }

    if(integratorMax < 0.0f) integratorMax = 0.0f;



     //calculate the feed-forward term which should oppose the damping we expect to encounter at the target rate.
    float feedForwardTorque = -1.0f * (rateDampingCoef * targetRate);  ///This needs to be an acceleration to work in this function...we could change the definition of the coeficents, or move rotational moment of inertia in here...

    float error = targetRate-measuredRate;
    float derivative = (error - _lastError) / deltaTime;
    _integratorValue += error * deltaTime;

    if(_integratorValue > integratorMax) _integratorValue = integratorMax;
    if(_integratorValue < -integratorMax) _integratorValue = -integratorMax;
    _integratorValue = 0.0f;//constrain_float(integratorValue, -integratorMax, integratorMax);


    float proportionalContribution = PCoef * error;
    float integralContribution = ICoef * _integratorValue;  //I'm not using a reset time or reset rate so we can turn off this term if needed.
    float derivitiveContribution = DCoef * derivative;
    float regulatorAccelerationContribution = proportionalContribution + integralContribution + derivitiveContribution;

    float regulatorTorqueContribution = momentOfInertia * regulatorAccelerationContribution;

    float torqueDemand = feedForwardTorque + regulatorTorqueContribution;

    //For logging:
    _lastRegulatorTarget = targetRate;
    _lastRegulatorEstimate = measuredRate;
    _lastRegulatorError = error;
    _lastRegulatorPcoef = PCoef;
    _lastRegulatorIcoef = ICoef;
    _lastRegulatorDcoef = DCoef;
    _lastRegulatorP = proportionalContribution;
    _lastRegulatorD = derivitiveContribution;
    _lastRegulatorI = integralContribution;
    _lastRegulatorAccelerationContribution = regulatorAccelerationContribution;
    _lastRegulatorTorqueContribution = regulatorTorqueContribution;
    _lastRegulatorRateDampingCoef = rateDampingCoef;
    _lastRegulatorFFTorqueDemand = feedForwardTorque;
    _lastRegulatorTorqueDemand = torqueDemand;



    //For Dterm.
    _lastError = error;
    return torqueDemand;
}