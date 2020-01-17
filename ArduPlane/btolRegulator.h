#pragma once


class BTOL_Regulator {
public:
    //https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
    //https://www.learncpp.com/cpp-tutorial/8-5a-constructor-member-initializer-lists/
    BTOL_Regulator(float integratorStartingValue, float startingLastError, float minimumDeltaTime, float maximumDeltaTime)
        :_integratorValue(integratorStartingValue), _lastError(startingLastError), _minimumDeltaTime(minimumDeltaTime), _maximumDeltaTime(maximumDeltaTime),
        _lastRegulatorTarget(0.0f),
        _lastRegulatorEstimate(0.0f),
        _lastRegulatorError(0.0f),
        _lastRegulatorPcoef(0.0f),
        _lastRegulatorIcoef(0.0f),
        _lastRegulatorDcoef(0.0f),
        _lastRegulatorP(0.0f),
        _lastRegulatorD(0.0f),
        _lastRegulatorI(0.0f),
        _lastRegulatorAccelerationContribution(0.0f),
        _lastRegulatorTorqueContribution(0.0f),
        _lastRegulatorRateDampingCoef(0.0f),
        _lastRegulatorFFTorqueDemand(0.0f),
        _lastRegulatorTorqueDemand(0.0f)
    {
       //assignments done via intalizations in member initalizer list



    }

float getTorqueDemand(float targetRate, float measuredRate, float deltaTime, float PCoef, float ICoef, float DCoef, float integratorMax, float rateDampingCoef, float momentOfInertia);
void throwWarning(int warningIdentifier);
void setIntegratorValue(float newIntegratorValue);
float getIntegratorValue(void);

float _lastRegulatorTarget;
float _lastRegulatorEstimate;
float _lastRegulatorError;
float _lastRegulatorPcoef;
float _lastRegulatorIcoef;
float _lastRegulatorDcoef;
float _lastRegulatorP;
float _lastRegulatorD;
float _lastRegulatorI;
float _lastRegulatorAccelerationContribution;
float _lastRegulatorTorqueContribution;
float _lastRegulatorRateDampingCoef;
float _lastRegulatorFFTorqueDemand;
float _lastRegulatorTorqueDemand;

private:
    float _integratorValue;
    float _lastError;
    float _minimumDeltaTime; //100Hz  IDK if i can set value here.
    float _maximumDeltaTime;
};
