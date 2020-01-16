#pragma once


class BTOL_Regulator {
public:
    //https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
    //https://www.learncpp.com/cpp-tutorial/8-5a-constructor-member-initializer-lists/
    BTOL_Regulator(float integratorStartingValue, float startingLastError, float minimumDeltaTime, float maximumDeltaTime)
        :_integratorValue(integratorStartingValue), _lastError(startingLastError), _minimumDeltaTime(minimumDeltaTime), _maximumDeltaTime(maximumDeltaTime) 
    {
       //assignments done via intalizations in member initalizer list


    }

float getTorqueDemand(float targetRate, float measuredRate, float deltaTime, float PCoef, float ICoef, float DCoef, float integratorMax, float rateDampingCoef, float momentOfInertia);
void throwWarning(int warningIdentifier);
void setIntegratorValue(float newIntegratorValue);
float getIntegratorValue(void);

private:
    float _integratorValue;
    float _lastError;
    float _minimumDeltaTime; //100Hz  IDK if i can set value here.
    float _maximumDeltaTime;
};
