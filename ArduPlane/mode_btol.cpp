#include "mode.h"
#include "Plane.h"

//Copied from Stablized.

bool ModeBTOL::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeBTOL::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

