#include "openhaldex.h"

float lock_target = 0;
float ped_value = 0;

float get_lock_target_adjustment(void)
{
    float target = 0;

    if (state.mode == MODE_5050)
    {
        if (ped_value >= state.ped_threshold || state.ped_threshold == 0)
        {
            return 100;
        }
        else
        {
            return 0;
        }
    }
    else if (state.mode == MODE_FWD)
    {
        return 0;
    }
    
    /* Find out which lockpoints we're between in terms of speed..
     * Look for the lockpoint above our current speed (lp_upper) */
    lockpoint lp_lower = state.custom_mode.lockpoints[0];
    lockpoint lp_upper = state.custom_mode.lockpoints[state.custom_mode.lockpoint_count - 1];

    for (int i = 0; i < state.custom_mode.lockpoint_count; i++)
    {
        if (vehicle_speed <= state.custom_mode.lockpoints[i].speed)
        {
            lp_upper = state.custom_mode.lockpoints[i];
            lp_lower = state.custom_mode.lockpoints[(i == 0) ? 0 : i - 1];
            break;
        }
    }

    /* Get the easy cases out the way first... */
    if (vehicle_speed <= lp_lower.speed)
    {
        return lp_lower.lock;
    }
    if (vehicle_speed >= lp_upper.speed)
    {
        return lp_upper.lock;
    }

    /* If locking at all... */
    if (ped_value >= state.ped_threshold || state.ped_threshold == 0)
    {
        /* Need to interpolate */
        float inter = (float)(lp_upper.speed - lp_lower.speed) / (float)(vehicle_speed - lp_lower.speed);

        target = lp_lower.lock + ((float)(lp_upper.lock - lp_lower.lock) / inter);
#ifdef STATE_DEBUG
        Serial.printf("lp_upper:%d@%d lp_lower:%d@%d speed:%d target=%0.2f\n",
                      lp_upper.lock, lp_upper.speed, lp_lower.lock, lp_lower.speed, vehicle_speed, target);
#endif
    }
    /* Else leave target at its initial value of 0. */

    return target;
}

uint8_t get_lock_target_adjusted_value(uint8_t value)
{
    if (state.mode == MODE_5050)
    {
        if (ped_value >= state.ped_threshold || state.ped_threshold == 0)
        {
            return value;
        }
        
        return 0;
    }
    else
    {
        // Potentially avoid doing math below..
        if (lock_target == 0)
        {
            return 0;
        }
        
        /*
         * Hackery to get the response closer to the target... we are trying to control the
         * Haldex as if it's linear.. but it's not. In future, I'd like to implement some sort
         * of feedback loop to trim the calculation being made here but this will do for now.
         */
        float target_fudge_factor = lock_target;
        target_fudge_factor = (target_fudge_factor / 2) + 20;
        
        return value * (target_fudge_factor / 100);
    }
}
