#ifndef GAIT_SCHEDULER_CODES
#define GAIT_SCHEDULER_CODES

// --content--
#define TAKE_STEP 10
#define DRAW_BACK 11
#define SHIFT     12
#define MINIMUM_PAUSE               15


#define STRIDE_LENGTH_INDEX         0
#define GAIT_AMPLITUDE_INDEX        1
#define GAIT_DRAW_BACK_FACTOR_INDEX 2
#define STEP_DURATION_INDEX         3
#define STEP_COUNT_INDEX            4
#define PAUSE_DURATION_INDEX        5
#define SCHEDULE_ACTIONS_INDEX      6
#define MAX_STEPS                   25

// Note that MINIMUM_PAUSE has a '1' prepended to it for parsing. If the number is 15, then it is actually 5.

#endif