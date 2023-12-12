//add constant from libmd25 use in main
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

/*****************************
 *         PARAMETERS        *
 *****************************/

// At the moment, we keep floats as we are not (yet) in optimization mode

#define TICKS_PER_REVOLUTION 4096 //360    // Nb ticks per wheel revolution
#define WHEEL_DIAM 68               // Diameter of the wheel (mm) UNUSED !!!
#define DIST_PER_REVOLUTION 216.2695 // 217.879 // 214.66 // 206.5//214.635// 210.481 //304.734 // Distance traveled for a full 	wheel revolution (in mm)
#define TICKS_PER_DEG 91.472 // 91.290 // 91.108345 // 91.001144 // 91.055 	// 89.28//89//89.06 //4.86          // Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg (old value: 4.7)
#define TICKS_OVERFLOW 65536
#define TICKS_half_OVERFLOW TICKS_OVERFLOW/2
#endif
