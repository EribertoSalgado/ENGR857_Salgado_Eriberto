#if !defined(_quanser_game_controller_h)
#define _quanser_game_controller_h

#include "quanser_packing.h"

typedef struct tag_game_controller_states
{
    t_single x;                 /* x-coordinate as a percentage of the range. Spans -1.0 to 1.0 */
    t_single y;                 /* y-coordinate as a percentage of the range. Spans -1.0 to 1.0 */
    t_single z;                 /* z-coordinate as a percentage of the range. Spans -1.0 to 1.0 */
    t_single rx;                /* rx-coordinate as a percentage of the range. Spans -1.0 to 1.0 */
    t_single ry;                /* ry-coordinate as a percentage of the range. Spans -1.0 to 1.0 */
    t_single rz;                /* rz-coordinate as a percentage of the range. Spans -1.0 to 1.0 */
    t_single sliders[2];        /* sliders as a percentage of the range. Spans 0.0 to 1.0 */
    t_single point_of_views[4]; /* point-of-view positions (in positive radians or -1 = centred). */
    t_uint32 buttons;           /* state of each of 32 buttons. If the bit corresponding to the button is 0 the button is released. If it is 1 then the button is pressed */
} PACKED_ATTRIBUTE t_game_controller_states;

#endif
