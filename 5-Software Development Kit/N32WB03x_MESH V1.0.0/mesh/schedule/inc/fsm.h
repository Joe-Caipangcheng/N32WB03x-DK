#ifndef MESH_FSM_H__
#define MESH_FSM_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


#define FSM_DEBUG (0)


typedef uint8_t fsm_state_id_t;
typedef uint8_t fsm_event_id_t;
typedef uint8_t fsm_guard_id_t;
typedef uint8_t fsm_action_id_t;
typedef struct
{
    fsm_event_id_t	event_id;        /**< FSM Event ID */
    fsm_guard_id_t	guard_id;        /**< FSM Guard ID */
    fsm_action_id_t	action_id;       /**< FSM Action ID */
    fsm_state_id_t	new_state_id;    /**< New state ID */
} fsm_transition_t;
#define FSM_TRANSITION(event_id, guard_id, action_id, new_state_id)	\
                     {(event_id), (guard_id), (action_id), (new_state_id)}

#define FSM_STATE(state_id)   {(state_id) | FSM_STATE_FLAG, 0, 0, 0}
#define FSM_NO_GUARD        0xFF
#define FSM_OTHERWISE       0xFF
#define FSM_ALWAYS          0xFF
#define FSM_NO_ACTION       0xFF
#define FSM_SAME_STATE      0xFF
#define FSM_ANY_STATE      0xFF
#define FSM_STATE_FLAG      0x80
typedef bool (* fsm_guard_t)(fsm_guard_id_t guard_id, void * p_data);
typedef void (* fsm_action_t)(fsm_action_id_t action_id, void * p_data);
typedef struct
{
    const fsm_transition_t * transition_table;
    uint8_t                  transitions_count;
    fsm_state_id_t           initial_state;
    fsm_guard_t              guard;
    fsm_action_t             action;
    
#if FSM_DEBUG
    /** Pointer to the string with fsm name.
     */
    const char *             fsm_name;

    /** Pointer to the array with action name strings.
     */
    const char **            action_lookup;

    /** Pointer to the array with guard name strings.
     */
    const char **            guard_lookup;

    /** Pointer to the array with event name strings.
     */
    const char **            event_lookup;

    /** Pointer to the array with state name strings.
     */
    const char **            state_lookup;
#endif    
    
} fsm_const_descriptor_t;

typedef struct
{
    const fsm_const_descriptor_t * fsm_const_desc;
    uint8_t                        any_state_transitions_index;
    volatile fsm_state_id_t        current_state;
    volatile uint8_t               recursion_protection;
} fsm_t;

void fsm_init(fsm_t * p_fsm, const fsm_const_descriptor_t * p_fsm_const);
void fsm_event_post(fsm_t * p_fsm, fsm_event_id_t event_id, void * p_data);
bool fsm_is_processing(const fsm_t * p_fsm);

#endif // MESH_FSM_H__
