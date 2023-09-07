#ifndef __COMMAND_TYPE_HPP__
#define __COMMAND_TYPE_HPP__

typedef union{
    struct{
        float x;
        float y;
        float z;
        float yaw;
    };
    float state[4];
} Satate_t; 

typedef struct{
    bool is_arm;
    bool is_joy;
    bool is_press;
} Flag_t;

typedef enum {STATE, ALL_DATA} Data_t;


#endif