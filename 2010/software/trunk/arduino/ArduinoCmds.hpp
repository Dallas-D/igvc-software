#ifndef ARDUINOCMDS_HPP_
#define ARDUINOCMDS_HPP_

#define PACKET_ERROR_CMD 0xFF

// MC ENUMS
// Enums for data settings
enum mc_opttype_t { MC_PUSHPULL = 0, MC_RET_T, MC_INTEROG_DL, MC_SETCLK, MC_RESENDPKT};//interogdl - miliseconds
enum mc_opt_t {MC_PUSH = 0, MC_PULL, MC_SEND_DTICK, MC_SEND_CURRENT};

//enum for general errors
enum arduino_error_t {DROPPED_PACKET, REQUESTED_PACKET_OUT_OF_RANGE};

//enum for sonar options
enum sonar_opttype_t { SN_SET_RNG = 0, SN_SET_FREQ, SN_SET_GAIN, SN_SET_WARN,  };

//IGVC2010 GENERAL COMMANDS
//enum arduino_general_cmd_t {ARDUINO_RESET, ARDUINO_SET_CLOCK};

//IGVC2010 MOTOR COMMANDS -- response echos this command val
//enum mc_cmd_t {MC_GET_ENCODER_TICK, MC_GET_CURRENT_VAL, MC_SET_R_SPEED, MC_SET_L_SPEED};

//IGVC2010 GENERAL COMMANDS
const byte ARDUINO_RESET		= 0x00;
const byte ARDUINO_SET_CLOCK		= 0x01;
const byte ARDUINO_GET_ID		= 0x02;
const byte ARDUINO_HALT_CATCH_FIRE	= 0x03;
const byte MC_GET_ENCODER_TICK		= 0x04;
const byte MC_GET_RL_CURR_VAL		= 0x05;
const byte MC_GET_R_CURR_VAL		= 0x06;
const byte MC_GET_L_CURR_VAL		= 0x07;
const byte MC_SET_R_SPEED		= 0x08;
const byte MC_SET_L_SPEED		= 0x09;
const byte MC_SET_RL_DUTY_CYCLE		= 0x0A;

//IGVC2010 Board IDs
const byte OSMC_IF_BOARD 	= 0x00;


//1 byte commands, laptop -> arduino
#define ARDUINO_GETSTATUS_CMD 'r'
#define ARDUINO_SETVAR_CMD 'w'
#define ARDUINO_ID_CMD ARDUINO_GET_ID
#define ARDUINO_RSND_PK_CMD 'p'

//1 byte commands/response arduino -> laptop
#define ARDUINO_ERROR_RESP 0xFF
#define ARDUINO_UNKOWN_CMD_RESP 0xFE

#define ARDUINO_GETSTATUS_RESP 'r'
#define ARDUINO_SETVAR_RESP 'w'
#define ARDUINO_ID_RESP ARDUINO_GET_ID
#define ARDUINO_RSND_PK_RESP 'p'

#endif //ARDUINO_COMMANDS_HPP_