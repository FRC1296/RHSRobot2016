/** \file
 * Tokens used in our scripting language
 */

#ifndef AUTOPARSER_H
#define AUTOPARSER_H

// any line in the parser file that begins with a space or a # is skipped

const char sComment = '#';
const char szDelimiters[] = " ,[]()";

///N - doesn't need a response; R - needs a response; _ - contained within auto thread
typedef enum AUTO_COMMAND_TOKENS
{
	AUTO_TOKEN_START_AUTO,			//!<R	send message to all components to set up for Autonomous
	AUTO_TOKEN_FINISH_AUTO,			//!<R	send message to all components that Autonomous is done
	AUTO_TOKEN_MODE,				//!<	mode block number, number(integer)
	AUTO_TOKEN_DEBUG,				//!<	debug mode, 0 = off, 1 = on
	AUTO_TOKEN_MESSAGE,				//!<	print debug message
	AUTO_TOKEN_BEGIN,				//!<	mark beginning of mode block
	AUTO_TOKEN_END,					//!<	mark end of mode block
	AUTO_TOKEN_DELAY,				//!<	delay (seconds - float)
	AUTO_TOKEN_MOVE,				//!<N	move (left & right PWM - float)
	AUTO_TOKEN_MMOVE,				//!<R	mmove (speed) (inches - float)
	AUTO_TOKEN_MLINE,				//!<R	mline (speed) (inches - float)
	AUTO_TOKEN_TURN,				//!<R	turn (degrees - float) (timeout)
	AUTO_TOKEN_STRAIGHT,			//!<R	straight drive (speed) (duration)
	AUTO_TOKEN_SEARCH,
	AUTO_TOKEN_AIM,
	AUTO_TOKEN_INTAKE,
	AUTO_TOKEN_INTAKESTOP,
	AUTO_TOKEN_RIDE,
	AUTO_TOKEN_LOWERINTAKE,
	AUTO_TOKEN_AFTERSHOOT,
	AUTO_TOKEN_THROWUP,
	AUTO_TOKEN_SHOOT,
	AUTO_TOKEN_SETANGLE,
	AUTO_TOKEN_LOWER,
	AUTO_TOKEN_RAISE,
	AUTO_TOKEN_TAILDOWN,
	AUTO_TOKEN_TAILUP,
	AUTO_TOKEN_REDSENSE,
	AUTO_TOKEN_START_DRIVE_FWD,
	AUTO_TOKEN_START_DRIVE_BCK,
	AUTO_TOKEN_STOP_DRIVE,
	AUTO_TOKEN_SHORT,
	AUTO_TOKEN_JAWOPEN,
	AUTO_TOKEN_JAWCLOSE,
	AUTO_TOKEN_LAST
} AUTO_COMMAND_TOKENS;

#endif  // AUTOPARSER_H

