//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// sftardcal.c - Generic framework for calibrating or controlling a serial port
//               Arduino-based device.  For BSD, Linux, and Windows
//
// This is a framework header file.  Each version must implement the
// 'calibrate_loop()' function (or build sftardcal.c with STAND_ALONE defined)
// '#define' values are typically used to determine which version to build,
// based on the Makefile, and any additional source files that implement the
// 'calibrate_loop()' function.
//
// Company web site:  http://mrp3.com/   e-mail:  bobf@mrp3.com
// 
// COPYRIGHT:
//
// Copyright (c) 2011-2015 S.F.T. Inc. - all rights reserved
//
// for licensing and distribution, see 'sftardcal.c'


// conditionally including device-specific header files
#ifdef TESTER
#include "Tester.h"
#endif
#ifdef POWERSUPPLY
#include "PowerSupply.h"
#endif



#ifndef WIN32
// the HANDLE data type is there to help WIN32 compatibility
#define HANDLE int
#define HAVE_GETOPT /* TODO - use configure script? */
#endif // WIN32


#define MY_GETS_BUFSIZE 4096 /* way too big on purpose */


// option vars
extern const char *pIn;
extern const char *pApp;
extern int bRawFlag;


// config utilities
HANDLE altconconfig(HANDLE iFile); // for 'alt console' when it's a pipe (I want it UN-TRANSLATED)
HANDLE conconfig(HANDLE iFile); // NOTE:  returns same file handle in POSIX, new? handle in winders
void ttyconfig(HANDLE iFile, int iBaud, int iParity, int iBits, int iStop);
int do_options(int argc, char *argv[], char * envp[]);
void reset_arduino(HANDLE iFile);
void set_rts_dtr(HANDLE iFile, int bSet); // low-level code similar to 'reset_arduino'

// input handling utilities
char * my_gets(HANDLE iFile);
char * my_gets2(HANDLE iFile, unsigned int dwTimeout); // similar to my_gets but with timeout
int my_pollin(HANDLE iFile);
void my_flush(HANDLE iFile);
const char * my_ltrim(const char *pStr);

// timing/state utilities
void MySleep(unsigned int dwMsec); // must be 32-bit unsigned integer for 'dwMsec' parameter
unsigned int MyGetTickCount();     // unsigned int on 32-bit and 64-bit platforms will always be 32-bit
void MyGetsEchoOff(void);          // disable echo during my_gets (or my_flush) the next time only
int TimeIntervalExceeds(unsigned int dwStart, unsigned int dwMSec);
                                   // returns != 0 if MyGetTickCount exceeds specified time interval from 'dwStart'

// global option abstractors

int Verbosity(void);     // 0 = NONE, 1-n determine actual level.  < 0 not defined
int FactoryReset(void);  // force a factory reset
int QuitFlag(void);      // check for 'quit' flag (after user input, typically), soft shutdown
void SetQuitFlag(void);  // manually set the 'quit' flag


// calibration-related utilities
char * get_reply(HANDLE iFile, int iMaxDelay);
   // wait for reply up to time delay value. stops waiting on ANY input (blocks until I get LF) up to time limit
   // this will accept multi-line replies, and does not filter out the command if it's echoed

char * send_command_get_reply_with_timeout(HANDLE iFile, const char *szCommand, unsigned int dwTimeout, unsigned int dwRepeatTimeout);
  // this function is a little more sophisticated, repeats the command every 'dwRepeatTimeout' (when non-zero), waits up to 'dwTimeout'
  // milliseconds (can be zero for 'no wait', though this would be impractical) for a response, then returns.
  // filters out the command if it's echoed in the first part of the reply.  only returns first non-blank line.

char * send_command_get_multiline_reply_with_timeout(HANDLE iFile, const char *szCommand, unsigned int dwTimeout, unsigned int dwRepeatTimeout);
  // this function is a little more sophisticated, repeats the command every 'dwRepeatTimeout' (when non-zero), waits up to 'dwTimeout'
  // milliseconds (can be zero for 'no wait', though this would be impractical) for a response, then returns.
  // calls 'get_reply' internally and does not filter out the command if that's echoed.

char * send_command_get_reply(HANDLE iFile, const char *szCommand); // returns first non-echo line (calls above function internally with defaults)

int send_command_get_reply_OK(HANDLE iFile, const char *szCommand); // returns non-zero if first non-echo line is 'OK' (calls above)

char * ask_for_user_input(HANDLE iConsole, const char * szPrompt);
int ask_for_user_input_YN(HANDLE iConsole, const char * szPrompt, int iDefault);
int ask_for_user_input_double(HANDLE iConsole, const char * szPrompt, double *pdRval);
int ask_for_user_input_doubleV(HANDLE iConsole, const char * szPrompt, double *pdRval, double dMin, double dMax);


// 'main loop' processing for stand-alone version
void console_loop(HANDLE iFile, HANDLE iConsole);


// 'question' processing
void question_loop(HANDLE iFile, HANDLE iConsole);


// external source defines this, or else #define STAND_ALONE so that it's not needed
// consider defining as 'weak' and calling 'console_loop' when not present externally
// as long as it doesn't break the Microsoft DevStudio compiler
void calibrate_loop(HANDLE iFile, HANDLE iConsole);


// read/write abstractors (WIN32 help, basically)

int my_write(HANDLE iFile, const void *pBuf, int cbBuf);
int my_read(HANDLE iFile, void *pBuf, int cbBuf);


// debug dump - 'iDir < 0' is receive, 'iDir > 0' is send
void sftardcal_debug_dump_buffer(int iDir, const void *pBuf, int cbBuf);


#define VERBOSITY_SILENT      0
#define VERBOSITY_NORMAL      1
#define VERBOSITY_INFORMATIVE 2
#define VERBOSITY_CHATTY      3
#define VERBOSITY_GEEKY       4
#define VERBOSITY_OBNOXIOUS   5
#define VERBOSITY_VOMITOUS    6
#define VERBOSITY_WAYTOOMUCH  7
#define VERBOSITY_TOTHEMAX    8

