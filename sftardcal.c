// sftardcal.c - Generic framework for calibrating or controlling a serial port
//               Arduino-based device.  For BSD, Linux, Cygwin, and Windows
//
// This is a framework application file.  Each version must implement the
// 'calibrate_loop()' function (or build sftardcal.c with STAND_ALONE defined)
// '#define' values are typically used to determine which version to build,
// based on the Makefile, and any additional source files that implement the
// 'calibrate_loop()' function.
//
// Company web site:  http://mrp3.com/   e-mail:  bobf@mrp3.com
//
// COPYRIGHT:
//
// Copyright (c) 2011-2013 S.F.T. Inc. - all rights reserved
//
// WARRANTY:
//
// There is NO WARRANTY, implied or otherwise.  This software is being
// supplied "as-is" and updates may be made without notification to you
// at any time.  You can expect to have to fix this for your platform
// and be pleasantly surprised if it works without any changes.  Some
// features may be incomplete, marked with 'TODO' and/or skeleton code.
// Your mileage may vary.  In other words, use it at your OWN risk.
//
// LICENSE TERMS:
//
// This code is available under a DUAL LICENSE, your choice of which one.
// You may either choose a BSD-like license, or GPLv2 (or later).
//
// BSD LICENSE
// You may copy, build, execute, reverse-engineer, derive work from,
// and/or re-distribute this source file (along with its corresponding
// header file) to any 3rd party you choose, provided that
//
// a) You include the original copyright, warrantee, and license terms
//    in their entirety
// b) You do not re-license this source in any manner inconsistent with
//    the copyright or license terms.
//
// You may ALSO produce a 'closed-source' (binary only) work that is
// either derived or entirely copied from this work, provided that you
// include a statement similar to:
//     Derived from sftardcal, Copyright 2011-2013 S.F.T. Inc.
// in a prominent place within your software license or documentation.
// This may include 'help' or 'about' displays that are activated by user
// action, or on application startup.
//
//
// GPLv2 (or later)
//
// Please see included 'gpl-2.0.txt' for a copy of version 2.0 of this license.
// Also you can view it at https://www.gnu.org/licenses/gpl-2.0.txt
//




#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#ifdef WIN32
#include <Windows.h> // generic windows defines
#include <process.h>
#else // !WIN32
#include <sys/time.h>
#include <arpa/inet.h>
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
//#ifdef __FreeBSD__
//#include <sys/ttycom.h> // stuff I need for IOCTLs etc.
//#else // __FreeBSD__ // assume Linux
#include <sys/ioctl.h> // linux needs this instead
//#endif // __FreeBSD__
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/types.h>
//#include <netinet6/in6.h> // sockaddr_in6 and ipv6-related stuff
#include <netinet/in.h> // sockaddr_in
#endif // WIN32

#include "sftardcal.h"

#ifdef WITH_XMODEM
#define SFTARDCAL
#include "xmodem.c"
#endif // WITH_XMODEM

#define DEFAULT_RESET_WAIT 5
//#define LINUX_SPECIAL_HANDLING

// DEFAULT SERIAL CONFIGURATION:  9600 baud, n, 8, 1 using argv[1] or /dev/ttyU0 as the input
// option vars
#ifdef WIN32
const char *pIn = "\\\\.\\COM1"; // winders needs something different, assume COM1 for now
#elif defined(__FreeBSD__)
const char *pIn = "/dev/ttyU0";
#elif defined(__gnu_linux__) && defined(__APCS_32__) && defined(__ARM_ARCH_6__) && defined(__ARMEL__)
// assume raspberry pi with raspbian using BCM2835 which is ARMv6 with VFP (uname shows as armv6l)
// for now I don't check for the presence of __VFP_FP__ nor __ARM_PCS_VFP
const char *pIn = "/dev/ttyAMA0"; // default serial on RPi with raspbian
#else // Linux assumed here
const char *pIn = "/dev/ttyACM0";
#endif // WIN32
const char *pApp;
int bRawFlag = 0;
HANDLE iStdOut; // note that for non-WIN32 I define 'HANDLE' as 'int'
#ifdef WIN32

#define pAltConsole FALSE

#define COMM_RW_EV EV_ERR | EV_RXCHAR | EV_TXEMPTY
#define COMM_RO_EV EV_ERR | EV_RXCHAR

#else // WIN32
char *pAltConsole = NULL;
#endif // WIN32

// default serial parameters
static char szBaud[256]="9600,n,8,1"; // baud rate expression, default is 9600,n,8,1
#ifdef WITH_XMODEM
static char szXModemFile[512]="";
#endif // WITH_XMODEM

static int iExperimental = 0;
static int iVerbosity = 0, bFactoryReset = 0, bLocalEcho = 0, bSerialDebug=0, bListenMode=0, bIsTCP=0,
           iResetWait=0, iFlowControl=0, bQuietFlag = 0;
static int iTerminator = 0; // CRLF [default]
#ifdef WITH_XMODEM
static int bXModemFlag=0;
#endif // WITH_XMODEM
static int bQuitFlag = 0; // assign to non-zero to force app to exit (must test for it after input)
static int bCommandRepeatOnTimeoutFlag = 1; // default is to repeat a command every second until it 'takes'

static char *pszQuestion = NULL; // not null to ask a question, get reply, and exit
static int iQuestionWait=5000; // default question wait time

// other internal (semi-global) flags

static int bMyGetsEchoFlag = 1;

// SAMPLE COMMANDS
// to pipe /dev/ttyU0 to/from an EXISTING unix socket,
//   sftardcal -c /var/run/unix_socket /dev/ttyU0

static void ttyconfigSTR(HANDLE iFile, char *szBaud);
#ifdef WITH_XMODEM
static void do_xmodem(HANDLE iFile, HANDLE iConsole); // internal XMODEM functionality
#endif // WITH_XMODEM

// console restore and 'alt console' - non-WIN32 only
#ifndef WIN32
static HANDLE hIOSConsoleRestoreHandle = -1;
static struct termios sIOSConsole;

void conrestore(void);
int configure_alt_console(HANDLE *piConsole); // NOTE:  'piConsole' must be a valid pointer
#endif // !WIN32

int do_main(int argc, char *argv[], char *envp[], HANDLE *piFile, HANDLE *piConsole);

#ifndef WIN32
HANDLE iGlobalFileHandle = -1; // global because FBSD will need to unlock it
#endif // WIN32

int main(int argc, char *argv[], char *envp[])
{
#ifdef WIN32
HANDLE iFile;
#endif // WIN32
HANDLE iConsole; // for non-WIN32 'HANDLE' is defined as 'int'
int iRval;

#ifdef WIN32
  iFile = iConsole = INVALID_HANDLE_VALUE;
#else // !WIN32
  iGlobalFileHandle = iConsole = -1;
#endif // WIN32

#ifdef WIN32

  iRval = do_main(argc, argv, envp, &iFile, &iConsole);

  if(iFile != INVALID_HANDLE_VALUE)
  {
    CloseHandle(iFile);
  }
  if(iConsole != INVALID_HANDLE_VALUE)
  {
    CloseHandle(iConsole);
  }

#else // !WIN32

  iRval = do_main(argc, argv, envp, &iGlobalFileHandle, &iConsole);

  // TODO:  restore default signal handlers?  disable signal handlers?

  if(iGlobalFileHandle >= 0)
  {
#ifdef __FreeBSD__
    flock(iGlobalFileHandle, LOCK_UN);
#endif // __FreeBSD__
    close(iGlobalFileHandle);
    iGlobalFileHandle = -1; // TODO:  race condition?
  }

  conrestore();

  if(iConsole >= 0)
  {
    close(iConsole);
  }
#endif // WIN32

  return iRval;
}


void usage()
{
  fprintf(stderr,
          "%s - Copyright (c) 2011-2013 S.F.T. Inc. - all rights reserved\n\n"
          "usage:\t%s [-h]|[-[e][r[m|n]][R][v[v...]][B baud][N|W wait]",
          pApp, pApp);
  fputs(
#ifndef WIN32
        "[c|l device|socket|[IP]:port]"
#endif // WIN32
        "] device\n"
        " where\t-r puts you in 'raw' terminal mode\n"
        " and\t-m sets the terminator to [CR] in 'raw' mode\n"
        " and\t-n sets the terminator to [LF] in 'raw' mode\n"
        " and\t-B assigns the 'baud rate' configuration string\n"
            "\t   example:  sftardcal -B 9600,n,8,1\n"
        " and\t-N disables the serial port auto-reset\n"
        " and\t-Q stifles output on stderr\n"
        " and\t-F enables hardware flow control (implies -N)\n"
        " and\t-W assigns the reset 'wait' period in seconds (default 5)\n"
#ifndef WIN32
        " and\t-d dumps [serial port] debug information\n"
#endif // WIN32
        " and\t-e enables local echo\n"
        " and\t-R forces a factory reset during calibration\n"
        " and\t-v increases verbosity of debug info sent to stderr\n"
            "\t   (can be specified multiple times to increase verbosity)\n"
#ifdef WITH_XMODEM
        " and\t-X[S|R][filename] performs an xmodem transfer\n"
        "\t   the 'S' or 'R' must immediately precede the file name (no space)\n"
        "\t   The command 'XSfilename' or 'XRfilename' (followed by \\r) is sent\n"
        "\t   to the remote device, followed by the file transfer itself.\n"
        "\t   This option may not be used with '-q', '-r', or '-R'\n"
#endif // WITH_XMODEM
#ifndef WIN32
        " and\t-c specifies an alternate console for stdin,stdout\n"
            "\t   (useful for providing a tunnel from a VM running this software)\n"
            "\t   specifying a port as ':port' assumes localhost:port\n"
        " and\t-l is a special case version of '-c' that allows you to LISTEN for\n"
            "\t   a TCP connection on a specific TCP port, optionally specifying\n"
            "\t   the 'bind' address (default is all available IPs).\n"
        " and\t-q specifies a 'question' to send.  response returned on stdout\n"
            "\t   implies '-N' to disable serial port auto-reset.\n"
            "\t   This option may not be used with '-r', '-R', or '-X'\n"
        " and\t-w specifies a wait time (for use with '-q'), default 5 seconds\n"
#endif // WIN32
        "\n"
        "-and-\t-h prints this message\n\n", stderr);

  fprintf(stderr, "Default serial port device is \"%s\"\n\n", pIn);
}

#ifndef WIN32
void signalproc(int iSig)
{
static const char szMsg[]="\nError - exit on signal (console settings restored)\n";

  if(iGlobalFileHandle != -1)
  {
#ifdef __FreeBSD__
    flock(iGlobalFileHandle, LOCK_UN);
#endif // __FreeBSD__
    close(iGlobalFileHandle); // a hack for now
    iGlobalFileHandle = -1;
  }

  conrestore();
  write(2, szMsg, sizeof(szMsg) - 1);

  _exit(iSig);  // bye [must call THIS version of 'exit()']
}
#endif // WIN32



int do_main(int argc, char *argv[], char *envp[], HANDLE *piFile, HANDLE *piConsole)
{
int i1;
const char *p1;

#ifndef WIN32
  // these signals all terminate the process, though I will
  // want to make sure I restore the console settings before
  // the program exits.  As such I use a signal proc, then call
  // '_exit()' to terminate the application.
  signal(SIGINT, signalproc);
  signal(SIGTSTP, signalproc);
  signal(SIGTERM, signalproc);
  signal(SIGUSR1, signalproc);
  signal(SIGUSR2, signalproc);
#endif // WIN32

  // stdout handle - for POSIX it's always '1', for Windows you need 'GetStdHandle()'
#ifdef WIN32
  iStdOut = GetStdHandle(STD_OUTPUT_HANDLE); // windows supplies an API for this
#else // !WIN32
  iStdOut = 1; // in POSIX OS's, stdout is always '1'
#endif // WIN32

  pApp = argv[0];
  p1 = pApp;
  pApp += strlen(pApp) - 1;
  while(pApp > p1 && *(pApp - 1) != '/')
  {
    pApp--;
  }


  i1 = do_options(argc, argv, envp);
  if(i1 || argc <= 0)
  {
    usage();
    return(i1);
  }

#ifdef WIN32
  if(!DuplicateHandle(GetCurrentProcess(), GetStdHandle(STD_INPUT_HANDLE),
                     GetCurrentProcess(), piConsole,
                     0, 0, DUPLICATE_SAME_ACCESS))
  if(*piConsole == INVALID_HANDLE_VALUE)
#else // WIN32
  // to allow testing windows in a 'virtualbox' VM I added the '-c' option, which
  // re-directs console I/O to a pipe, device, or socket.  device/pipe I/O is attempted
  // first, followed by socket I/O.  TODO:  use 'stat' to determine which to use

  if(pAltConsole) // alternate console, for testing via tunnel to VM's serial port
  {
    i1 = configure_alt_console(piConsole); // NOTE:  this assigns 'iConsole'
    if(i1)
    {
      return i1; // an error
    }
  }
  else
  {
    *piConsole = dup(0); // copy of STDIN's handle
  }


  if(*piConsole < 0)
#endif // WIN32
  {
    fprintf(stderr, "Unable to dup console, %d\n", errno);
    return -2;
  }

#ifndef WIN32
  if(pAltConsole) // if using alternate console, don't do 'conconfig'
  {
    altconconfig(*piConsole); // in case I must configure it like a console
  }
  else
#endif // WIN32
  {
    conconfig(*piConsole); // configure the console (for windows it starts a thread)
  }

#ifdef WIN32

  *piFile = CreateFile(pIn, GENERIC_READ | GENERIC_WRITE, 0,
                       NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
  if(*piFile == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "Unable to open %s\n", pIn);
    CloseHandle(*piConsole);
    *piConsole = INVALID_HANDLE_VALUE;
    return -1;
  }

#else // WIN32

#ifdef __FreeBSD__

  *piFile = open(pIn, (O_RDWR | O_NONBLOCK /*| O_EXLOCK*/), 0);

  if(*piFile >= 0)
  {
    if(flock(*piFile, LOCK_EX | LOCK_NB)  < 0)
    {
      fprintf(stderr, "ERROR:  unable to lock \"%s\", errno=%d\n",
              pIn, errno);
      close(*piFile);
      *piFile = -1;
    }
  }
#elif defined(__linux__) && defined(LINUX_SPECIAL_HANDLING)
  *piFile = open(pIn, (O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY), 0); // Linux does not support O_EXLOCK and needs 'O_NOCTTY' and 'O_NDELAY'
#else // !__FreeBSD__, !__linux__
  *piFile = open(pIn, (O_RDWR | O_NONBLOCK), 0); // Linux does not support O_EXLOCK
#endif // __FreeBSD__

  if(*piFile < 0)
  {
    fprintf(stderr, "Unable to open %s\n", pIn);
    close(*piConsole);
    *piConsole = -1;
    return -1;
  }

#endif // WIN32

  // properly parse out the baud rate as BAUD,parity,bits,stop  i.e. "9600,n,8,1"
  // and assign it

  ttyconfigSTR(*piFile, szBaud); // POOBAH

  if(iExperimental)
  {
    // perform experiments here

    goto exit_point;
  }

  if(iResetWait >= 0) // say something about it at this point (before I assign a default value)
  {
    fputs("Reset Arduino via Serial Port\n", stdout);
    fflush(stdout);
  }

  if(!iResetWait) // i.e. 'use default'
  {
    iResetWait = DEFAULT_RESET_WAIT;
  }

  if(iResetWait > 0) // arduino reset using the RTS+DTR line
  {
    reset_arduino(*piFile);

    fputs("Waiting for device reset to complete...", stdout);
    fflush(stdout);

    // wait several seconds for initialization
    while(iResetWait > 1)
    {
      MySleep(1000);
      fputs(".", stdout);
      fflush(stdout);
      iResetWait--;
    }

    MySleep(1000);
  }
  else if(iFlowControl > 0) // only if there's no 'reset wait' - they ARE mutually exclusive!
  {
    set_rts_dtr(*piFile, 1); // this simply sets the RTS+DTR line to a 'LOW' state [necessary for flow control]
  }

  if(!pszQuestion)
  {
    fputs("!\n", stdout);
    fflush(stdout);
  }

  if(pszQuestion)
  {
    question_loop(*piFile, *piConsole);
  }
  else if(bRawFlag)
  {
    console_loop(*piFile, *piConsole);
  }
#ifdef WITH_XMODEM
  else if(bXModemFlag)
  {
    do_xmodem(*piFile, *piConsole);
  }
#endif // WITH_XMODEM
  else
  {
    // what I normally do
    calibrate_loop(*piFile, *piConsole);
  }

  if(iResetWait >= 0 || iFlowControl > 0)
  {
    set_rts_dtr(*piFile, 0); // NOTE:  avrdude does this, setting RTS and DTR _LOW_ at this point - should I?
    // NOTE:  this forces a reset next time someone opens the device, so...
  }

exit_point:
#ifdef WIN32

  CloseHandle(*piFile);
  CloseHandle(*piConsole);
  *piFile = *piConsole = INVALID_HANDLE_VALUE;

#else // WIN32

  // TODO:  restore default signal handlers?  disable signal handlers?

  signal(SIGINT, SIG_DFL);  // SIG_IGN);
  signal(SIGTSTP, SIG_DFL); // SIG_IGN);
  signal(SIGTERM, SIG_DFL); // SIG_IGN);
  signal(SIGUSR1, SIG_DFL); // SIG_IGN);
  signal(SIGUSR2, SIG_DFL); // SIG_IGN);

#ifdef __FreeBSD__
  flock(*piFile, LOCK_UN);
#endif // __FreeBSD__

  close(*piFile); // I'm closing here [possible race condition?]
  conrestore();
  close(*piConsole);
  *piFile = *piConsole = -1;

#endif // WIN32

  return 0;
}

int do_options(int argc, char *argv[], char * envp[])
{
#ifndef HAVE_GETOPT /* basically, WIN32 */
int i1, optind;

  for(optind=1; optind < argc; optind++)
  {
    if(argv[optind][0] != '-')
      break;

    if(argv[optind][1] == '-')
    {
      optind++;
      break;
    }

    for(i1=1; argv[optind][i1]; i1++)
    {
      if(argv[optind][i1] == 'd')
      {
        bSerialDebug = 1;
      }
      else if(argv[optind][i1] == 'B')
      {
        if(argv[optind][i1 + 1])
        {
#ifdef WIN32
          strncpy_s(szBaud, sizeof(szBaud) - 1,
                    &(argv[optind][i1 + 1]), _TRUNCATE);
#else // WIN32
          strncpy(szXBaud, &(argv[optind][i1 + 1]), sizeof(szBaud) - 1);
#endif // WIN32
        }
        else
        {
          optind++;
#ifdef WIN32
          strncpy_s(szBaud, sizeof(szBaud) - 1,
                    argv[optind], _TRUNCATE);
#else // WIN32
          strncpy(szBaud, argv[optind], sizeof(szBaud) - 1);
#endif // WIN32
        }

        szBaud[sizeof(szBaud) - 1] = 0;
        break;
      }
      else if(argv[optind][i1] == 'N')
      {
        iResetWait = -1; // no reset wait
      }
      else if(argv[optind][i1] == 'F')
      {
        iFlowControl = 1;
        iResetWait = -1; // implies no reset wait as well
      }
      else if(argv[optind][i1] == 'W')
      {
        // TODO:  wait period, mutually exclusive with 'N'
      }
      else if(argv[optind][i1] == 'e')
      {
        bLocalEcho = 1;
      }
      else if(argv[optind][i1] == 'r')
      {
        if(
#ifdef WITH_XMODEM
           bXModemFlag ||
#endif // WITH_XMODEM
           bFactoryReset ||
           pszQuestion != NULL)
        {
          usage();
          return 1;
        }

        bRawFlag = 1;
      }
      else if(argv[optind][i1] == 'm')
      {
        if(
#ifdef WITH_XMODEM
           bXModemFlag ||
#endif // WITH_XMODEM
           bFactoryReset)
        {
          usage();
          return 1;
        }

        bRawFlag = 1;
      }
      else if(argv[optind][i1] == 'c')
      {
        iTerminator = '\r';
      }
      else if(argv[optind][i1] == 'n')
      {
        iTerminator = '\n';
      }
      else if(argv[optind][i1] == 'R')
      {
        if(
#ifdef WITH_XMODEM
           bXModemFlag ||
#endif // WITH_XMODEM
           bRawFlag ||
           pszQuestion != NULL)
        {
          usage();
          return 1;
        }

        bFactoryReset = 1;
      }
      else if(argv[optind][i1] == 'h' ||
              argv[optind][i1] == 'H' ||
              argv[optind][i1] == '?')
      {
        usage();
        return 1;
      }
      else if(argv[optind][i1] == 'v')
      {
        iVerbosity++;
      }
#ifdef WITH_XMODEM
      else if(argv[optind][i1] == 'X')
      {
        // next char must be S or R followed by the file name
        // file name can ALSO be the next parameter

        if(bRawFlag || bFactoryReset || pszQuestion != NULL ||
           (argv[optind][i1 + 1] &&
            argv[optind][i1 + 1] != 'R' &&
            argv[optind][i1 + 1] != 'S') ||
           (!argv[optind][i1 + 1] &&
            ((optind + 1) >= argc ||
             (argv[optind+1][0] != 'R' &&
              argv[optind+1][0] != 'S'))))
        {
          usage();
          return 1;
        }

        if(argv[optind][i1 + 1])
        {
#ifdef WIN32
          strncpy_s(szXModemFile, sizeof(szXModemFile) - 1,
                    &(argv[optind][i1 + 1]), _TRUNCATE);
#else // WIN32
          strncpy(szXModemFile, &(argv[optind][i1 + 1]), sizeof(szXModemFile) - 1);
#endif // WIN32
        }
        else
        {
          optind++;
#ifdef WIN32
          strncpy_s(szXModemFile, sizeof(szXModemFile) - 1,
                    argv[optind], _TRUNCATE);
#else // WIN32
          strncpy(szXModemFile, argv[optind], sizeof(szXModemFile) - 1);
#endif // WIN32
        }

        szXModemFile[sizeof(szXModemFile) - 1] = 0;
        bXModemFlag = 1;
        break;
      }
#endif // WITH_XMODEM
      else
      {
        fprintf(stderr, "Illegal or unrecognized option\n");
        usage();
        return 1;
      }
    }
  }
#else  // HAVE_GETOPT
int i1;

  while((i1 = getopt(argc, argv,
                     "xhrmndeFRvNQW:l:B:c:q:w:"
#ifdef WITH_XMODEM
                     "X:"
#endif // WITH_XMODEM
                     )) != -1)
  {
    switch(i1)
    {
      case 'x':
        // experimental option
        iExperimental = 1;
        break;
      case 'd': // serial port debug
        bSerialDebug = 1;
        break;
      case 'e': // local echo
        bLocalEcho = 1;
        break;
      case 'r': // raw console access
        if(
#ifdef WITH_XMODEM
           bXModemFlag ||
#endif // WITH_XMODEM
           bFactoryReset ||
           pszQuestion != NULL)
        {
          usage();
          return 1;
        }

        bRawFlag = 1;
        break;

      case 'm':
        iTerminator = '\r';
        break;

      case 'n':
        iTerminator = '\n';
        break;

      case 'R': // Factory Reset
        if(
#ifdef WITH_XMODEM
           bXModemFlag ||
#endif // WITH_XMODEM
           pszQuestion != NULL ||
           bRawFlag)
        {
          usage();
          return 1;
        }

        bFactoryReset = 1;
        break;
      case 'v': // verbosity
        iVerbosity++;
        break;
      case 'F': // flow control
        iFlowControl = 1;
        if(iResetWait > 0)
        {
          usage();
          exit(1);
        }
        iResetWait = -1; // flow control implies reset wait also
        break;
      case 'N': // no 'reset wait'
        if(iResetWait > 0)
        {
          usage();
          exit(1);
        }
        iResetWait = -1;
        break;
      case 'W': // wait time
        if(iResetWait < 0) // used '-N' option?
        {
          usage();
          exit(1);
        }

        iResetWait = atoi(optarg);

        if(iResetWait <= 0)
        {
          usage();
          exit(1);
        }
        break;
      case 'l': // 'listen' mode (TCP only, implies -c)
        bListenMode = 1;
      case 'c': // specify a console
//        if(!optarg || *optarg == ':')
//        {
//          fputs("console not specified\n", stderr);
//          usage();
//          exit(1);
//        }
        pAltConsole = malloc(strlen(optarg) + 1);
        strcpy(pAltConsole, optarg);
        break;
      case 'B': // specify a console
//        if(!optarg || *optarg == ':')
//        {
//          fputs("BAUD string not specified\n", stderr);
//          usage();
//          exit(1);
//        }
        strncpy(szBaud, optarg, sizeof(szBaud));
        break;

#ifdef WITH_XMODEM
      case 'X': // xmodem transfer
        // next char must be S or R followed by the file name
        // file name can ALSO be the next parameter

        if(bRawFlag || bFactoryReset || pszQuestion != NULL ||
           !optarg || !*optarg ||
           (optarg[0] != 'R' && optarg[0] != 'S'))
        {
          usage();
          return 1;
        }

        strncpy(szXModemFile, optarg, sizeof(szXModemFile) - 1);
        szXModemFile[sizeof(szXModemFile) - 1] = 0;
        bXModemFlag = 1;

        break;
#endif // WITH_XMODEM

      case 'Q': // quiet mode
        bQuietFlag = 1;
        break;

      case 'q':
        if(bRawFlag || bFactoryReset
#ifdef WITH_XMODEM
           || bXModemFlag
#endif // WITH_XMODEM
           || !optarg || !*optarg)
        {
          usage();
          return 1;
        }

        // will send 'optarg' using specified line endings

        pszQuestion = malloc(strlen(optarg) + 2);
        if(!pszQuestion)
        {
          fprintf(stderr, "Unable to allocate memory for arg!\n");
          return 1;
        }

        memcpy(pszQuestion, optarg, strlen(optarg) + 1);

        iResetWait = -1; // no reset wait implied

        break;

      case 'w': // wait time

        iQuestionWait = atoi(optarg);

        if(iQuestionWait <= 0)
        {
          usage();
          exit(1);
        }
        break;

      default:
        fprintf(stderr, "Illegal or unrecognized option\n");
      case 'h':
      case '?':
        return 1;
    }
  }
#endif // HAVE_GETOPT

  argc -= optind;
  argv += optind;

  if(argc > 0) // only one extra option allowed at this time
  {
    pIn = argv[0];
    argc--;
    argv++;
  }

  if(argc < 0)
  {
    return -1;
  }

  return 0;
}


#define TTYCONFIG_PARSE(X,Y) if(!(Y = strchr(X, ','))) { Y = (X) + strlen(X) + 1; } else { *((Y)++) = 0; }

static void ttyconfigSTR(HANDLE iFile, char *szBaud)
{
  int iBaud=9600, iParity=0, iBits=8, iStop=1;
  char *p1, *p2;

  p1 = szBaud;

  if(*p1)
  {
    TTYCONFIG_PARSE(p1,p2);
    if(*p1)
    {
      iBaud = atoi(p1);
      if(iBaud <= 0)
      {
        iBaud = 9600;
      }
    }
    p1 = p2;
  }

  if(*p1)
  {
    TTYCONFIG_PARSE(p1,p2);
    if(*p1)
    {
      if(*p1 == 'n' || *p1 == 'N')
      {
        iParity = 0;
      }
      else if(*p1 == 'e' || *p1 == 'E')
      {
        iParity = -1;
      }
      else if(*p1 == 'o' || *p1 == 'O')
      {
        iParity = 1;
      }
      else
      {
        iParity = 0;
      }
    }
    p1 = p2;
  }
  if(*p1)
  {
    TTYCONFIG_PARSE(p1,p2);
    if(*p1)
    {
      iBits = atoi(p1);
      if(iBits < 6 || iBits > 8)
      {
        iBits = 8;
      }
    }
    p1 = p2;
  }
  if(*p1)
  {
    TTYCONFIG_PARSE(p1,p2);
    if(*p1)
    {
      iStop = atoi(p1);
      if(iStop < 1 || iStop > 2)
      {
        iStop = 1; // do NOT handle 1.5
      }
    }
    p1 = p2;
  }

  if(!bQuietFlag)
  {
    fprintf(stderr, "Setting serial config to %d,%c,%d,%d\n",
            iBaud, (char)(iParity ? (iParity > 0 ? 'o' : 'e') : 'n'), iBits, iStop);
  }

  ttyconfig(iFile, iBaud, iParity, iBits, iStop);
}

#ifndef WIN32
int configure_alt_console(HANDLE *piConsole)
{
  struct sockaddr_storage sa; // sufficiently large enough to handle IPv4 or IPv6
  int i2, iTemp = -1;
  int bIsSocket = 0;
  struct sockaddr_in *pSA4 = NULL;
  struct sockaddr_in6 *pSA6 = NULL;
  char *p1;
  const char *pName = pAltConsole;

  // the purpose of THIS code is to let me use the '-c' or '-l' option to open a pipe,
  // device, or socket and use it in lieu of the actual console input

  if(NULL != (p1 = strrchr(pAltConsole, ':'))) // look for the final ':' and it better be TCP or else
  {
    memset(&sa, 0, sizeof(sa));

//#ifdef __FreeBSD__ /* Linux doesn't have this data member */
//      ((struct sockaddr_in6 *)&sa)->sin6_len = sizeof(sa); // uint8_t, same for sockaddr_in::sin_len
//#endif // __FreeBSD__

    *(p1++) = 0; // so that 'p1' points to the port

    if(!*pAltConsole) // no IP address
    {
      pSA4 = (struct sockaddr_in *)&sa;
      pSA4->sin_family = AF_INET;
      pName = "*"; // so I will report the name as '*'

#ifdef __FreeBSD__ /* Linux doesn't have this data member */
      pSA4->sin_len = sizeof(*pSA4);
#endif // __FreeBSD__

      if(!bListenMode)
      {
        // this was unsigned long but still worked because dest is sockaddr_storage, not sockaddr_in
        // and so if I used 'memcpy' with sizeof(unsigned long) it didn't overrun any buffers but
        // only worked properly on 64-bit if the system was low endian.  this fixes it.
        uint32_t ulTemp = htonl(INADDR_LOOPBACK); // localhost, 127.0.0.1

        // this assigns 'sin_addr' assuming it's a uint32_t using memcpy to avoid certain irritating warnings
        // while maintaining some compatibility across versions, OSs, etc.
        memcpy(&(pSA4->sin_addr), &ulTemp, sizeof(uint32_t)); // assign using memcpy for warning avoidance; should optimize ok
      }
    }
    else if(*pAltConsole == '[') // required for ipv6 as "[ip:ad:dre:ss]:port"
    {
      pName++; // point past the '[' (mostly for error messages)

      pSA6 = (struct sockaddr_in6 *)&sa;
      pSA6->sin6_family = AF_INET6;

#ifdef __FreeBSD__ /* Linux doesn't have this data member */
      pSA6->sin6_len = sizeof(*pSA6);
#endif // __FreeBSD__

      // zero-byte the ']' at the end
      if(*(p1 - 2) == ']') // should be
      {
        *(p1 - 2) = 0;
      }

      if(0 >= inet_pton(AF_INET6, pAltConsole, &(pSA6->sin6_addr)))
      {
        fprintf(stderr, "Invalid alternate (ipv6?) console '%s'\n (must be '[IPv6]:port' or 'IP:port' or ':port')\n", pName);
        usage();
        return 1;
      }
    }
    else
    {
      pSA4 = (struct sockaddr_in *)&sa;
      pSA4->sin_family = AF_INET;

#ifdef __FreeBSD__ /* Linux doesn't have this data member */
      pSA4->sin_len = sizeof(*pSA4);
#endif // __FreeBSD__

      // this used to have &sa on it, but 'teh intarwebs' says it should be sin_addr (not sure why it might have worked before)
      if(0 >= inet_pton(AF_INET, pAltConsole, &(pSA4->sin_addr))) //(struct sockaddr *)&sa))
      {
        fprintf(stderr, "Invalid alternate console '%s'\n (must be '[IPv6]:port' or 'IP:port' or ':port')\n", pName);
        usage();
        return 1;
      }
    }

    // assign the port
    if(pSA4)
    {
      pSA4->sin_port = htons(atoi(p1));
    }
    else // assume pSA6 assigned
    {
      pSA6->sin6_port = htons(atoi(p1));
    }

    bIsTCP = 1; // is definitely TCP
  }
  else
  {
    struct stat st;

    if(bListenMode)
    {
      fprintf(stderr, "Invalid alternate console '%s' for 'listen' option\n   (must be '[IPv6]:port' or 'IP:port' or ':port')\n", pName);
      usage();
      return 1;
    }

    memset(&st, 0, sizeof(st));

    if(0 > stat(pAltConsole, &st))
    {
      fprintf(stderr, "Cannot 'stat' %s (errno=%d)\n", pName, errno);
      usage();
      return 1;
    }
    else if(0 != (st.st_mode & S_IFSOCK))
    {
      bIsSocket = 1; // it's definitely a socket
    }
    else
    {
      iTemp = open(pAltConsole, O_RDWR, 0); // NOTE:  does not work on sockets...
    }
  }

  if(bIsSocket || // is definitely a socket
      bIsTCP ||    // is definitely TCP
      (iTemp == -1 && errno == EOPNOTSUPP)) // might be a socket (TODO:  use stat() )
  {
    if(bIsTCP)
    {
      iTemp = socket((pSA6 ? PF_INET6 : PF_INET), SOCK_STREAM, 0); // TCP socket
    }
    else
    {
      iTemp = socket(PF_LOCAL, SOCK_STREAM, 0); // UNIX-style socket
    }

    if(iTemp != -1)
    {
      if(pSA4 || pSA6)
      {
//          char new_tbuf[256];
//
//          inet_ntop(pSA6 ? AF_INET6 : AF_INET,
//                    (pSA6 ? (const struct sockaddr *)pSA6 : (const struct sockaddr *)pSA4),
//                    new_tbuf, sizeof(new_tbuf));

        if(bListenMode)
        {
          i2 = 1;  // set non-blocking mode for the 'listen' socket (to prevent issues)
          if(ioctl(iTemp, FIONBIO, &i2) < 0)
          {
            fprintf(stderr, "Warning:  'ioctl(FIONBIO)' failed, errno = %d\n", errno);
          }

          if(pSA6)
          {
            i2 = bind(iTemp, (const struct sockaddr *)pSA6, sizeof(*pSA6));
          }
          else
          {
            i2 = bind(iTemp, (const struct sockaddr *)pSA4, sizeof(*pSA4));
          }
          if(i2 < 0)
          {
            close(iTemp);

//              inet_ntop(pSA6 ? AF_INET6 : AF_INET,
//                        (pSA6 ? (const struct sockaddr *)pSA6 : (const struct sockaddr *)pSA4),
//                        new_tbuf, sizeof(new_tbuf));
//
//              fprintf(stderr, "Cannot 'bind' to '%s' (%d, errno=%d) %p %p\n   %s\n", pName, i2, errno, pSA4, pSA6, new_tbuf);

            return -9;
          }

          while(1)
          {
            if(listen(iTemp, 5) < 0)
            {
              close(iTemp);
              fprintf(stderr, "Cannot 'listen' on '%s' (errno=%d)\n", pName, errno);
              usage();
              return -9;
            }

            // wait for a connect and FORK if I get one, wait for process to end, and loop back
            // if I get a signal on THIS process, I'll terminate

            if(!bQuietFlag && !bSerialDebug)
            {
              fputs("listening for TCP connect", stderr);

              if(!bSerialDebug)
              {
                fputs("\n", stderr);
              }
              else
              {
                fputs("...", stderr);
              }

              fflush(stderr);
            }

            while(1)
            {
              int i2, i3, sAccept; // the socket i'm accepting
              fd_set fds1;
              struct timeval tmvWait;

              FD_ZERO(&fds1);
              FD_SET(iTemp, &fds1); // waiting on this one (for accept)

              tmvWait.tv_sec = 0;
              tmvWait.tv_usec = 500000;  // up to 500 millisecs

              i2 = select(iTemp, &fds1, NULL, NULL, &tmvWait);
              sAccept = -1;

              if(i2 >= 0)// && FD_ISSET(iTemp, &fds1))
              {
                struct sockaddr_storage saAccept;
                socklen_t cbAccept;

                cbAccept = sizeof(saAccept);

                sAccept = accept(iTemp, (struct sockaddr *)&saAccept, &cbAccept);
                if(sAccept < 0)
                {
                  if(errno != EWOULDBLOCK)
                  {
                    if(bSerialDebug)
                    {
                      fputs("\n", stderr);
                      fflush(stderr);
                    }

                    fputs("WARNING - invalid socket returned by 'accept'\n", stderr);
                    fflush(stderr);

                    continue; // so I don't print anything else this loop
                  }
                }
                else
                {
                  // fork!
                  i2 = fork();

                  if(i2 == -1)     // error
                  {
                    fprintf(stderr, "Connect accepted, unable to 'fork' (errno=%d)\n", errno);

                    close(iTemp);

                    return -10;
                  }
                  else if(!i2) // this is the FORKED process
                  {
                    close(iTemp); // I don't need a copy of the original in the forked version

                    bSerialDebug = 0; // I don't want serial debug in the forked process, kthx

                    iStdOut = *piConsole = iTemp = sAccept; // assign new socket
//                      goto the_console_fork_spot; // used a label as there are too many 'if' blocks otherwise
                    return 0; // this tells caller to "just continue"
                  }
                  else // process ID is in 'i2' - wait for it
                  {
                    close(sAccept); // do this or the client will 'hang' if the forked process ends prematurely

                    if(bSerialDebug)
                    {
                      fputs("!\n", stderr);
                      fflush(stderr);
                    }

                    if(!bQuietFlag && !bSerialDebug)
                    {
                      fprintf(stderr, "forked process id %d, waiting", i2);

                      if(bSerialDebug)
                      {
                        fputs("...\n", stderr);
                      }
                      else
                      {
                        fputs("\n", stderr);
                      }

                      fflush(stderr);
                    }

                    i3 = 0;

                    while(!waitpid(i2, &i3, WNOHANG))
                    {
                      i3++;
                      usleep(25000); // wait 25 msecs

                      if(i3 >= 20) // more than 1/2 sec
                      {
                        i3 = 0;

                        if(bSerialDebug)
                        {
                          fputs(".", stderr);
                          fflush(stderr);
                        }
                      }
                    }

                    if(bSerialDebug)
                    {
                      fputs("!", stderr);
                    }

                    if(!bQuietFlag && !bSerialDebug)
                    {
                      fputs("\nProcess terminated, listening again", stderr);

                      if(bSerialDebug)
                      {
                        fputs("...", stderr);
                      }
                      else
                      {
                        fputs("\n", stderr);
                      }

                      fflush(stderr);
                    }

                    continue;  // so I don't print anything else this loop (see last section)
                  }
                }
              }

              if(bSerialDebug)
              {
                fputs(".", stderr);
                fflush(stderr);
              }
            }

          }
        }
      }
      else
      {
        // for this I need to malloc a structure for 'sockaddr' that includes the entire
        // string used for 'pAltConsole', rather than rely on 'sa' (above) being big enough.

        struct sockaddr * pSA = (struct sockaddr *)malloc(sizeof(struct sockaddr) + strlen(pAltConsole));

        if(!pSA)
        {
          close(iTemp);
          iTemp = -1;
        }
        else
        {
          int iLen = sizeof(struct sockaddr) - sizeof(pSA->sa_data)
                    + strlen(pAltConsole) + 1;
#ifdef __FreeBSD__
          // FreeBSD uses an 'sa_len' member.  Other BSDs may be similar (need to check)
          // TODO:  add a configure script step to determine presence of 'sa_len', similar to qsort_r test
          pSA->sa_len = iLen;
#endif // __FreeBSD__
          pSA->sa_family = AF_LOCAL;
          strcpy(pSA->sa_data, pAltConsole);

          // NOTE:  the socket SHOULD be creatable, either with 'bind()' or as a FIFO using 'mkfifo()' so MAYBE
          //        an option to create it if it's not already there?

          if(-1 == connect(iTemp, pSA, iLen)) // connect to PF_LOCAL (i.e. UNIX socket)
          {
            // TODO:  OPTION:  use 'stat()' to see if the file exists; if it doesn't, then optionally
            //        create the socket/FIFO with 'bind()' instead of 'connect()', or else use
            //        'mkfifo()' to create a FIFO and then re-try the connect.
            //
            //        (doing 'bind' creates a socket; doing 'mkfifo()' creates a FIFO; either should work,
            //         though it's possible a FIFO can only be used 'one way' and you'd need a pair of them)

            close(iTemp);
            iTemp = -1;
          }

          free((void *)pSA);
        }
      }
    }
  }

  if(iTemp == -1)
  {
    fprintf(stderr, "Unable to open alternate console %s (a) errno=%d\n", pAltConsole, errno);
    return -1;
  }

  iStdOut = *piConsole = iTemp; // iConsole == iStdOut when re-directed. It's simpler that way.

  return 0; // "just continue"
}

#endif // !WIN32


// *************** //
// DEBUG FUNCTIONS //
// *************** //

void sftardcal_debug_dump_buffer(int iDir, const void *pBuf, int cbBuf)
{
int i1, i2;
const unsigned char *p1, *p2;

  if(cbBuf <= 0)
  {
    return;
  }

  if(Verbosity() >= VERBOSITY_GEEKY)
  {
    fprintf(stderr, "[%u]\n", MyGetTickCount());
  }

  p1 = p2 = (const unsigned char *)pBuf;

  for(i1=0, i2=0; i1 <= cbBuf; i1++, p1++)
  {
    if(!i1 || i2 >= 16 || i1 == cbBuf)
    {
      if(i1)
      {
        while(i2 < 16)
        {
          fputs("    ", stderr); // fill up spaces where data would be
          i2++;
        }

        fputs(" : ", stderr);

        while(p2 < p1)
        {
          if(*p2 >= 32 && *p2 <= 127)
          {
            fputc(*p2, stderr);
          }
          else
          {
            fputc('.', stderr);
          }

          p2++;
        }

        fputc('\n', stderr);
      }

      if(!i1 && iDir > 0)
      {
        fputs("--> ", stderr);
      }
      else if(!i1 && iDir < 0)
      {
        fputs("<-- ", stderr);
      }
      else
      {
        fputs("    ", stderr);
      }

      i2 = 0;
      p2 = p1; // make sure
    }

    if(i1 < cbBuf)
    {
      if(!i2)
      {
        fprintf(stderr, "%02x: %02x", i1, *p1);
      }
      else
      {
        fprintf(stderr, ", %02x", *p1);
      }

      i2++;
    }
  }

  fputc('\n', stderr);
  fflush(stderr);
}

static char clddBufI[32], clddBufO[32];
static unsigned int clddCountI = 0, clddCountO = 0, clddTickI = 0, clddTickO = 0;
int clddDir = 0;
void console_loop_debug_dump(int iDir, const void *pBuf0, int cbBuf)
{
const unsigned char *pBuf = (const unsigned char *)pBuf0;

// dump I/O every 100 msecs or if buffer fills up
  if((clddDir && iDir && clddDir != iDir) ||
     (clddCountI > 0 /*&& clddDir < 0*/ && (MyGetTickCount() - clddTickI) >= 100) ||
     clddCountI >= sizeof(clddBufI) ||
     (clddCountO > 0 /*&& clddDir > 0*/ && (MyGetTickCount() - clddTickO) >= 100) ||
     clddCountO >= sizeof(clddBufO))
  {
    if(clddDir < 0 && clddCountI > 0) // do the "in" buffer first
    {
      sftardcal_debug_dump_buffer(-1, clddBufI, clddCountI);
      clddCountI = 0;
      clddTickI = MyGetTickCount();
    }
    else if(clddDir > 0 && clddCountO > 0) // do the "out" buffer first
    {
      sftardcal_debug_dump_buffer(1, clddBufO, clddCountO);
      clddCountO = 0;
      clddTickO = MyGetTickCount();
    }

    if(clddCountI > 0) // do the "in" buffer
    {
      sftardcal_debug_dump_buffer(-1, clddBufI, clddCountI);
      clddCountI = 0;
      clddTickI = MyGetTickCount();
    }

    if(clddCountO > 0) // do the "out" buffer
    {
      sftardcal_debug_dump_buffer(1, clddBufO, clddCountO);
      clddCountO = 0;
      clddTickO = MyGetTickCount();
    }
  }

  if(!iDir) // just a periodic check
  {
    return;
  }

  clddDir = iDir;

  if(!pBuf0 || !cbBuf)
  {
    return;
  }

  if(iDir < 0)
  {
    while(clddCountI + cbBuf >= sizeof(clddBufI))
    {
      memcpy(clddBufI + clddCountI, pBuf, sizeof(clddBufI) - clddCountI);

      pBuf += sizeof(clddBufI) - clddCountI;
      cbBuf -= sizeof(clddBufI) - clddCountI;
      clddCountI = 0;

      sftardcal_debug_dump_buffer(-1, clddBufI, sizeof(clddBufI));
    }

    if(cbBuf > 0)
    {
      memcpy(clddBufI + clddCountI, pBuf, cbBuf);
      clddCountI += cbBuf;
    }

    clddTickI = MyGetTickCount();
  }
  else
  {
    while(clddCountO + cbBuf >= sizeof(clddBufO))
    {
      memcpy(clddBufO + clddCountO, pBuf, sizeof(clddBufO) - clddCountO);

      pBuf += sizeof(clddBufO) - clddCountI;
      cbBuf -= sizeof(clddBufO) - clddCountO;
      clddCountO = 0;

      sftardcal_debug_dump_buffer(1, clddBufO, sizeof(clddBufO));
    }

    if(cbBuf > 0)
    {
      memcpy(clddBufO + clddCountO, pBuf, cbBuf);
      clddCountO += cbBuf;
    }

    clddTickO = MyGetTickCount();
  }
}

// direct console access

void console_loop(HANDLE iFile, HANDLE iConsole)
{
#ifndef WIN32
struct pollfd aFD[2];
#endif // WIN32
int i1;
int iWasCR = 0;

  do
  {
#ifndef WIN32
    aFD[0].fd = iFile;
    aFD[0].events = POLLIN | POLLERR;
    aFD[0].revents = 0;
    aFD[1].fd = iConsole; // stdin
    aFD[1].events = POLLIN;
    aFD[1].revents = 0;

    i1 = poll(aFD, 2, 100);
    if(!i1)
    {
      goto end_of_loop;
//      continue;
    }

    if(i1 < 0 || (aFD[0].revents & POLLERR) || (aFD[1].revents & POLLERR))
    {
      fprintf(stderr, "poll error %d\n", errno);
      return;
    }
#endif // WIN32

#ifdef WIN32
    if(my_pollin(iConsole) > 0)
#else // WIN32
    if(aFD[1].revents & POLLIN)
#endif // WIN32
    {
      char c1;
      i1 = my_read(iConsole, &c1, 1);

      if(i1 > 0)
      {
#ifndef WIN32
        if(pAltConsole) // no translation or 'local echo' if 'alt console'
        {
          if(Verbosity() >= VERBOSITY_CHATTY)
          {
            console_loop_debug_dump(1, &c1, 1);
          }

          my_write(iFile, &c1, 1);
        }
        else
#endif // WIN32
            if(c1 == 13 || c1 == 10) // either on input, translates into 'iTerminator' or CRLF
        {
          if(bLocalEcho)
          {
            fputs("\r\n", stdout);
            fflush(stdout);
          }
          if(!iTerminator)
          {
            my_write(iFile, "\r\n", 2);

            if(Verbosity() >= VERBOSITY_CHATTY)
            {
              console_loop_debug_dump(1, "\r\n", 2);
            }
          }
          else
          {
            c1 = iTerminator;
            my_write(iFile, &c1, 1);

            if(Verbosity() >= VERBOSITY_CHATTY)
            {
              console_loop_debug_dump(1, &c1, 1);
            }
          }
        }
        else if(c1 == 4 || c1 == 26) // ctrl+d or ctrl+z
        {
          SetQuitFlag();
        }
        else
        {
          if(bLocalEcho)
          {
            fwrite(&c1, 1, 1, stdout);
            fflush(stdout);
          }

          my_write(iFile, &c1, 1);

          if(Verbosity() >= VERBOSITY_CHATTY)
          {
            console_loop_debug_dump(1, &c1, 1);
          }
        }
      }
      else if(i1 <= 0 && bIsTCP)
      {
        SetQuitFlag(); // read error when the poll event said there WAS something indicates CLOSED SOCKET
      }
    }

#ifdef WIN32
    if(my_pollin(iFile) > 0)
#else // WIN32
    if(aFD[0].revents & POLLIN)
#endif // WIN32
    {
      char c1;
      i1 = my_read(iFile, &c1, 1);

      if(i1 > 0)
      {
        if(pAltConsole) // unmodified
        {
          if(Verbosity() >= VERBOSITY_CHATTY)
          {
            console_loop_debug_dump(-1, &c1, 1);
          }

          my_write(iStdOut, &c1, 1); // output to stdout, always
        }
        else if(c1 == '\r')
        {
          if(iTerminator == '\r')
          {
            my_write(iStdOut, "[CR]\n", 5);
          }
          else
          {
            iWasCR = 1;
            my_write(iStdOut, "[CR]", 4);
          }

          if(Verbosity() >= VERBOSITY_CHATTY)
          {
            console_loop_debug_dump(-1, "\r", 1);
          }
        }
        else
        {
          if(iWasCR)
          {
            if(c1 != '\n')
            {
              my_write(iStdOut, "\r", 1);

              if(Verbosity() >= VERBOSITY_CHATTY)
              {
                console_loop_debug_dump(-1, "\r", 1);
              }
            }

            iWasCR = 0;
          }

          if(c1 == '\n') // regardless of 'iTerminator' settings
          {
            my_write(iStdOut, "[LF]", 4);
          }

          my_write(iStdOut, &c1, 1); // output to stdout, always

          if(Verbosity() >= VERBOSITY_CHATTY)
          {
            console_loop_debug_dump(-1, &c1, 1);
          }
        }
      }
    }

#ifndef WIN32
end_of_loop: // necessary since 'continue' won't perform this properly for some reason
#endif // WIN32

    if(/*pAltConsole &&*/ Verbosity() >= VERBOSITY_CHATTY)
    {
      console_loop_debug_dump(0, NULL, 0); // allows timeouts to work to display data after 'no activity'
    }

  } while(!QuitFlag());
}


void question_loop(HANDLE iFile, HANDLE iConsole)
{
char *p1;

  bMyGetsEchoFlag = 0;
  p1 = send_command_get_multiline_reply_with_timeout(iFile, pszQuestion, iQuestionWait, 0);

  if(p1)
  {
#ifndef WIN32
    write(iStdOut, p1, strlen(p1));
#endif // WIN32

    free(p1);
  }

  return;
}


#ifdef WITH_XMODEM
// XMODEM transfers - some microcontroller devices may use
// this to transfer files reliably.  The xmodem library is
// cross-platform so you can use the same code HERE and THERE
void do_xmodem(HANDLE iFile, HANDLE iConsole)
{
int i1, i2;

  // try 3 times to accomplish this.

  bMyGetsEchoFlag = 0;

  for(i1=0; i1 < 3; i1++)
  {
    my_write(iFile, "X", 1);
    my_write(iFile, szXModemFile, strlen(szXModemFile));
    my_write(iFile, "\r", 1);

    if(szXModemFile[0] == 'S')
    {
      if(!bQuietFlag)
      {
        fprintf(stderr, "Sending file %s\n", &(szXModemFile[1]));
        fflush(stdout);
      }

      if(!(i2 = XSend(iFile, &(szXModemFile[1]))))
      {
        if(!bQuietFlag)
        {
          fputs("\nComplete!\n", stderr);
          fflush(stdout);
        }
        break;
      }

      if(!bQuietFlag)
      {
        fprintf(stderr, "\nXSend returns %d\n", i2);
        fflush(stdout);
      }
    }
    else // assume receive
    {
      if(!bQuietFlag)
      {
        fprintf(stderr, "Getting file %s\n", &(szXModemFile[1]));
        fflush(stdout);
      }

      if(!(i2 = XReceive(iFile, &(szXModemFile[1]), 0664)))
      {
        if(!bQuietFlag)
        {
          fputs("\nComplete!\n", stderr);
          fflush(stdout);
        }
        break;
      }

      if(!bQuietFlag)
      {
        fprintf(stderr, "\nXReceive returns %d\n", i2);
        fflush(stdout);
      }
    }
  }
}
#endif // WITH_XMODEM


#ifdef STAND_ALONE

// CALIBRATION

void calibrate_loop(HANDLE iFile, HANDLE iConsole)
{
char *p1;
static const char szID[]="Fake Device that does not exist";

  p1 = send_command_get_reply(iFile, "I"); // identify yourself

  if(!p1)
  {
    return;  // error message should have already printed
  }

  if(strncmp(szID, my_ltrim(p1), sizeof(szID) - 1))
  {
    fprintf(stderr, "Equipment ID \"%s\" does not match - exiting\n", p1);
    free(p1);
    return;
  }

  free(p1);

  my_flush(iFile); // get rid of anything else waiting before next command

  p1 = send_command_get_reply(iFile, "E 0"); // echo off

  if(!p1)
  {
    return;  // error message should have already printed
  }

  if(strcmp("ECHO is now OFF", p1))
  {
    fprintf(stderr, "WARNING - ECHO command may not have worked properly\n");
  }

  my_flush(iFile); // flush additional stuff
  free(p1);

  if(!ask_for_user_input_YN(iConsole, "Start calibration process", 0))
  {
    printf("Terminated at user request\n");
    return;
  }

  // sample, do C 2 which should snapshot the "stuff" and remain in cal mode
  p1 = send_command_get_reply(iFile, "C 2"); // cal step 0
  if(!p1)
  {
    return; // error already printed
  }

  // DO SOMETHING WITH THE DATA

  my_flush(iFile); // flush additional stuff
  free(p1);

  // TODO:  print instructions to calibrator (wiring, setup, knobs, whatever)

  if(!ask_for_user_input_YN(iConsole, "Perform next step in calibration process", 0))
  {
    printf("Terminated at user request\n");
    return;
  }

  p1 = send_command_get_reply(iFile, "C 1"); // cal step 1
  if(!p1)
  {
    return; // error already printed
  }

  // DO SOMETHING WITH THE DATA

  my_flush(iFile); // flush additional stuff
  free(p1);


  fputs("Calibration process complete!\n", stdout);
  fflush(stdout);
}

#endif // STAND_ALONE



// **********
// UTILITIES
// **********

// returns != 0 if MyGetTickCount exceeds specified time interval from 'dwStart'
int TimeIntervalExceeds(unsigned int dwStart, unsigned int dwMSec)
{
  return (int)(MyGetTickCount() - (dwStart + dwMSec)) >= 0;
}



// ****************************
// SETTING AND QUERYING OPTIONS
// ****************************

int Verbosity(void)
{
  return iVerbosity;
}

int FactoryReset(void)
{
  return bFactoryReset;
}

int QuitFlag(void)
{
  return bQuitFlag;
}

void SetQuitFlag(void)
{
  bQuitFlag = 1;
}



// ******************************
// CONSOLE AND DEVICE INTERACTION
// ******************************

char * get_reply(HANDLE iFile, int iMaxDelay)
{
unsigned int dwStart;
int i1;
char *pRval, *p1, *pEnd, *pLimit;
unsigned int bOldEchoFlag;


  pRval = malloc(MY_GETS_BUFSIZE * 2 + 1);
  if(!pRval)
  {
    bMyGetsEchoFlag = 1;  // reset it
    return NULL;
  }

  pLimit = pRval + MY_GETS_BUFSIZE;
  pEnd = pRval;

  dwStart = MyGetTickCount();

  while(!TimeIntervalExceeds(dwStart, iMaxDelay))
  {
    if(!my_pollin(iFile)) // if nothing there yet, cycle until there is
    {
      MySleep(1);
      continue;
    }

    // I've got something!

    bOldEchoFlag = bMyGetsEchoFlag; // preserve it, 'my_gets2' resets it
    p1 = my_gets2(iFile, iMaxDelay);
    bMyGetsEchoFlag = bOldEchoFlag; // restore it

    dwStart = MyGetTickCount();

    if(!p1)
    {
      break;
    }

    i1 = strlen(p1);

    if(i1 + 2 + pEnd >= pLimit)
    {
      free(p1);
      break;
    }

    memcpy(pEnd, p1, i1);
    pEnd += i1;

    if(iTerminator)
    {
      *(pEnd++) = iTerminator;
    }
    else
    {
      *(pEnd++) = '\r';
      *(pEnd++) = '\n';
    }

    *pEnd = 0; // always
  }

  bMyGetsEchoFlag = 1;  // reset it

  if(pEnd == pRval)
  {
    free(pRval);
    pRval = NULL; // nothing to return
  }

  return pRval;  // timeout
}

//
// NOTE:  the 'send command get reply' functions ONLY reset 'bMyGetsEchoFlag' - they do not check verbosity nor clear the flag
//        HOWEVER they DO make use of it and restore it for consistency (when necessary), and reset it to 1 before returning
//


char * send_command_get_multiline_reply_with_timeout(HANDLE iFile, const char *szCommand, unsigned int dwTimeout, unsigned int dwRepeatTimeout)
{
unsigned int dwStart, dwStart2;
char *pRval;
int bOldMyGetsEchoFlag;


  if(szCommand)
  {
    my_write(iFile, szCommand, strlen(szCommand));
    my_write(iFile, "\n", 1);  // must be a newline at end
  }
  else
  {
    my_write(iFile, "\x1b", 1); // send an escape
  }

  pRval = NULL;
  dwStart = dwStart2 = MyGetTickCount();

  while(1) // waits on 'my_pollin'
  {
    if(TimeIntervalExceeds(dwStart, dwTimeout)) // more than 'n' milliseconds?
    {
      bMyGetsEchoFlag = 1;  // reset it
      return NULL;
    }
    else if(dwRepeatTimeout && TimeIntervalExceeds(dwStart2, dwRepeatTimeout)) // each second
    {
      if(bCommandRepeatOnTimeoutFlag)
      {
        if(szCommand)
        {
          my_write(iFile, szCommand, strlen(szCommand));
          if(!iTerminator) // CRLF ending
          {
            my_write(iFile, "\r\n", 2);  // must be a CRLF at end
          }
          else if(iTerminator == '\r')
          {
            my_write(iFile, "\r", 1);  // must be a return at end
          }
          else if(iTerminator == '\n')
          {
            my_write(iFile, "\n", 1);  // must be a newline at end
          }
          else
          {
            my_write(iFile, (char *)&iTerminator, 1); // just do this...
          }
        }
        else
        {
          my_write(iFile, "\x1b", 1); // send an escape
        }
      }

      dwStart2 += 1000;
    }

    if(!my_pollin(iFile)) // if nothing there yet, cycle until there is
    {
      MySleep(1);
      continue;
    }

    bOldMyGetsEchoFlag = bMyGetsEchoFlag; // make backup
    pRval = get_reply(iFile, dwTimeout);  // will be something here (this resets bMyGetsEchoFlag to 1)
    bMyGetsEchoFlag = bOldMyGetsEchoFlag; // restore it before continuing loop

    if(!pRval)
    {
      fprintf(stderr, "Not enough memory to continue\n");
      return NULL;
    }

    if(*pRval)
    {
      break;
    }
  }

  bMyGetsEchoFlag = 1; // reset it (make sure)
  return pRval;
}


char * send_command_get_reply_with_timeout(HANDLE iFile, const char *szCommand, unsigned int dwTimeout, unsigned int dwRepeatTimeout)
{
unsigned int dwStart, dwStart2;
char *pRval;
const char *p2;
int bOldMyGetsEchoFlag;

  if(szCommand)
  {
    my_write(iFile, szCommand, strlen(szCommand));
    my_write(iFile, "\n", 1);  // must be a newline at end
  }
  else
  {
    my_write(iFile, "\x1b", 1); // send an escape
  }

  pRval = NULL;
  dwStart = dwStart2 = MyGetTickCount();

  while(1)
  {
    if(TimeIntervalExceeds(dwStart, dwTimeout)) // more than 'n' milliseconds?
    {
      fprintf(stderr, "Unit is not responding\n");
      bMyGetsEchoFlag = 1;  // reset it
      return NULL;
    }
    else if(dwRepeatTimeout && TimeIntervalExceeds(dwStart2, dwRepeatTimeout)) // each second
    {
      if(bCommandRepeatOnTimeoutFlag)
      {
        if(szCommand)
        {
          my_write(iFile, szCommand, strlen(szCommand));
          if(!iTerminator) // CRLF ending
          {
            my_write(iFile, "\r\n", 2);  // must be a CRLF at end
          }
          else if(iTerminator == '\r')
          {
            my_write(iFile, "\r", 1);  // must be a return at end
          }
          else if(iTerminator == '\n')
          {
            my_write(iFile, "\n", 1);  // must be a newline at end
          }
          else
          {
            my_write(iFile, (char *)&iTerminator, 1); // just do this...
          }
        }
        else
        {
          my_write(iFile, "\x1b", 1); // send an escape
        }
      }

      dwStart2 += 1000;
    }

    if(!my_pollin(iFile)) // if nothing there yet, cycle until there is
    {
      MySleep(1);
      continue;
    }

    bOldMyGetsEchoFlag = bMyGetsEchoFlag; // make backup
    pRval = my_gets2(iFile, dwTimeout);  // will be something here (this resets bMyGetsEchoFlag to 1)
    bMyGetsEchoFlag = bOldMyGetsEchoFlag; // restore it before continuing loop

    if(!pRval)
    {
      fprintf(stderr, "Not enough memory to continue\n");
      return NULL;
    }

    p2 = my_ltrim(pRval);

    if(*p2 && (!szCommand || strcmp(p2, szCommand)))  // non-blank line does NOT match my command (not an echo)
    {
      break;
    }

    free(pRval);
    pRval = NULL;
  }

  bMyGetsEchoFlag = 1; // reset it (make sure)
  return pRval;
}

char * send_command_get_reply(HANDLE iFile, const char *szCommand) // returns first non-echo line
{
  // default will wait up to 10 seconds for a reply, repeating the command every 1 second

  return send_command_get_reply_with_timeout(iFile, szCommand, 10000, 1000);
}

int send_command_get_reply_OK(HANDLE iFile, const char *szCommand)
{
char *p1;

  p1 = send_command_get_reply(iFile, szCommand);

  if(!p1 || strcmp(p1,"OK"))
  {
    if(p1)
    {
      free((void *)p1);
    }
    return 0;
  }

  return 1;  // 'OK' result
}

char * ask_for_user_input(HANDLE iConsole, const char * szPrompt)
{
char *pRval;

  if(szPrompt)
  {
    my_write(iStdOut, szPrompt, strlen(szPrompt));
    my_write(iStdOut, " ?", 2);
  }

  pRval = my_gets(iConsole);

  if(!pRval)
  {
    fprintf(stderr, "Not enough memory to continue\n");
  }

  MyGetsEchoOff();
  my_flush(iConsole); // kills all typeahead

  return pRval;
}

int ask_for_user_input_YN(HANDLE iConsole, const char * szPrompt, int iDefault)
{
char * p1;
const char * p2;

  if(szPrompt)
  {
    my_write(iStdOut, szPrompt, strlen(szPrompt));

    if(iDefault)
    {
      my_write(iStdOut, " (n/Y) ?", 8);
    }
    else
    {
      my_write(iStdOut, " (y/N) ?", 8);
    }
  }

  p1 = my_gets(iConsole);

  if(!p1)
  {
    fprintf(stderr, "Not enough memory to continue\n");
    return iDefault; // for now, later setjmp or return -1 ?
  }

  p2 = my_ltrim(p1);

  if(iDefault)
  {
    if(*p2 == 'n' || *p2 == 'N')
    {
      free(p1);
      return 0;
    }
  }
  else
  {
    if(*p2 == 'y' || *p2 == 'Y')
    {
      free(p1);
      return 1;
    }
  }

  free(p1);

  MyGetsEchoOff();
  my_flush(iConsole);

  return iDefault ? 1 : 0;
}

int ask_for_user_input_double(HANDLE iConsole, const char * szPrompt, double *pdRval)
{
char *p1;

  p1 = ask_for_user_input(iConsole, szPrompt);

  if(!p1 ||
     (*p1 != '-' && *p1 != '+' && *p1 != '.'
      && (*p1 < '0' || *p1 > '9'))) // not a number (simple test)
  {
    if(p1)
    {
      free((void *)p1);
      return 1; // meaning there was a return that's not an error, but blank or not a number
    }

    return -1;
  }

  if(pdRval) // allow for NULL, just because
  {
    *pdRval = atof(p1);
  }

  free((void *)p1);

  return 0; // ok
}

int ask_for_user_input_doubleV(HANDLE iConsole, const char * szPrompt, double *pdRval, double dMin, double dMax)
{
  while(!ask_for_user_input_double(iConsole, szPrompt, pdRval))
  {
    if(*pdRval >= dMin && *pdRval <= dMax)
    {
      return 0;  // validation check complete
    }

    fprintf(stderr, "Please enter a value between %g and %g, or <ENTER> to cancel\n",
            dMin, dMax);
    fflush(stderr);
  }

  return 1;  // canceled (not an error at this time)
}

void MyGetsEchoOff(void)
{
  bMyGetsEchoFlag = 0; // good for one input
}

char * my_gets2(HANDLE iFile, unsigned int dwTimeout)
{
int i1, iWasCR = 0;
char c1, *pBuf, *p1, *pEnd;
unsigned int dwStart;


  dwStart = MyGetTickCount();

  pBuf = malloc(MY_GETS_BUFSIZE);
  if(!pBuf)
  {
    bMyGetsEchoFlag = 1; // reset echo flag
    return NULL;
  }

  p1 = pBuf;
  pEnd = p1 + MY_GETS_BUFSIZE - 1;

  do
  {
    i1 = my_pollin(iFile);

    if(!i1)
    {
      continue;
    }

    if(i1 < 0)
    {
      fprintf(stderr, "poll error %d\n", errno);
      bMyGetsEchoFlag = 1; // reset echo flag
      return NULL;
    }

    i1 = my_read(iFile, &c1, 1);

    if(i1 > 0)
    {
      dwStart = MyGetTickCount(); // reset timeout counter whenever I get something

      if(c1 == '\r')
      {
        iWasCR = 1;
      }
      else
      {
        if(iWasCR)
        {
          if(c1 != '\n')
          {
            *(p1++) = '\r';
            if(bMyGetsEchoFlag && !pAltConsole) // never echo for alt console
            {
              my_write(iStdOut, "\r", 1); // output to stdout, always
            }
          }

          iWasCR = 0;
        }

        if(bMyGetsEchoFlag && !pAltConsole) // ever echo for alt console
        {
          if(c1 != '\x08' || p1 > pBuf) // handle backspace only if not at beginning of buffer
          {
            if(c1 == 4 || c1 == 13) // ctrl+d or ctrl+z
            {
              free(pBuf);
              pBuf = NULL;  // for obvious reasons
              p1 = NULL;    // for not-so-obvious reasons
              bQuitFlag = 1; // end the application
              break;
            }

            my_write(iStdOut, &c1, 1); // output to stdout, always

            if(c1 == '\x08')
            {
              my_write(iStdOut, " \x08", 2);  // erase char under cursor
            }
          }
          else
          {
            my_write(iStdOut, "\x07", 1); // bell (traditional, eh?)
          }
        }

        if(c1 == '\n')
        {
          break;  // I am done (do not put '\n' into buffer, it is implied)
        }
        else if(c1 == '\x08') // a backspace
        {
          if(p1 > pBuf)
          {
            *(--p1) = 0;  // erase previous character
          }
        }
        else
        {
          *(p1++) = c1;
        }
      }
    }
    else if(iFile == iStdOut && i1 <= 0 && bIsTCP)
    {
      SetQuitFlag(); // read error when the poll event said there WAS something indicates CLOSED SOCKET
    }
  } while(p1 < pEnd && !TimeIntervalExceeds(dwStart, dwTimeout));

  if(pBuf && p1) // can be NULL
  {
    *p1 = 0; // make sure zero byte at end
  }

  bMyGetsEchoFlag = 1; // reset echo flag
  return pBuf;
}

char * my_gets(HANDLE iFile) // line must end in '\n' or '\r\n', auto-echo to stdout
{
int i1, iWasCR = 0;
char c1, *pBuf, *p1, *pEnd;

  pBuf = malloc(MY_GETS_BUFSIZE);
  if(!pBuf)
  {
    bMyGetsEchoFlag = 1; // reset echo flag
    return NULL;
  }

  p1 = pBuf;
  pEnd = p1 + MY_GETS_BUFSIZE - 1;

  do
  {
    i1 = my_pollin(iFile);

    if(!i1)
    {
      continue;
    }

    if(i1 < 0)
    {
      fprintf(stderr, "poll error %d\n", errno);
      bMyGetsEchoFlag = 1; // reset echo flag
      return NULL;
    }

    i1 = my_read(iFile, &c1, 1);

    if(i1 > 0)
    {
      if(c1 == '\r')
      {
        iWasCR = 1;
      }
      else
      {
        if(iWasCR)
        {
          if(c1 != '\n')
          {
            *(p1++) = '\r';
            if(bMyGetsEchoFlag && !pAltConsole) // never echo for alt console
            {
              my_write(iStdOut, "\r", 1); // output to stdout, always
            }
          }

          iWasCR = 0;
        }

        if(bMyGetsEchoFlag && !pAltConsole) // ever echo for alt console
        {
          if(c1 != '\x08' || p1 > pBuf) // handle backspace only if not at beginning of buffer
          {
            if(c1 == 4 || c1 == 13) // ctrl+d or ctrl+z
            {
              free(pBuf);
              pBuf = NULL;  // for obvious reasons
              p1 = NULL;    // for not-so-obvious reasons
              bQuitFlag = 1; // end the application
              break;
            }

            my_write(iStdOut, &c1, 1); // output to stdout, always

            if(c1 == '\x08')
            {
              my_write(iStdOut, " \x08", 2);  // erase char under cursor
            }
          }
          else
          {
            my_write(iStdOut, "\x07", 1); // bell (traditional, eh?)
          }
        }

        if(c1 == '\n')
        {
          break;  // I am done (do not put '\n' into buffer, it is implied)
        }
        else if(c1 == '\x08') // a backspace
        {
          if(p1 > pBuf)
          {
            *(--p1) = 0;  // erase previous character
          }
        }
        else
        {
          *(p1++) = c1;
        }
      }
    }
    else if(iFile == iStdOut && i1 <= 0 && bIsTCP)
    {
      SetQuitFlag(); // read error when the poll event said there WAS something indicates CLOSED SOCKET
    }
  } while(p1 < pEnd);

  if(pBuf && p1) // can be NULL
  {
    *p1 = 0;
  }

  bMyGetsEchoFlag = 1; // reset echo flag
  return pBuf;
}

const char * my_ltrim(const char *pStr)
{
  while(*pStr && *pStr <= ' ')
  {
    pStr++;
  }

  return pStr;
}



//====================================================================

// this is the OS-specific section


// **********************************
// OS-DEPENDENT I/O helper utilities
// **********************************

#ifndef WIN32 /* POSIX versions - much simpler, straightforward */

// --------------
// POSIX VERSIONS
// --------------

void MySleep(unsigned int dwMsec)
{
unsigned int dwEnd = MyGetTickCount() + dwMsec;

  while((int)(dwEnd - MyGetTickCount()) > 0)
  {
    usleep(100);
  }
}

unsigned int MyGetTickCount()
{
  struct timeval tv;
  gettimeofday(&tv, NULL); // 2nd parameter is obsolete anyway

  // NOTE:  this won't roll over the way 'GetTickCount' does in WIN32 so I'll truncate it
  //        down to a 32-bit value to make it happen.  Everything that uses 'MyGetTickCount'
  //        must handle this rollover properly using 'int' and not 'long' (or cast afterwards)
  return((unsigned int)((unsigned long)tv.tv_sec * 1000L + (unsigned long)tv.tv_usec / 1000L));
}

int my_pollin(HANDLE iFile)
{
struct pollfd sFD;
int i1;

  sFD.fd = iFile;
  sFD.events = POLLIN | POLLERR;
  sFD.revents = 0;

  i1 = poll(&sFD, 1, 100);

  // must check for error first
  if(i1 < 0 || (sFD.revents & POLLERR))
  {
    return -1;
  }

  if(i1 > 0 && (sFD.events & POLLIN))
  {
    return 1;
  }

  return 0;  // nothing
}

void my_flush(HANDLE iFile)
{
int i1;
char c1;

  while(my_pollin(iFile) > 0)
  {
    i1 = my_read(iFile, &c1, 1);
    if(i1 > 0 && c1 != '\r' && bMyGetsEchoFlag)
    {
      my_write(1, &c1, 1);
    }
    else if(iFile == iStdOut && i1 <= 0 && bIsTCP)
    {
      SetQuitFlag(); // read error when the poll event said there WAS something indicates CLOSED SOCKET
      break;
    }
  }

  bMyGetsEchoFlag = 1;  // reset it
}

int my_write(HANDLE iFile, const void *pBuf, int cbBuf)
{
  return write(iFile, pBuf, cbBuf);
}

int my_read(HANDLE iFile, void *pBuf, int cbBuf)
{
  return read(iFile, pBuf, cbBuf);
}

void set_rts_dtr(HANDLE iFile, int bSet)
{
unsigned int sFlags;

  ioctl(iFile, TIOCMGET, &sFlags);

  if(bSet)
  {
    sFlags |= TIOCM_RTS | TIOCM_DTR;
  }
  else
  {
    sFlags &= ~(TIOCM_RTS | TIOCM_DTR);
  }

  ioctl(iFile, TIOCMSET, &sFlags);
}

void reset_arduino(HANDLE iFile)
{
unsigned int sFlags;
// toggle the RTS and DTR high, low, then high - so much easier via POSIX-compatible OS!

  ioctl(iFile, TIOCMGET, &sFlags);

//  sFlags |= TIOCM_DTR | TIOCM_RTS;
//  ioctl(iFile, TIOCMSET, &sFlags);
//
//  MySleep(50);

  sFlags &= ~(TIOCM_DTR | TIOCM_RTS); // the high to low transition discharges the capacitor (signal is inverted on board)
  if(ioctl(iFile, TIOCMSET, &sFlags) < 0)
  {
    fprintf(stderr, "WARNING:  ioctl() returns < 0, errno=%d (%xH)\n", errno, errno);
  }

  MySleep(250); // avrdude does this for 50 msecs, my change has it at 50msecs

  sFlags |= TIOCM_DTR | TIOCM_RTS; // leave it in THIS state when I'm done
  if(ioctl(iFile, TIOCMSET, &sFlags) < 0)
  {
    fprintf(stderr, "WARNING:  ioctl() returns < 0, errno=%d (%xH)\n", errno, errno);
  }

  MySleep(50); // avrdude does this for 50 msecs (no change)

  MyGetsEchoOff();
  my_flush(iFile); // avrdude does this too - flush any extraneous input

  MySleep(500);  // And I allow 1/2 second for device reset
}

HANDLE altconconfig(HANDLE iFile)
{
#if 0 /* in case I need to do something here i.e. "not a pipe" and "not a socket" */
struct termios sIOS;

//  i1 = fcntl(iFile, F_GETFL);
//  fcntl(iFile, F_SETFL, i1 | O_NONBLOCK); // turn ON non-blocking?

  if(!tcgetattr(iFile, &sIOS))
  {
    // make sure echoing and ALL translations are disabled especially CTRL+C handling

    // do not translate characters or xon/xoff and ignore break
    sIOS.c_iflag &= ~(IGNBRK | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | IMAXBEL | ISTRIP); // turn these off

#if defined(__FreeBSD__)
    sIOS.c_oflag &= ~(OPOST | ONLCR | OCRNL | TABDLY | ONOEOT | ONOCR | ONLRET); // FreeBSD version
#else // Linux? YMMV
    sIOS.c_oflag &= ~(OPOST | ONLCR | OCRNL | TABDLY | ONOCR | ONLRET); // turn these off too (see man termios)
#endif // FBSD vs Linux

    // make sure echoing is disabled and control chars aren't translated or omitted
#if defined(__FreeBSD__)
    sIOS.c_lflag &= ~(ECHO | ECHOKE | ECHOE | ECHONL | ECHOPRT | ECHOCTL | ICANON | IEXTEN | ISIG | ALTWERASE);
#else // Linux? YMMV
    sIOS.c_lflag &= ~(ECHO | ECHOKE | ECHOE | ECHONL | ECHOPRT | ECHOCTL | ICANON | IEXTEN | ISIG);
#endif // FBSD vs Linux
    sIOS.c_cc[VMIN] = 0;  // ensures no 'grouping' of input
    sIOS.c_cc[VTIME] = 0; // immediate return

    if(tcsetattr(iFile, TCSANOW, &sIOS))
    {
      fprintf(stderr, "error %d setting console attributes\n", errno);
    }
  }
  else
  {
    fprintf(stderr, "error %d getting console attributes\n", errno);
  }
#endif // 0
  return iFile;  // return same handle that was passed to me
}

void conrestore(void)
{
  if(hIOSConsoleRestoreHandle >= 0)
  {
    tcsetattr(hIOSConsoleRestoreHandle, TCSANOW, &sIOSConsole); // restore console

#ifndef WIN32
    close(hIOSConsoleRestoreHandle);
#endif // WIN32
    hIOSConsoleRestoreHandle = -1;
  }
}

HANDLE conconfig(HANDLE iFile)
{
int i1;
struct termios sIOS;

  i1 = fcntl(iFile, F_GETFL);
  fcntl(iFile, F_SETFL, i1 | O_NONBLOCK); // turn ON non-blocking?

  memset(&sIOSConsole, 0, sizeof(sIOSConsole));
  hIOSConsoleRestoreHandle = -1;

  if(!tcgetattr(iFile, &sIOS))
  {
    memcpy(&sIOSConsole, &sIOS, sizeof(sIOSConsole)); // so I can restore it
#ifdef WIN32
    hIOSConsoleRestoreHandle = iFile;
#else // WIN32
    hIOSConsoleRestoreHandle = dup(iFile); // copy of console handle (so I can restore it)
#endif // WIN32

    // make sure echoing is disabled
    sIOS.c_lflag &= ~(ECHO | ECHOKE | ECHOE | ECHONL
#ifdef ECHOPRT
                  | ECHOPRT
#else
#warning no 'ECHOPRT'
#endif // ECHOPRT
                  | ECHOCTL | ICANON);
    sIOS.c_cc[VMIN] = 0;  // ensures no 'grouping' of input
    sIOS.c_cc[VTIME] = 0; // immediate return

    if(tcsetattr(iFile, TCSANOW, &sIOS))
    {
      fprintf(stderr, "error %d setting console attributes\n", errno);
    }
  }
  else
  {
    fprintf(stderr, "error %d getting console attributes\n", errno);
  }

  return iFile;  // return same handle that was passed to me
}

static void dump_serial_attr(struct termios *pIOS)
{
static const char * const asz_c_iflag[] =
{
  "IGNBRK",   /* ignore BREAK condition */
  "BRKINT",   /* map BREAK to SIGINTR */
  "IGNPAR",   /* ignore (discard) parity errors */
  "PARMRK",   /* mark parity and framing errors */
  "INPCK",    /* enable checking of parity errors */
  "ISTRIP",   /* strip 8th bit off chars */
  "INLCR",    /* map NL into CR */
  "IGNCR",    /* ignore CR */
  "ICRNL",    /* map CR to NL (ala CRMOD) */
  "IXON",     /* enable output flow control */
  "IXOFF",    /* enable input flow control */
  "IXANY",    /* any char will restart after stop */
  "IMAXBEL"   /* ring bell on input queue full */
};
static const int a_c_iflag[] =
{
  IGNBRK,   /* ignore BREAK condition */
  BRKINT,   /* map BREAK to SIGINTR */
  IGNPAR,   /* ignore (discard) parity errors */
  PARMRK,   /* mark parity and framing errors */
  INPCK,    /* enable checking of parity errors */
  ISTRIP,   /* strip 8th bit off chars */
  INLCR,    /* map NL into CR */
  IGNCR,    /* ignore CR */
  ICRNL,    /* map CR to NL (ala CRMOD) */
  IXON,     /* enable output flow control */
  IXOFF,    /* enable input flow control */
  IXANY,    /* any char will restart after stop */
  IMAXBEL   /* ring bell on input queue full */
};
static const char * const asz_c_oflag[] =
{
  "OPOST",   /* enable following output processing */
  "ONLCR",   /* map NL to CR-NL (ala CRMOD) */
  "OCRNL",   /* map CR to NL */
#ifdef __FreeBSD__
  "OXTABS",  /* expand tabs to spaces */
  "ONOEOT",  /* discard EOT's `^D' on output) */
#endif // __FreeBSD__
  "ONOCR",   /* do not transmit CRs on column 0 */
  "ONLRET"   /* on the terminal NL performs the CR function */
};
static const int a_c_oflag[] =
{
  OPOST,   /* enable following output processing */
  ONLCR,   /* map NL to CR-NL (ala CRMOD) */
  OCRNL,   /* map CR to NL */
#ifdef __FreeBSD__
  OXTABS,  /* expand tabs to spaces */
  ONOEOT,  /* discard EOT's `^D' on output) */
#endif // __FreeBSD__
  ONOCR,   /* do not transmit CRs on column 0 */
  ONLRET   /* on the terminal NL performs the CR function */
};
static const char * const asz_c_cflag[] =
{
//  "CSIZE",       /* character size mask */
//  "CS5",         /* 5 bits (pseudo) */
//  "CS6",         /* 6 bits */
//  "CS7",         /* 7 bits */
//  "CS8",         /* 8 bits */
  "CSTOPB",      /* send 2 stop bits */
  "CREAD",       /* enable receiver */
  "PARENB",      /* parity enable */
  "PARODD",      /* odd parity, else even */
  "HUPCL",       /* hang up on last close */
  "CLOCAL",      /* ignore modem status lines */
#ifdef __FreeBSD__
  "CCTS_OFLOW",  /* CTS flow control of output */
#endif // __FreeBSD__
  "CRTSCTS",     /* same as CCTS_OFLOW */
#ifdef __FreeBSD__
  "CRTS_IFLOW",  /* RTS flow control of input */
  "MDMBUF"       /* flow control output via Carrier */
#endif // __FreeBSD__
};
static const int a_c_cflag[] =
{
//  CSIZE,       /* character size mask */
//  CS5,         /* 5 bits (pseudo) */
//  CS6,         /* 6 bits */
//  CS7,         /* 7 bits */
//  CS8,         /* 8 bits */
  CSTOPB,      /* send 2 stop bits */
  CREAD,       /* enable receiver */
  PARENB,      /* parity enable */
  PARODD,      /* odd parity, else even */
  HUPCL,       /* hang up on last close */
  CLOCAL,      /* ignore modem status lines */
#ifdef __FreeBSD__
  CCTS_OFLOW,  /* CTS flow control of output */
#endif // __FreeBSD__
  CRTSCTS,     /* same as CCTS_OFLOW */
#ifdef __FreeBSD__
  CRTS_IFLOW,  /* RTS flow control of input */
  MDMBUF       /* flow control output via Carrier */
#endif // __FreeBSD__
};
static const char * const asz_c_lflag[] =
{
  "ECHOKE",      /* visual erase for line kill */
  "ECHOE",       /* visually erase chars */
  "ECHO",        /* enable echoing */
  "ECHONL",      /* echo NL even if ECHO is off */
#ifdef ECHOPRT
  "ECHOPRT",     /* visual erase mode for hardcopy */
#endif // ECHOPRT
  "ECHOCTL",     /* echo control chars as ^(Char) */
  "ISIG",        /* enable signals INTR, QUIT, [D]SUSP */
  "ICANON",      /* canonicalize input lines */
#ifdef __FreeBSD__
  "ALTWERASE",   /* use alternate WERASE algorithm */
#endif // __FreeBSD__
  "IEXTEN",      /* enable DISCARD and LNEXT */
#ifdef __FreeBSD__
  "EXTPROC",     /* external processing */
#endif // __FreeBSD__
  "TOSTOP",      /* stop background jobs from output */
  "FLUSHO",      /* output being flushed (state) */
#ifdef __FreeBSD__
  "NOKERNINFO",  /* no kernel output from VSTATUS */
#endif // __FreeBSD__
#ifdef PENDIN
  "PENDIN",      /* XXX retype pending input (state) */
#endif // PENDIN
  "NOFLSH"       /* don't flush after interrupt */
};
static const int a_c_lflag[] =
{
  ECHOKE,      /* visual erase for line kill */
  ECHOE,       /* visually erase chars */
  ECHO,        /* enable echoing */
  ECHONL,      /* echo NL even if ECHO is off */
#ifdef ECHOPRT
  ECHOPRT,     /* visual erase mode for hardcopy */
#endif // ECHOPRT
  ECHOCTL,     /* echo control chars as ^(Char) */
  ISIG,        /* enable signals INTR, QUIT, [D]SUSP */
  ICANON,      /* canonicalize input lines */
#ifdef __FreeBSD__
  ALTWERASE,   /* use alternate WERASE algorithm */
#endif // __FreeBSD__
  IEXTEN,      /* enable DISCARD and LNEXT */
#ifdef __FreeBSD__
  EXTPROC,     /* external processing */
#endif // __FreeBSD__
  TOSTOP,      /* stop background jobs from output */
  FLUSHO,      /* output being flushed (state) */
#ifdef __FreeBSD__
  NOKERNINFO,  /* no kernel output from VSTATUS */
#endif // __FreeBSD__
#ifdef PENDIN
  PENDIN,      /* XXX retype pending input (state) */
#endif // PENDIN
  NOFLSH       /* don't flush after interrupt */
};
int i1, i2;


  fprintf(stderr, "  c_iflag: %d (%08xH)", pIOS->c_iflag, pIOS->c_iflag);

  for(i1=0, i2=0; i1 < sizeof(asz_c_iflag) / sizeof(asz_c_iflag[0]); i1++)
  {
    if(pIOS->c_iflag & a_c_iflag[i1])
    {
      if(i2)
      {
        fputc(',',stderr);
      }

      i2=1;
      fputs(asz_c_iflag[i1], stderr);
    }
  }

  fputc('\n', stderr);


  fprintf(stderr, "  c_oflag: %d (%08xH)", pIOS->c_oflag, pIOS->c_oflag);

  for(i1=0, i2=0; i1 < sizeof(asz_c_oflag) / sizeof(asz_c_oflag[0]); i1++)
  {
    if(pIOS->c_oflag & a_c_oflag[i1])
    {
      if(i2)
      {
        fputc(',',stderr);
      }

      i2=1;
      fputs(asz_c_oflag[i1], stderr);
    }
  }

  fputc('\n', stderr);


  fprintf(stderr, "  c_cflag: %d (%08xH)", pIOS->c_cflag, pIOS->c_cflag);

  for(i1=0, i2=0; i1 < sizeof(asz_c_cflag) / sizeof(asz_c_cflag[0]); i1++)
  {
    if(pIOS->c_cflag & a_c_cflag[i1])
    {
      if(i2)
      {
        fputc(',',stderr);
      }

      i2=1;
      fputs(asz_c_cflag[i1], stderr);
    }
  }

  if(i2)
  {
    fputc(',',stderr);
  }

  if((pIOS->c_cflag & CSIZE) == CS5)
  {
    fputs("CS5",stderr);
  }
  else if((pIOS->c_cflag & CSIZE) == CS6)
  {
    fputs("CS6",stderr);
  }
  else if((pIOS->c_cflag & CSIZE) == CS7)
  {
    fputs("CS7",stderr);
  }
  else if((pIOS->c_cflag & CSIZE) == CS8)
  {
    fputs("CS8",stderr);
  }
  else
  {
    fputs("ERR (bits)", stderr);
  }

  fputc('\n', stderr);


  fprintf(stderr, "  c_lflag: %d (%08xH)", pIOS->c_lflag, pIOS->c_lflag);

  for(i1=0, i2=0; i1 < sizeof(asz_c_lflag) / sizeof(asz_c_lflag[0]); i1++)
  {
    if(pIOS->c_lflag & a_c_lflag[i1])
    {
      if(i2)
      {
        fputc(',',stderr);
      }

      i2=1;
      fputs(asz_c_lflag[i1], stderr);
    }
  }

  fputc('\n', stderr);


}

void ttyconfig(HANDLE iFile, int iBaud, int iParity, int iBits, int iStop)
{
int i1;
struct termios sIOS;
#if defined(__linux__) && defined(LINUX_SPECIAL_HANDLING)
int bDoThisAgainForLinux = 0;
#endif // defined(__linux__)


  i1 = fcntl(iFile, F_GETFL);
  if(bSerialDebug)
  {
    fprintf(stderr, "SERIAL FLAGS (before):  %d (%08xH)\n", i1, i1);
  }

  i1 |= O_NONBLOCK; // i1 &= ~O_NONBLOCK); // turn OFF non-blocking?

  fcntl(iFile, F_SETFL, i1);

  if(bSerialDebug)
  {
    fprintf(stderr, "SERIAL FLAGS (after):  %d (%08xH)\n", i1, i1);
  }

  if(!tcgetattr(iFile, &sIOS))
  {
    if(bSerialDebug)
    {
      fprintf(stderr, "SERIAL ATTRIBUTES (before)\n");
      dump_serial_attr(&sIOS);
    }

#ifdef __CYGWIN__
    cfsetispeed(&sIOS, iBaud);
    cfsetospeed(&sIOS, iBaud);
#elif defined(__linux__) && defined(LINUX_SPECIAL_HANDLING)
    if(cfsetspeed(&sIOS, iBaud) == -1) // a non-standard baud rate?
    {
//      bDoThisAgainForLinux = 1;
      sIOS.c_cflag &= ~CBAUD;
      sIOS.c_cflag |= CBAUDEX | ((CBAUDEX) << 16)/*BOTHER*/; // "other" baud rate (note BOTHER defined in asm-generic/termbits.h == CBAUDEX)

      sIOS.c_ispeed = sIOS.c_ospeed = iBaud;

      fprintf(stderr, "WARNING:  linux may not support %d baud\n", iBaud);
    }
#else // !__CYGWIN__, !__linux__
    cfsetspeed(&sIOS, iBaud); // just do it
#endif // __CYGWIN__, __linux__
    sIOS.c_cflag &= ~(CSIZE|PARENB|CS5|CS6|CS7|CS8);
    sIOS.c_cflag |= iBits == 5 ? CS5 : iBits == 6 ? CS6 : iBits == 7 ? CS7 : CS8; // 8 is default
    if(iStop == 2)
    {
      sIOS.c_cflag |= CSTOPB;
    }
    else
    {
      sIOS.c_cflag &= ~CSTOPB;
    }

    if(iFlowControl > 0)
    {
      sIOS.c_cflag |= CRTSCTS; // hardware flow control
#if defined(__FreeBSD__)
      sIOS.c_cflag |= CRTS_IFLOW;
#endif // FBSD vs Linux
    }
    else
    {
      sIOS.c_cflag &= ~CRTSCTS; // hardware flow control _DISABLED_ (so I can do the reset)
#if defined(__FreeBSD__)
      sIOS.c_cflag &= ~CRTS_IFLOW;
#endif // FBSD vs Linux
    }

    sIOS.c_cflag |= CLOCAL; // ignore any modem status lines

    if(!iParity)
    {
      sIOS.c_cflag &= ~(PARENB | PARODD);
    }
    else if(iParity > 0) // odd
    {
      sIOS.c_cflag |= (PARENB | PARODD);
    }
    else // even (negative)
    {
      sIOS.c_cflag &= PARODD;
      sIOS.c_cflag |= PARENB;
    }

    // do not translate characters or xon/xoff and ignore break
    sIOS.c_iflag &= ~(IGNBRK | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | IMAXBEL | ISTRIP); // turn these off

#if defined(__FreeBSD__)
    sIOS.c_oflag &= ~(OPOST | ONLCR | OCRNL | TABDLY | ONOEOT | ONOCR | ONLRET); // FreeBSD version
#else // Linux? YMMV
    sIOS.c_oflag &= ~(OPOST | ONLCR | OCRNL | TABDLY | ONOCR | ONLRET); // turn these off too (see man termios)
#endif // FBSD vs Linux

    // make sure echoing is disabled and control chars aren't translated or omitted
#if defined(__FreeBSD__)
    sIOS.c_lflag &= ~(ECHO | ECHOKE | ECHOE | ECHONL | ECHOPRT | ECHOCTL | ICANON | IEXTEN | ISIG | ALTWERASE);
#else // Linux? YMMV
    sIOS.c_lflag &= ~(ECHO | ECHOKE | ECHOE | ECHONL
#ifdef ECHOPRT
                  | ECHOPRT
#else
#warning no 'ECHOPRT'
#endif // ECHOPRT
                  | ECHOCTL | ICANON | IEXTEN | ISIG);
#endif // FBSD vs Linux
    sIOS.c_cc[VMIN] = 0;  // ensures no 'grouping' of input
    sIOS.c_cc[VTIME] = 0; // immediate return

    if(tcsetattr(iFile, TCSANOW, &sIOS))
    {
      fprintf(stderr, "error %d setting attributes\n", errno);
    }

    if(bSerialDebug)
    {
      fprintf(stderr, "SERIAL ATTRIBUTES (after)\n");
      dump_serial_attr(&sIOS);
    }

#if defined(__linux__) && defined(LINUX_SPECIAL_HANDLING)
    if(bDoThisAgainForLinux)
    {
      // this definition is derived from asm-generic/ioctls.h and asm-generic/termbits.h
      struct termios2 // need to define this for the IOCTLs' sake
      {
        struct termios t;
      };

      if(ioctl(iFile, TCGETS2, &sIOS)) // re-get settings THIS way
      {
        fprintf(stderr, "error %d getting termios for baud rate change\n", errno);
        dump_serial_attr(&sIOS);
      }

      sIOS.c_cflag &= ~CBAUD;
      sIOS.c_cflag |= CBAUDEX | ((CBAUDEX) << 16)/*BOTHER*/; // "other" baud rate (note BOTHER defined in asm-generic/termbits.h == CBAUDEX)

      sIOS.c_ispeed = sIOS.c_ospeed = iBaud;
//      cfsetispeed(&sIOS, iBaud);
//      cfsetospeed(&sIOS, iBaud);

      if(ioctl(iFile, TCSETS2, &sIOS)) // must be done last
      {
        fprintf(stderr, "error %d setting baud rate\n", errno);
        dump_serial_attr(&sIOS);
      }
    }
#endif // __linux__
  }
  else
  {
    fprintf(stderr, "error %d getting attributes\n", errno);
  }
}

#else // WIN32 versions - unnecessarily complicated, full of pitfalls

// ----------------
// WINDOWS VERSIONS
// ----------------

// windows has a lot of limitations with respect to serial I/O and
// the need for using 'overlapped', inability to (easily) peek at
// the console or the I/O, yotta yotta yotta so I end up using
// worker threads and bastardized algorithms.  Hence, THIS section.

#define USE_CRITICAL_SECTION /* un-define this to go back to using a mutex */
#define COM_BUFSIZE 128

volatile unsigned int nConThreadID = 0;
volatile HANDLE hConFile = INVALID_HANDLE_VALUE;
volatile HANDLE hConThread = INVALID_HANDLE_VALUE;
#ifdef USE_CRITICAL_SECTION
CRITICAL_SECTION csCon;
volatile LPCRITICAL_SECTION pConCS = NULL;
#else  // USE_CRITICAL_SECTION
volatile HANDLE hConMutex = INVALID_HANDLE_VALUE;
#endif // USE_CRITICAL_SECTION
volatile char aConBufIn[4096];
volatile int iConHTInB=0, iConHTInE=0;

volatile unsigned int nTtyThreadID = 0;
volatile HANDLE hTtyFile = INVALID_HANDLE_VALUE;
volatile HANDLE hTtyThread = INVALID_HANDLE_VALUE;
#ifdef USE_CRITICAL_SECTION
CRITICAL_SECTION csTty;
volatile LPCRITICAL_SECTION pTtyCS = NULL;
#else  // USE_CRITICAL_SECTION
volatile HANDLE hTtyMutex = INVALID_HANDLE_VALUE;
#endif // USE_CRITICAL_SECTION
volatile char aSerBufIn[4096], aSerBufOut[4096];
volatile int iSerHTInB=0, iSerHTInE=0, iSerHTOutB=0, iSerHTOutE=0;

__inline int HeadTailDataAvailable(int B, int E) { return E != B; }
__inline int HeadTailDataOverflow(int B, int E, int Z) { return ((E + 1) % Z == B); }
__inline int HeadTailDataInBuffer(int B, int E, int Z) { return (E + Z - B) % Z; }
__inline int HeadTailBufferLeft(int B, int E, int Z) { return Z - 1 - (E + Z - B) % Z; }

UINT APIENTRY TtyThreadEntry(void *pParam); // thread procs
UINT APIENTRY ConThreadEntry(void *pParam);



void HeadTailPush(char cAdd, volatile int *pB, volatile int *pE,
                  volatile char *pBuf, int cbSize)
{
  register int iB = *pB;
  register int iE = *pE;

  pBuf[iE] = cAdd;

  iE++;
  if(iE >= cbSize)
  {
    iE = 0;
  }
  if(iB == iE) // overload
  {
    iB++;  // lose the oldest data (for now)

    if(iB >= cbSize)
    {
      iB = 0;
    }
  }

  *pE = iE;
  *pB = iB;
}

int HeadTailPop(volatile int *pB, int iE, volatile const char *pBuf, int cbSize, char *pRval)
{
  register int iB = *pB;

  if(iB == iE) // empty
  {
    return 0;
  }

  *pRval = pBuf[iB];

  iB++;
  if(iB >= cbSize)
  {
    iB = 0;
  }

  *pB = iB;

  return 1; // non-zero means data available
}

// now for the required API implementation

void MySleep(unsigned int dwMsec)
{
unsigned int dwTick = MyGetTickCount();
unsigned int dwEnd = dwTick + dwMsec;
int iDelta;

  while((iDelta = (int)(dwEnd - dwTick)) > 0)
  {
    if(iDelta > 10)
    {
      Sleep(iDelta / 4);
    }
    else
    {
      Sleep(1);
    }

    dwTick = MyGetTickCount();
  }
}

unsigned int MyGetTickCount()
{
  return GetTickCount();
}

int my_pollin(HANDLE iFile)
{
int iRval = 0;

  if(iFile == hConFile)
  {
    static int iLastTime = 0;
#ifdef USE_CRITICAL_SECTION
    DWORD dwStart;

try_again_two:
    dwStart = MyGetTickCount();
    while(!TryEnterCriticalSection(pConCS))
    {
      if(TimeIntervalExceeds(dwStart, 2)) // more than 1 msec (2 or more, in other words)
      {
        return 0;  // assume 'nothing available' until mutex is released
      }
      Yield(); // lose a time slice (needed for single core)
    }
#else // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hConMutex, 1) != WAIT_OBJECT_0)
    {
      return 0;  // assume 'nothing available' until mutex is released
    }
#endif // USE_CRITICAL_SECTION

    iRval = HeadTailDataAvailable(iConHTInB, iConHTInE);

#ifdef USE_CRITICAL_SECTION
    LeaveCriticalSection(pConCS);
#else // USE_CRITICAL_SECTION
    ReleaseMutex(hConMutex);
#endif // USE_CRITICAL_SECTION

    // this section limits the 'spinning'
    if(!iRval)
    {
      if(iLastTime <= 0)
      {
        Sleep(2);

        if(!iLastTime)
        {
          iLastTime--;
          goto try_again_two;
        }
      }
      else
      {
        iLastTime--;
      }
    }
    else
    {
      iLastTime = 5;
    }
  }
  else if(iFile == hTtyFile)
  {
    static int iLastTime = 0;
#ifdef USE_CRITICAL_SECTION
    DWORD dwStart;

try_again_one:

    dwStart = MyGetTickCount();
    while(!TryEnterCriticalSection(pTtyCS))
    {
      if(TimeIntervalExceeds(dwStart, 2)) // as before, 'more than 1' is 2 or more
      {
        return 0;  // assume 'nothing available' until mutex is released
      }
      Yield(); // lose a time slice (needed for single core)
    }
#else // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hTtyMutex, 1) != WAIT_OBJECT_0)
    {
      return 0;  // assume 'nothing available' until mutex is released
    }
#endif // USE_CRITICAL_SECTION

    iRval = HeadTailDataAvailable(iSerHTInB, iSerHTInE);

#ifdef USE_CRITICAL_SECTION
    LeaveCriticalSection(pTtyCS);
#else // USE_CRITICAL_SECTION
    ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

    // this section limits the 'spinning'
    if(!iRval)
    {
      if(iLastTime <= 0)
      {
        Sleep(2);

        if(!iLastTime)
        {
          iLastTime--;
          goto try_again_one;
        }
      }
      else
      {
        iLastTime--;
      }
    }
    else
    {
      iLastTime = 5;
    }
  }

  return iRval;
}

void my_flush(HANDLE iFile)
{
int i1;
char c1;

  while((i1 = my_read(iFile, &c1, 1)) > 0)
  {
    if(c1 != '\r' && bMyGetsEchoFlag)
    {
      my_write(iStdOut, &c1, 1);
    }
  }

  bMyGetsEchoFlag = 1;  // reset it
}

int my_write(HANDLE iFile, const void *pBuf, int cbBuf)
{
DWORD cb1 = 0;
int cbTotal = 0;

  if(iFile == hTtyFile)
  {

#ifdef USE_CRITICAL_SECTION
    EnterCriticalSection(pTtyCS);
#else // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
    {
      return cbTotal; // not all were written
    }
#endif // USE_CRITICAL_SECTION

    for(cbTotal=0; cbTotal < cbBuf; cbTotal++)
    {
      // if head/tail overflows, wait for it
      while(HeadTailDataOverflow(iSerHTOutB, iSerHTOutE, sizeof(aSerBufOut)))
      {
#ifdef USE_CRITICAL_SECTION
        LeaveCriticalSection(pTtyCS);
#else // USE_CRITICAL_SECTION
        ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

        MySleep(10);

#ifdef USE_CRITICAL_SECTION
        EnterCriticalSection(pTtyCS);
#else // USE_CRITICAL_SECTION
        if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
        {
          return cbTotal; // not all were written
        }
#endif // USE_CRITICAL_SECTION
      }

      HeadTailPush(((char *)pBuf)[cbTotal], &iSerHTOutB, &iSerHTOutE,
                   aSerBufOut, sizeof(aSerBufOut));
    }

#ifdef USE_CRITICAL_SECTION
    LeaveCriticalSection(pTtyCS);
#else // USE_CRITICAL_SECTION
    ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

    SetCommMask(hTtyFile, COMM_RW_EV); // enable wait for RX and TX
      // this will abandon any comm event wait states according to MSDN docs

    return cbTotal;
  }

  do
  {
    if(!WriteFile(iFile, pBuf, cbBuf, &cb1, NULL))
    {
      if(GetLastError() == ERROR_IO_PENDING)
      {
        MySleep(2);
        continue;
      }

      return -1; // to indicate error
    }
    cbTotal += cb1;
    cbBuf -= cb1;
  } while(cbBuf > 0);

  return cbTotal;
}

int my_read(HANDLE iFile, void *pBuf, int cbBuf)
{
DWORD cb1 = 0;
char c1;

  if(iFile == hTtyFile)
  {
#ifdef USE_CRITICAL_SECTION
    DWORD dwStart = MyGetTickCount();
    while(!TryEnterCriticalSection(pTtyCS))
    {
      if(TimeIntervalExceeds(dwStart, 2)) // more than 2 msec
      {
        return 0;  // assume 'nothing available' until mutex is released
      }
      Yield(); // lose a time slice (needed for single core)
    }
#else // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hTtyMutex, 10) != WAIT_OBJECT_0)
    {
      return 0;  // assume 'nothing available' until mutex is released
    }
#endif // USE_CRITICAL_SECTION

    while((int)cb1 < cbBuf &&
          HeadTailPop(&iSerHTInB, iSerHTInE, aSerBufIn, sizeof(aSerBufIn), &c1))
    {
      ((char *)pBuf)[cb1++] = c1;
    }

#ifdef USE_CRITICAL_SECTION
    LeaveCriticalSection(pTtyCS);
#else // USE_CRITICAL_SECTION
    ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION
    return cb1;
  }

  if(iFile == hConFile)
  {
#ifdef USE_CRITICAL_SECTION
    DWORD dwStart = MyGetTickCount();
    while(!TryEnterCriticalSection(pConCS))
    {
      if(TimeIntervalExceeds(dwStart, 2)) // more than 2 msec
      {
        return 0;  // assume 'nothing available' until mutex is released
      }
      Yield(); // lose a time slice (needed for single core)
    }
#else // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hConMutex, 10) != WAIT_OBJECT_0)
    {
      return 0;  // assume 'nothing available' until mutex is released
    }
#endif // USE_CRITICAL_SECTION

    while((int)cb1 < cbBuf &&
          HeadTailPop(&iConHTInB, iConHTInE, aConBufIn, sizeof(aConBufIn), &c1))
    {
      ((char *)pBuf)[cb1++] = c1;
    }

#ifdef USE_CRITICAL_SECTION
    LeaveCriticalSection(pConCS);
#else // USE_CRITICAL_SECTION
    ReleaseMutex(hConMutex);
#endif // USE_CRITICAL_SECTION
    return cb1;
  }

  if(ReadFile(iFile, pBuf, cbBuf, &cb1, NULL))
  {
    return cb1;
  }

  if(GetLastError() == ERROR_IO_PENDING)
  {
    return 0; // nothing read, but it's restartable
  }

  // TODO:  error?

  return 0;  // for now just return 0 even if it was an error
}

void set_rts_dtr(HANDLE iFile, int bSet)
{
DCB sDCB;

  // NOTE:  this function _DISABLES_ normal flow control

  memset(&sDCB, 0, sizeof(sDCB));
  sDCB.DCBlength = sizeof(sDCB);
  if(!GetCommState(iFile, &sDCB))
  {
    fprintf(stderr, "* ERROR * - unable to get comm state, %d (%x) (b)\n",
            GetLastError(), GetLastError());
    return;
  }

  sDCB.fDtrControl = DTR_CONTROL_DISABLE; // disabling RTS and DTR
  sDCB.fRtsControl = RTS_CONTROL_DISABLE; // necessary so I can assign them

  if(!SetCommState(iFile, &sDCB))
  {
    fprintf(stderr, "* ERROR * - unable to set comm state, %d (%x) (b)\n",
            GetLastError(), GetLastError());
  }

  if(bSet)
  {
    EscapeCommFunction(iFile, SETDTR);
    EscapeCommFunction(iFile, SETRTS);
  }
  else
  {
    EscapeCommFunction(iFile, CLRDTR);
    EscapeCommFunction(iFile, CLRRTS);
  }
}

void reset_arduino(HANDLE iFile)
{
DCB sDCB;
DWORD fDtr, fRts;

  memset(&sDCB, 0, sizeof(sDCB));
  sDCB.DCBlength = sizeof(sDCB);
  if(!GetCommState(iFile, &sDCB))
  {
    fprintf(stderr, "* ERROR * - unable to get comm state, %d (%x) (b)\n",
            GetLastError(), GetLastError());
    return;
  }

  fDtr = sDCB.fDtrControl; // preserve existing values
  fRts = sDCB.fRtsControl;

  sDCB.fDtrControl = DTR_CONTROL_DISABLE; // disabling RTS and DTR
  sDCB.fRtsControl = RTS_CONTROL_DISABLE; // necessary to prevent reset during comms

  if(!SetCommState(iFile, &sDCB))
  {
    fprintf(stderr, "* ERROR * - unable to set comm state, %d (%x) (b)\n",
            GetLastError(), GetLastError());
  }

//  // FIRST, make sure the lines are SET (high, low, then high again)
//
//  EscapeCommFunction(iFile, SETDTR);
//  EscapeCommFunction(iFile, SETRTS); // reset these - the transition should toggle reset
//
//  MySleep(0);  // delay again

  // bring the lines low at roughly the same time to discharge the capacitor

  EscapeCommFunction(iFile, CLRDTR);
  EscapeCommFunction(iFile, CLRRTS);

  // wait a bit
  MySleep(250);  // forces a 250msec delay

  EscapeCommFunction(iFile, SETDTR);
  EscapeCommFunction(iFile, SETRTS); // reset these - the transition should toggle reset

  MySleep(50);  // delay again

//  sDCB.fDtrControl = fDtr; // restore them to what they were (probably high)
//  sDCB.fRtsControl = fRts;
//
//  if(!SetCommState(iFile, &sDCB))
//  {
//    fprintf(stderr, "* ERROR * - unable to set comm state, %d (%x) (c)\n",
//            GetLastError(), GetLastError());
//  }

  MyGetsEchoOff();
  my_flush(iFile); // avrdude does this too - flush any extraneous input

  MySleep(500);  // 1/2 second wait following reset (for now)
}

HANDLE conconfig(HANDLE iFile)
{
  hConFile = iFile;

  // disable echo but allow line processing (for now)
  SetConsoleMode(iFile, ENABLE_PROCESSED_INPUT); // no echo, character input, ctrl+c is processed

  // start up a worker thread for the I/O along with a sync object
  // that I can use for threaded access to the buffer

#ifdef USE_CRITICAL_SECTION
  InitializeCriticalSection(&csCon);
  pConCS = &csCon;
#else  // USE_CRITICAL_SECTION
  hConMutex = CreateMutex(NULL, FALSE, NULL);

  if(hConMutex == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "* ERROR * - unable to create mutex (a), %d (%x) (a)\n",
            GetLastError(), GetLastError());
  }
#endif // CRITICAL_SECTION

  hConThread = (HANDLE)_beginthreadex(NULL, 65536, ConThreadEntry,
                                      NULL, CREATE_SUSPENDED,
                                      (DWORD *)&nConThreadID);

  if(hConThread == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "* ERROR * - unable to create thread (b), %d (%x) (a)\n",
            GetLastError(), GetLastError());
  }

  ResumeThread(hConThread);

  return iFile;
}

void ttyconfig(HANDLE iFile, int iBaud, int iParity, int iBits, int iStop)
{
COMMTIMEOUTS sCTM;
DCB sDCB;

  memset(&sDCB, 0, sizeof(sDCB));

  SetupComm(iFile, COM_BUFSIZE, COM_BUFSIZE);  // size for KERNEL buffers

  // From Docs:  A value of MAXDWORD, combined with zero values for both the
  // ReadTotalTimeoutConstant and ReadTotalTimeoutMultiplier members, specifies
  // that the read operation is to return immediately with the bytes that have
  // already been received, even if no bytes have been received.
  sCTM.ReadIntervalTimeout = MAXDWORD;
  sCTM.ReadTotalTimeoutMultiplier = 0;
  sCTM.ReadTotalTimeoutConstant = 0;
  sCTM.WriteTotalTimeoutMultiplier = 0;
  sCTM.WriteTotalTimeoutConstant = 0;

  SetCommTimeouts(iFile, &sCTM); // immediate return with received data or 0

  sDCB.DCBlength = sizeof(sDCB);
  if(!GetCommState(iFile, &sDCB))
  {
    fprintf(stderr, "* ERROR * - unable to get comm state, %d (%x) (a)\n",
            GetLastError(), GetLastError());
    return;
  }

  switch(iBaud)
  {
    // TODO:  add other baud rates and constants
    case 256000:
      sDCB.BaudRate = CBR_256000;
      break;
    case 128000:
      sDCB.BaudRate = CBR_128000;
      break;
    case 115200:
      sDCB.BaudRate = CBR_115200;
      break;
    case 57600:
      sDCB.BaudRate = CBR_57600;
      break;
    case 56000:
      sDCB.BaudRate = CBR_56000;
      break;
    case 38400:
      sDCB.BaudRate = CBR_38400;
      break;
    case 19200:
      sDCB.BaudRate = CBR_19200;
      break;
    case 14400:
      sDCB.BaudRate = CBR_14400;
      break;
    case 4800:
      sDCB.BaudRate = CBR_4800;
      break;
    case 2400:
      sDCB.BaudRate = CBR_2400;
      break;
    case 1200:
      sDCB.BaudRate = CBR_1200;
      break;

    default:
      fprintf(stderr, "* WARNING * - baud rate %d not supported, using 9600\n", iBaud);
      // flow through to '9600'
    case 9600:
      sDCB.BaudRate = CBR_9600;
      break;
  }

  sDCB.ByteSize = iBits;             // data size, xmit, and rcv
  sDCB.Parity = iParity > 0 ? ODDPARITY : iParity < 0 ? EVENPARITY : NOPARITY;
  sDCB.StopBits = iStop == 2 ? TWOSTOPBITS : ONESTOPBIT; // no 1.5

  if(!SetCommState(iFile, &sDCB))
  {
    fprintf(stderr, "* ERROR * - unable to set comm state, %d (%x) (a)\n",
            GetLastError(), GetLastError());
  }


  // start up a worker thread for the I/O along with a sync object
  // that I can use for threaded access to the buffer
  hTtyFile = iFile;

#ifdef USE_CRITICAL_SECTION
  InitializeCriticalSection(&csTty);
  pTtyCS = &csTty;
#else  // USE_CRITICAL_SECTION
  hTtyMutex = CreateMutex(NULL, FALSE, NULL);

  if(hTtyMutex == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "* ERROR * - unable to create mutex (c), %d (%x) (a)\n",
            GetLastError(), GetLastError());
  }
#endif // USE_CRITICAL_SECTION

  hTtyThread = (HANDLE)_beginthreadex(NULL, 65536, TtyThreadEntry,
                                      NULL, CREATE_SUSPENDED,
                                      (DWORD *)&nTtyThreadID);

  if(hTtyThread == INVALID_HANDLE_VALUE)
  {
    fprintf(stderr, "* ERROR * - unable to create thread (d), %d (%x) (a)\n",
            GetLastError(), GetLastError());
  }

  ResumeThread(hTtyThread);
}

// windows requires worker threads for console and serial I/O
// or else it won't work properly.  Serial I/O will be overlapped
// so I can do simultaneous read/write, and simulate what 'poll()'
// does for POSIX-compatible OSs (like Linux, BSD).  Something so
// incredibly simple shouldn't be this flaming difficult.

// TODO:  re-factor this so it performs better, no more 'goto's

UINT APIENTRY ConThreadEntry(void *pParam)
{
DWORD cb1 = 0;
//OVERLAPPED ovlIn;
//
//  memset(&ovlIn, 0, sizeof(ovlIn));

  iConHTInB=iConHTInE=0;

  while(nConThreadID) // set to zero to kill the thread
  {
#ifdef USE_CRITICAL_SECTION
    EnterCriticalSection(pConCS);
#else  // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hConMutex, INFINITE) != WAIT_OBJECT_0)
    {
      break; // will exit thread
    }
#endif // USE_CRITICAL_SECTION

short_cycle_read:
    if(!HeadTailDataOverflow(iConHTInB, iConHTInE, sizeof(aConBufIn)))
    {
      char c1;
      DWORD cb1 = 0;

#ifdef USE_CRITICAL_SECTION
      LeaveCriticalSection(pConCS);
#else  // USE_CRITICAL_SECTION
      ReleaseMutex(hConMutex);
#endif // USE_CRITICAL_SECTION

      if(ReadFile(hConFile, &c1, 1, &cb1, NULL) && cb1)
      {
#ifdef USE_CRITICAL_SECTION
        EnterCriticalSection(pConCS);
#else  // USE_CRITICAL_SECTION
        if(WaitForSingleObject(hConMutex, INFINITE) != WAIT_OBJECT_0)
        {
          break; // will exit thread
        }
#endif // USE_CRITICAL_SECTION

        HeadTailPush(c1, &iConHTInB, &iConHTInE,
                     aConBufIn, sizeof(aConBufIn));

        goto short_cycle_read;
      }
      else
      {
        Sleep(2); // make sure I yield
      }
    }
    else
    {
#ifdef USE_CRITICAL_SECTION
      LeaveCriticalSection(pConCS);
#else  // USE_CRITICAL_SECTION
      ReleaseMutex(hConMutex);
#endif // USE_CRITICAL_SECTION
      Sleep(2);  // make sure I yield
    }
  }

  nConThreadID = 0;  // make sure
  hConThread = INVALID_HANDLE_VALUE; // indicates I'm ending

  {
#ifdef USE_CRITICAL_SECTION
    LPCRITICAL_SECTION pC = pConCS;
    pConCS = NULL;
    EnterCriticalSection(pC);
    LeaveCriticalSection(pC);
    Sleep(100);
    DeleteCriticalSection(pC);
#else  // USE_CRITICAL_SECTION
    HANDLE hM0 = hConMutex;
    hConMutex = INVALID_HANDLE_VALUE;
    WaitForSingleObject(hM0, INFINITE); // acquire before destroy
    CloseHandle(hM0);
#endif // USE_CRITICAL_SECTION
  }

  WriteFile(GetStdHandle(STD_ERROR_HANDLE), "* THREAD EXIT *\r\n", 17, &cb1, NULL);
  _endthreadex(0);
  return 0;
}

UINT APIENTRY TtyThreadEntry(void *pParam)
{
OVERLAPPED ovlIn, ovlOut, ovlWait;
//char c1, cIn;
DWORD cb1 = 0, cb2, cb3, dwEvt;
int iOverlapRead = 0, iOverlapWrite = 0, iOverlapWait = 0; // state flags
int iLoopDataFlag, nChars;
char rbuf[256], wbuf[256]; // 256 chars at a time if I can do it


  iSerHTInB=iSerHTInE=iSerHTOutB=iSerHTOutE=0;

  // 'overlapped' structures - zero them out first
  memset(&ovlIn, 0, sizeof(ovlIn));
  memset(&ovlOut, 0, sizeof(ovlOut));
  memset(&ovlWait, 0, sizeof(ovlWait));

  // after zeroing, create 2 events (one for each).
  ovlIn.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  ovlOut.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  ovlWait.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

  GetCommMask(hTtyFile, &dwEvt);
//    SetCommMask(hTtyFile, dwEvt | EV_RXFLAG); // enable wait for these
  SetCommMask(hTtyFile, COMM_RW_EV); // enable wait for RX and TX

  iLoopDataFlag = 0; // to indicate no I/O this 'loop'

  while(nTtyThreadID) // set to zero to kill the thread
  {
    dwEvt = 0;
    if(iOverlapWait || !iLoopDataFlag)
    {
      iLoopDataFlag = 0; // re-init to zero at this time

      if(!iOverlapWait && WaitCommEvent(hTtyFile, &dwEvt, &ovlWait))
      {
        DWORD dwTemp = GetLastError();

        if(dwTemp == ERROR_IO_PENDING)
        {
          iOverlapWait = 1;
        }
      }
      else
      {
        iOverlapWait = 0;
      }

      // ok I'm either waiting for an event, or I have the event
      // if I'm still waiting for an event, I want to see if I have data
      // to send.  If I do, send it.

      if(iOverlapWait)
      {
#ifdef USE_CRITICAL_SECTION
        EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
        if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
        {
          break; // will exit thread
        }
#endif  // USE_CRITICAL_SECTION

        // waiting for event into, see if I need to send.  this will loop back
        // if I don't need to send data
        if(HeadTailDataAvailable(iSerHTOutB, iSerHTOutE))
        {
          goto send_data; // short-cycle the receive part
        }

#ifdef USE_CRITICAL_SECTION
        LeaveCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
        ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

        // re-own mutex, bail after xx msecs
        // The 'WaitCommEvent' is _SUPPOSED_ to be canceled
        // whenever you change the 'wait' mask via 'SetCommMask()'

        if(WaitForSingleObject(ovlWait.hEvent, 5) == WAIT_OBJECT_0)
        {
          cb3 = 0;
          GetOverlappedResult(hTtyFile, &ovlWait, &cb3, 1);
          iOverlapWait = 0; // no longer waiting
        }
//        else
//        {
//          // assume timed out
//          Sleep(10);
//          if(!iOverlapRead && !iOverlapWrite)
//          {
//            continue; // short-cycle the loop [nothing to do]
//          }
//        }
      }
    }
    else
    {
      iLoopDataFlag = 0; // re-init to zero at this time
    }

    // here we assume that we have data to read.  Since the
    // 'ReadFile()' can return zero bytes, I can get away with this.

#ifdef USE_CRITICAL_SECTION
    EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
    if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
    {
      break; // will exit thread
    }
#endif  // USE_CRITICAL_SECTION

//short_cycle_read:
    if(!HeadTailDataOverflow(iSerHTInB, iSerHTInE, sizeof(aSerBufIn)))
    {
#ifdef USE_CRITICAL_SECTION
      LeaveCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
      ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

      if(iOverlapRead) // overlapped I/O in progress
      {
        if(WaitForSingleObject(ovlIn.hEvent, 0) == WAIT_OBJECT_0)
        {
          cb1 = 0;
          if(GetOverlappedResult(hTtyFile, &ovlIn, &cb1, FALSE))
          {
            iOverlapRead = 0; // I am done waiting for overlap I/O

#ifdef USE_CRITICAL_SECTION
            EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
            if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
            {
              break; // will exit thread
            }
#endif // USE_CRITICAL_SECTION

            if(cb1 > 0)
            {
              for(nChars=0; nChars < (int)cb1; nChars++)
              {
                HeadTailPush(rbuf[nChars], &iSerHTInB, &iSerHTInE,
                              aSerBufIn, sizeof(aSerBufIn));
              }
//              HeadTailPush(cIn, &iSerHTInB, &iSerHTInE,
//                           aSerBufIn, sizeof(aSerBufIn));
              iLoopDataFlag = 1;
//              goto short_cycle_read;
            }
          }
          else
          {
            if(GetLastError() != ERROR_IO_INCOMPLETE)
            {
              iOverlapRead = 0; // cancel waiting for it
            }

            // NOTE:  if I wait for the event again, will it deadlock?

#ifdef USE_CRITICAL_SECTION
            EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
            // re-own the mutex for the next section
            if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
            {
              break; // will exit thread
            }
#endif // USE_CRITICAL_SECTION
          }
        }
      }
      else
      {
//do_the_read_operation:
        // I must not overlap-read more than I have room for in the head/tail buffer
        cb3 = HeadTailBufferLeft(iSerHTInB, iSerHTInE, sizeof(aSerBufIn));
        if(cb3 > sizeof(rbuf)) // probably
        {
          cb3 = sizeof(rbuf);
        }

        // read up to sizeof(rbuf) bytes.  Since timeout is set to return
        // immediately, no problem if I read FEWER CHARACTERS
        if(cb3)// && (dwEvt & EV_RXCHAR))
        {
          cb1 = 0;
          if(ReadFile(hTtyFile, rbuf, cb3, &cb1, &ovlIn))
          {
#ifdef USE_CRITICAL_SECTION
            EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
            if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
            {
              break; // will exit thread
            }
#endif // USE_CRITICAL_SECTION

            if(cb1)
            {
              for(nChars=0; nChars < (int)cb1; nChars++)
              {
                HeadTailPush(rbuf[nChars], &iSerHTInB, &iSerHTInE,
                             aSerBufIn, sizeof(aSerBufIn));
              }

              iLoopDataFlag = 1;
//              goto short_cycle_read;
            }
//            else if(!(dwEvt & EV_RXCHAR))
//            {
//              Sleep(2); // no data was available, so yield just a little
//            }
          }
          else
          {
            if(GetLastError() == ERROR_IO_PENDING)
            {
              iOverlapRead = 1;
            }

#ifdef USE_CRITICAL_SECTION
            EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
            // re-acquire mutex
            if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
            {
              break; // will exit thread
            }
#endif // USE_CRITICAL_SECTION
          }
        }
      }
    }

    // that part above is too complex and makes my brain hurt
    // thanks MS windows.  you suck at serial port I/O
    // ok the 'hTtyMutex' should be 'pwned' at this point

    if(HeadTailDataAvailable(iSerHTOutB, iSerHTOutE))
    {
send_data: // must own crit section or mutex here

      if(iOverlapWrite) // waiting for write
      {
#ifdef USE_CRITICAL_SECTION
        LeaveCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
        ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

        // wait up to 5 msecs for the write operation to complete.
        if(WaitForSingleObject(ovlOut.hEvent, 0) != WAIT_OBJECT_0)
        {
          goto end_of_the_loop; // not finished writing
        }

        cb1 = 0;
        if(!GetOverlappedResult(hTtyFile, &ovlOut, &cb1, FALSE))
        {
          if(GetLastError() == ERROR_IO_INCOMPLETE)
          {
            // I/O incomplete for 'overlapped result' - unlikely

            iOverlapWrite = 0; // this will avoid dead-locking the event
            goto end_of_the_loop; // hopefully all is well if I do this
          }
        }

        iOverlapWrite = 0; // Assume I am done waiting for overlap I/O

#ifdef USE_CRITICAL_SECTION
        EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
        if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
        {
          break; // will exit thread
        }
#endif // USE_CRITICAL_SECTION
      }

      // next, get up to sizeof(wbuf) characters to write
      nChars=0;
      do
      {
        HeadTailPop(&iSerHTOutB, iSerHTOutE, aSerBufOut, sizeof(aSerBufOut),
                    &(wbuf[nChars++]));
      } while(nChars < sizeof(wbuf) &&
              HeadTailDataAvailable(iSerHTOutB, iSerHTOutE));

#ifdef USE_CRITICAL_SECTION
      LeaveCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
      ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

      cb2 = 0;
      if(!WriteFile(hTtyFile, wbuf, nChars, &cb2, &ovlOut))
      {
        if(GetLastError() == ERROR_IO_PENDING)
        {
          iOverlapWrite = 1;
        }
        else
        {
          // TODO:  report error???
          Sleep(2); // spin but not too quickly (for now)
        }
      }
      else if(cb2)
      {
        iLoopDataFlag = 1;
#ifdef USE_CRITICAL_SECTION
        EnterCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
        if(WaitForSingleObject(hTtyMutex, INFINITE) != WAIT_OBJECT_0)
        {
          break; // will exit thread
        }
#endif // USE_CRITICAL_SECTION
      }
      else
      {
        // put unwritten characters back into the buffer?
      }
    }
    else
    {
#ifdef USE_CRITICAL_SECTION
      LeaveCriticalSection(pTtyCS);
#else  // USE_CRITICAL_SECTION
      ReleaseMutex(hTtyMutex);
#endif // USE_CRITICAL_SECTION

      // with no data in the outgoing queue, no need to wait
      // for the send buffer to empty as an event.
      SetCommMask(hTtyFile, COMM_RO_EV); // enable wait for RX but not TX
    }

end_of_the_loop:
    if(!iLoopDataFlag)
    {
      if(!iOverlapRead && !iOverlapWrite)
      {
        Sleep(10); // always (since no data going in or out nor overlapping)
      }
      else
      {
        Sleep(2);
      }
    }
  }

  if(iOverlapWait)
  {
//    CancelIoEx(hTtyFile, &ovlWait); // cancel the 'wait for comm event'
    CancelIo(hTtyFile); // XP compatibility - it doesn't have CancelIoEx
  }

  nTtyThreadID = 0;  // make sure
  hTtyThread = INVALID_HANDLE_VALUE; // indicates I'm ending

  {
#ifdef USE_CRITICAL_SECTION
    LPCRITICAL_SECTION pC = pTtyCS;
    pTtyCS = NULL;
    EnterCriticalSection(pC);
    LeaveCriticalSection(pC);
    Sleep(100);
    DeleteCriticalSection(pC);
#else  // USE_CRITICAL_SECTION
    HANDLE hM0 = hTtyMutex;
    hTtyMutex = INVALID_HANDLE_VALUE;
    WaitForSingleObject(hM0, INFINITE); // acquire before destroy
    CloseHandle(hM0);
#endif  // USE_CRITICAL_SECTION
  }

  if(iOverlapRead)
  {
//    CancelIoEx(hTtyFile, &ovlIn); // cancel write
    CancelIo(hTtyFile); // XP compatibility - it doesn't have CancelIoEx
  }
  if(iOverlapWrite)
  {
//    CancelIoEx(hTtyFile, &ovlOut); // cancel write
    CancelIo(hTtyFile); // XP compatibility - it doesn't have CancelIoEx
  }

  CloseHandle(ovlIn.hEvent);
  CloseHandle(ovlOut.hEvent);
  CloseHandle(ovlWait.hEvent);

  WriteFile(GetStdHandle(STD_ERROR_HANDLE), "* THREAD EXIT *\r\n", 17, &cb1, NULL);
  _endthreadex(0);
  return 0;
}


#endif // WIN32

