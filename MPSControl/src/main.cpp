#include "../include/util.h"
#include "../include/mpsControl.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/dwSerial.h"
#include <iostream>

#define PROGNAME "socketCAN"
#define VERSION  "1.0.0"
int  testSerial(MPSControl *context);

static sig_atomic_t sigval;
MPSControl *mps = nullptr;

static void onsig(int val)
{
    printf("on sig\n");

    if (nullptr != mps)
    {
        mps->stop();
    }
    sigval = (sig_atomic_t)val;
}

static void usage(void)
{
    puts("Usage: " PROGNAME "[OPTIONS] IFACE\n"
         "Where:\n"
         "  IFACE    CAN network interface\n"
         "Options:\n"
         "  -h       Display this help then exit\n"
         "  -v       Display version info then exit\n");
}

static void version(void)
{
    puts(PROGNAME " " VERSION "\n");
}

int startThread(MPSControl *ctl)
{
    return 0;
}

int main(int argc, char **argv)
{
    int opt;
    char *iface;

    mps = new MPSControl();
    //startThreads();
    /* Check if at least one argument was specified */
    if (argc < 2)
    {
        fputs("Too few arguments!\n", stderr);
        usage();
        return EXIT_FAILURE;
    }

    /* Parse command line options */
    while ((opt = getopt(argc, argv, "hv")) != -1)
    {
        switch (opt)
        {
        case 'h':
            usage();
            return EXIT_SUCCESS;
        case 'v':
            version();
            return EXIT_SUCCESS;
        default:
            usage();
            return EXIT_FAILURE;
        }
    }

    /* Exactly one command line argument must remain; the interface to use */
    if (optind == (argc - 1))
    {
        iface = argv[optind];
        mps->setListenerUSB(iface);
    }
    else
    {
        fputs("Only one interface may be used!\n", stderr);
        usage();
        return EXIT_FAILURE;
    }

    /* Register signal handlers */
    if (signal(SIGINT, onsig)    == SIG_ERR ||
            signal(SIGTERM, onsig)   == SIG_ERR ||
            signal(SIGCHLD, SIG_IGN) == SIG_ERR)
    {
        perror(PROGNAME);
        return errno;
    }

     //testSerial(mps);
    //if (true)
    //    return 0;

    /* Open the CAN interface */

    int error = mps->openCAN();
    if (error != 0)
    {
        printf("ERROR CAN OPEN: %d\n", error);
        perror("unable to Open the CAN interface");
        return error;
    }

    error = mps->startThreads();
    if (error != 0)
    {
        printf("ERROR THREADS: %d\n", error);
        perror("Start the threads error");
        return error;
    }
    mps->closePGN();

    puts("\nGoodbye!");


    return EXIT_SUCCESS;
}


int  testSerial(MPSControl *context)
{
    context->serial.Open(context->usbConnection,115200);
    if(!context->serial.IsOpen())
        return 1;
    unsigned char line[64];
    context->serial.SenddwCommand("\r\r");
    usleep(DELAY1SEC);
    context->serial.SenddwCommand("lec\r");
    usleep(DELAY1SEC);
    while (true)
    {
        memset(line, 0, sizeof(line));
        int len = context->serial.ReceiveLocation(line, sizeof line);
        if (len > 0)
        {
            printf("line - %s\n",  line);
        }
    }
}
