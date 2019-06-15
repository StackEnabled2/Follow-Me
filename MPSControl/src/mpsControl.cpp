/*
The MIT License (MIT)

Copyright (c) 2015 Jacob McGladdery

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

-------------------------------------------------------------------------------

Broadcast Manager Interface Demo

This program demonstrates reading and writing to a CAN bus using SocketCAN's
Broadcast Manager interface. The intended behavior of this program is to read
in CAN messages which have an ID of 0x123, add one to the value of each data
byte in the received message, and then write that message back out on to the
bus with the message ID defined by the macro MSGID.
*/

#include "../include/util.h"
#include "../include/mpsControl.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ctime>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <linux/can/bcm.h>
#include <queue>

#include <pthread.h>

#define PROGNAME "socketCAN"
#define VERSION  "1.0.0"

using namespace std;

/*
// q handling
bool MPSControl::addToQueue(std::queue<q_item *> *workq, q_item *item, pthread_mutex_t qlock, bool &available)
{
    if (workq->size() >= max_q_size)
    {
        cout << "couldn't add to occ q" << endl;
        while (true)
        {
            q_item * item = removeFromQueue(workq,qlock, available);
            if (nullptr == item)
                break;
        }
        return false;

    }
    pthread_mutex_lock(&qlock);
    workq->push(item);
    if (workq->size() > max)
    {
        max = workq->size();
    }
    available = true;
    pthread_mutex_unlock(&qlock);
    return true;
}

MPSControl::q_item * MPSControl::removeFromQueue(std::queue<q_item *> *workq,  pthread_mutex_t qlock, bool &available)
{
    pthread_mutex_lock(&qlock);
    q_item * item = nullptr;
    if (!workq->empty())
    {
        available = true;
        item = workq->front();
        workq->pop();
    }
    else
    {
        available = false;
    }
    pthread_mutex_unlock(&qlock);
    return item;
}

void MPSControl::setMaxQSize(int parm)
{
    max_q_size = parm;
}
*/

/************************************************************
 *
 *
 *
 ************************************************************
 */

void error( char *msg )
{
    perror(  msg );
    exit(1);
}

int func( int a )
{
    return 2 * a;
}


/************************************************************
 *
 * function:
 *
 ************************************************************
 */
void MPSControl::start()
{
}

void unblockTCPListener()
{
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    portno = 8080;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname("127.0.0.1");
    if (server == NULL)
    {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");
    printf("Should have unblocked the TCP listen\n");
}

/************************************************************
 *
 * function:
 *
 ************************************************************
 */
void MPSControl::stop()
{
    printf("got a stop\n");
    this->killthreads = true;
    if (this->socktcp > -1)
    {
        close(this->socktcp);
        unblockTCPListener();
    }
    if (this->clientsocket > 0)
    {

        close(this->clientsocket);
        //this->clientsocket = -1;
    }

}


/************************************************************
 *
 * function: init(const char * interface)
 *
 ************************************************************
 */

int MPSControl::initCAN(char * interface)
{

    int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;

    this->iface = interface;    /* Open the CAN interface */
    this->sockcan = socket(family, type, proto);
    if (this->sockcan < 0)
    {

        return errno;
    }

    strncpy(this->ifr.ifr_name, this->iface, IFNAMSIZ);

    return 0;
}

/************************************************************
 *
 * function:
 *
 ************************************************************
 */

int MPSControl::openCAN()
{
    strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);

    this->ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    this->ifr.ifr_ifindex = if_nametoindex(this->ifr.ifr_name);

    if (!this->ifr.ifr_ifindex)
    {
        perror("if_nametoindex");
        return 1;
    }


    this->addr.can_family = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;

    if ((this->sockcan = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        return 1;
    }

    //setsockopt(this->sockcan, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);


    if (bind(this->sockcan, (struct sockaddr *)&this->addr, sizeof(this->addr)) < 0)
    {
        perror("bind");
        close(this->sockcan);
        return 1;
    }

    /* Set socket to non-blocking */
    this->flags = fcntl(this->sockcan, F_GETFL, 0);
    if (this->flags < 0)
    {
        perror(PROGNAME ": fcntl: F_GETFL");
        return errno;
    }

    if (fcntl(this->sockcan, F_SETFL, this->flags | O_NONBLOCK) < 0)
    {
        perror(PROGNAME ": fcntl: F_SETFL");
        return errno;
    }

    return 0;
}
int MPSControl::sendPGN(unsigned int PGN, ssize_t priority, unsigned char *data, int length)
{
    ssize_t required_mtu=16;
    struct canfd_frame frame;

    frame.can_id = PGN;
    frame.can_id = frame.can_id << 8;
    frame.can_id |= 0x05; // source
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_id |= priority;


    frame.len = length ;
    for (int i=0; i < length; i++)
    {
        frame.data[i] = data[i];
    }

    /* send frame */
    ssize_t received_mtu = write(this->sockcan, &frame, required_mtu);

    if (received_mtu != required_mtu)
    {
        perror("write");
        return 1;
    }
    return 0;
}

/************************************************************
*
* function:
*
************************************************************
*/

int MPSControl::receivePGN(unsigned char *pgn, unsigned char *data)
{
    struct can_frame frame;

    while (true && !this->killthreads)
    {
        bool ok=false;
        int size = recv_frame(&frame);
        int i=0;
        if (size < 0)
        {
            pthread_yield();
            usleep(DELAY);
            continue;
        }

        unsigned char pg2 =  frame.can_id >> 8;
        unsigned char pg3 =  frame.can_id >> 16;

        for (unsigned int i=0; pgn[i] != 0x00 && i < sizeof(pgn); i+=2)
        {
            if (pg3 == pgn[i] && pg2 == pgn[i+1])
            {
                ok=true;
                break;
            }

        }
        if (ok == false)
        {
            pthread_yield();
            continue;
        }
        *(data + i++) = pg3;
        *(data + i++) = pg2;

        //printf("%04x: ", frame.can_id);   // filter out PGN
        if (frame.can_id & CAN_RTR_FLAG)
        {
            printf("remote request");
        }
        else if (frame.can_dlc > 0)
        {
            //printf("[%d]", frame.can_dlc);
            for (int j=0;  j < frame.can_dlc; j++)
            {
                *(data + i++) = frame.data[j];
                //printf(" %02x", frame.data[j]);
            }
        }
        *(data + i++) = (unsigned char) 0xde;
        *(data + i++) = (unsigned char) 0xad;
        *(data + i++) = (unsigned char) 0xbe;
        *(data + i++) = (unsigned char) 0xef;
        //printf("\n");
        pthread_yield();
        return i;
    }
}
static int verbose;

int MPSControl::recv_frame(struct can_frame *frame)
{
    int ret;

    ret = recv(this->sockcan, frame, sizeof(*frame), 0);

    if (ret != sizeof(*frame))
    {
        if (ret < 0)
        {
            // perror("recv failed");
        }
        else
        {
            printf("recv returned %d", ret);
        }
        return ret;
    }
    return ret;
}

int MPSControl::send_frame(struct can_frame *frame)
{
    int ret;

    while ((ret = send(this->sockcan, frame, sizeof(*frame), 0))
            != sizeof(*frame))
    {
        if (ret < 0)
        {
            if (errno != ENOBUFS)
            {
                perror("send failed");
                return -1;
            }
            else
            {
                if (verbose)
                {
                    printf("N");
                    fflush(stdout);
                }
            }
        }
        else
        {
            fprintf(stderr, "send returned %d", ret);
            return -1;
        }
    }
    return 0;
}



/************************************************************
*
* function:
*
************************************************************
*/
int MPSControl::closePGN()
{
    close(this->sockcan);
    return 0;
}
// TCP sockets
/************************************************************
*
* function:
*
************************************************************
*/
void MPSControl::send_brake_command()
{
    unsigned char data[6];
    unsigned int pgn=0xFD;
    pgn = pgn << 8;
    pgn |= 0xE8;
    data[0]= 0;
    data[1] = 2;  // autonomous
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 1;  // e-stop

    sendPGN( pgn, PRIORITY1, data, 6);

    printf("lost connection - better get things stopped\n");
}
/************************************************************
*
* function:
*
************************************************************
*/
void MPSControl::send_can_heartbeat()
{
    unsigned char data[6];
    unsigned int pgn=0xFD;
    pgn = pgn << 8;
    pgn |= 0xE8;
    data[0]= 0x09;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;  // e-stop

    sendPGN( pgn, PRIORITY3, data, 6);

    printf("lost connection - better get things stopped\n");
}
// TCP sockets
/************************************************************
*
* function:
*
************************************************************
*/
#include <arpa/inet.h>
void printipaddr(int newfd)
{
    struct sockaddr_in addr;
    socklen_t  addr_size= sizeof(struct sockaddr_in);
    char * clientip = new char[20];

    getpeername(newfd, (struct sockaddr *) &addr, &addr_size);

    strcpy(clientip, inet_ntoa(addr.sin_addr));

    printf("connected: %s sock: %d\n", clientip, newfd);
    delete(clientip);
}

/************************************************************
*
* function:
*
************************************************************
*/
void MPSControl::send_ping_response(int clientsocket, unsigned char *data, int length)
{
    printf("send ping response to %d\n", clientsocket);

    sendTCPData(clientsocket,data,length );

    send_can_heartbeat();

}

/************************************************************
*
* function:
*
************************************************************
*/
int MPSControl::checkTCPMessage(int datalength, unsigned char data[])
{
    int ret = -1;
    int i = 0;

    struct msg_header hdr;

    int dbeefloc = 0;
    //---- wait for a number from client ---
    // minimum size
    if (datalength <( (unsigned int) sizeof(msg_header) + sizeof(DEADBEEF)))
    {
        ret = -1;
    }
    else
    {

        hdr.length = data[i++];
        hdr.PGN[0] = data[i++];
        hdr.PGN[1] = data[i++];
        if (hdr.PGN[0] == 0xff
                && hdr.PGN[1] == 0xff)
        {
            send_ping_response(this->clientsocket, data, datalength);
            return 0;
        }
        dbeefloc = hdr.length - sizeof(DEADBEEF)  ;
        int j = dbeefloc ;
        // check the end
        if (data[j++] != 0xde ||
                data[j++] != 0xad ||
                data[j++] != 0xbe ||
                data[j++] != 0xef)
        {
            ret = -1;
        }
        else
        {
            ret = j;
        }
    }
    return ret;
}

int MPSControl::processTCPStream(int datalength, unsigned char data[])
{
    int i=0;
    for (int i=0; i < datalength; i++)
    {
        printf("%02X ", data[i]);
    }
    printf("\n");
    while (datalength > 0)
    {

        int next = checkTCPMessage(datalength, &data[i]);
        if (next < 1)
        {
            return -1;
        }
        else
        {
            unsigned char * buf = &data[i];
            unsigned int pgn = *(buf+1);
            //printf("PGN: %04x \n", pgn);

            pgn = pgn << 8;
            pgn = pgn & 0xff00;
            pgn |= *(buf+2);

            if (0 != sendPGN( pgn, PRIORITY3, &data[i+3], *buf - sizeof(struct msg_header) - sizeof(DEADBEEF)))
            {
                send_brake_command();
                return -1;
            }
            i = next;
            datalength -= i;
        }
    }
    return 0;
}

/************************************************************
*
* function:
*
************************************************************
*/
int MPSControl::openTCPServer(int port)
{
    int ret = 0;
    int clilen;

    struct sockaddr_in serv_addr, cli_addr;

    printf( "using port #%d\n", port );
    while (true && !this->killthreads)
    {
        this->socktcp = -1;
        socktcp = socket(AF_INET, SOCK_STREAM, 0);
        if (socktcp < 0)
        {
            printf("Have to get a listening socket\n");
            usleep(DELAY1SEC);
        }
        else
        {
            break;
        }
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons( port );

    while (true && !this->killthreads)
    {
        if (bind(socktcp, (struct sockaddr *) &serv_addr,
                 sizeof(serv_addr)) < 0)
        {
            perror( const_cast<char *>( "ERROR on binding" ) );
            usleep(1000000);
        }
        else
        {
            break;
        }
    }
    listen(socktcp,5);
    clilen = sizeof(cli_addr);

    //--- infinite wait on a connection ---
    while (!this->killthreads )
    {
        printf( "waiting for new client...\n" );
        this->clientsocket = -1;
        if ( ( this->clientsocket = accept( socktcp, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
        {
            perror("ERROR on accept");
            pthread_yield();
            usleep(1000000);
            continue;
        }
        if (this->killthreads)
            return -1;
        printipaddr(this->clientsocket);

        while ( !this->killthreads)
        {
            unsigned char data[64];
            int datalength;

            memset(data, 0, sizeof(data));

            datalength = this->getTCPData( this->clientsocket, data, sizeof(data) - 1 );

            if (datalength < 1)
            {
                send_brake_command();
                break;
            }

            processTCPStream(datalength, data);

            pthread_yield();

        }
        printf("\n");

    }
    return ret;
}


/************************************************************
*
* function:
*
************************************************************
*/
int MPSControl::sendTCPData( int sockfd, unsigned char * buff, int length)
{
    int n;
    int flag = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

    n = write( sockfd, buff, length );

    for (int i=0; i < length; i++)
    {
        //printf(" %02x", *(buff+i));
    }
    //printf("\n");
    //flag = 0;
    //setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

    return n;

}

/************************************************************
*
* function:
*
************************************************************
*/
int MPSControl::getTCPData( int sockfd, unsigned char *buff, int size)
{
    int n;

    if ( (n = read(sockfd,buff,size) ) < 0 )
        perror( "ERROR reading from socket");

    if (n == 0)
        return -1;
    return n;
}


/************************************************************
*
* function:
*
************************************************************
*/
void MPSControl:: setListenerUSB(char * usb)
{
    this->usbConnection = usb;
}


/************************************************************
*
* function:
*
************************************************************
*/

static void *rcvCAN_thread(void *ptr)
{
    MPSControl *context = (MPSControl *) ptr;
    clock_t now = clock();
    clock_t last = clock();
    double timePassed;
    double mPassed;

    while (!context->killthreads)
    {
        ssize_t nbytes;
        unsigned char data[32];
        unsigned char pgn[5]= {0xff,0x10,0xff,0x11,0x00};

        memset(data, 0, sizeof(data));
        //for (unsigned int i=0; pgn[i] != 0x00 && i < sizeof(pgn); i+=2)
        //{
        nbytes = context->receivePGN(pgn, (data+1));
        //}
        if (nbytes < 1)
        {
            usleep(0x1000);
            continue;
        }

        now = clock();
        timePassed = now - last;
        mPassed = timePassed / ((double) CLOCKS_PER_SEC/1000);
        //if (mPassed < 3.0)
        //{
        //printf("timePassed: %f  mpassed: %f", timePassed, mPassed);
        //    usleep(DELAY);
        //    continue;
        //}

        last = clock();
        *(data) = (unsigned char) ++nbytes;
        if (context->clientsocket > 0)
        {
            int retry = 0;
            while (retry < 3)
            {
                int bytes = context->sendTCPData( context->clientsocket, data, nbytes);
                if (0 == bytes || nbytes != bytes)
                {
                    if (retry == 0)
                    {
                        context->send_brake_command();
                    }
                    retry++;
                    continue;
                }
                else
                {
                    retry = 0;
                    break;
                }

            }
            if (retry != 0)
            {
                close(context->clientsocket);
                context->clientsocket = -1;
            }
        }
        usleep(DELAY);
    }
    return nullptr;

}

/************************************************************
*
* function:
*
************************************************************
*/
static void *rcvTCP_thread(void *ptr)
{
    MPSControl *context = (MPSControl *) ptr;

    if (context->openTCPServer(8080) == -1)
        printf ("found the dead TCP server\n");

    return nullptr;
}


static void createPGNHeader(unsigned int pgn, unsigned char *data)
{
    pgn = pgn << 8;
    pgn |= 0xE8;
//    data[0]= (unsigned char ) pgn << 8;
//    data[1] = (unsigned char ) pgn;
    data[0]= 0xff;
    data[1] = 0xfe;

}

/************************************************************
*
* function:
*
************************************************************
*/

unsigned char message[64];

static void *dwRCV_thread(void *ptr)
{
    MPSControl *context = (MPSControl *) ptr;
    //ool toggle = false;
    // context->serial.Open("/dev/ttyACM0",115200);
    unsigned char line[64];

    context->serial.Open(context->usbConnection,115200);
    if(!context->serial.IsOpen())
        return nullptr;
    bool toggle = true;
    context->serial.SenddwCommand("\r\r");
    int tries = 0;
    int wait=0;
    while (!context->killthreads)
    {
        memset(line, 0, sizeof line);
        int len = context->serial.ReceiveLocation(line, sizeof line);
        printf ("received %d bytes\n",len);
        if ((line[7]== 'P'
                  && line[8]== 'O')
                 || ( line[0]== 'P'
                      && line[1]== 'O'))
        {
            break;
        }
        if (line[0] == '@')
        {
            context->serial.SenddwCommand("\r\r");
            usleep(DELAY1SEC);
            len = context->serial.ReceiveLocation(line, sizeof line);
            if (len > sizeof line) // it was flushed
            {
                toggle = true;
            }
        }
        else
        {
            string lec("dwm> lec\n");
            printf("compare\n");
            if (lec.compare((const char *)line) == 0)
            {
                wait = 0;
                printf("%s", line);
            }
            if (wait++ > 5) {
                context->serial.SenddwCommand("lec\r");
                wait = 0;
            }
            pthread_yield();
            usleep(DELAY1SEC/2);
            printf("came back after sleeep\n");

            //context->serial.SenddwCommand("abc\r");


        }
        pthread_yield();
    }
    int dotnumber = 0;
    while(true && !context->killthreads)
    {
        memset(line, 0, sizeof line);
        int rcvd = context->serial.ReceiveLocation(line, sizeof line);
        // if (rcvd != 0) printf("rcvd: %d\n", rcvd);
        if (rcvd > 10)
        {
            printf("l:%d  %s\n", rcvd,line);
            memset(message, 0, sizeof message);
            //printf(".");
            if (dotnumber++ > 80)
            {
                //printf("\n");
                dotnumber = 0;
            }
            message[0] = 0x01;
            createPGNHeader(0x50, &message[1]);
            int i=3;
            for (; i < sizeof message - 3; i++)
                //for (i=0; i < sizeof message; i++)
            {
                message[i] = line[i-3];
            }
            int size = strlen((const char *)message);
            if (size < sizeof message - 7) {
                message[size] = 0xde;
                message[size+1] = 0xad;
                message[size+2] = 0xbe;
                message[size+3] = 0xef;
            }
            message[0] = size + 4;
            if (context->clientsocket > 0)
            {
                //printf("%s\n", &message[2]);
                context->sendTCPData(context->clientsocket,message, size + 4);
                //printf("%s\n", message);
                //context->sendTCPData(context->clientsocket,message, i);

            }
        }
        pthread_yield();
        usleep(0x1000);
    }

    return nullptr;
}



int MPSControl::startThreads()
{

    int  iret1, iret2, iret3;
    pthread_t threadCANread;
    pthread_t threadTCP;
    pthread_t threadDW;

    /* Create independent threads each of which will execute function */

    iret1 = pthread_create( &threadCANread, NULL, rcvCAN_thread, this);
    iret2 = pthread_create( &threadTCP, NULL, rcvTCP_thread, this);
    iret3 = pthread_create(&threadDW, NULL, dwRCV_thread, this);


    pthread_join( threadCANread, NULL);
    pthread_join( threadTCP, NULL);
    pthread_join(threadDW, NULL);

    printf("Thread 1 returns: %d\n",iret1);
    printf("Thread 2 returns: %d\n",iret2);
    printf("Thread 3 returns: %d\n",iret3);

    return 0;

}


