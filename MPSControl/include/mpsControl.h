#ifndef _MPSCONTROL_H_
#define _MPSCONTROL_H_

#include <queue>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/bcm.h>

#include "../include/dwSerial.h"

#define MSGID   (0x0BC)

#define NFRAMES (1)

#define DELAY (1000)
#define DELAY1SEC (1000000)

#define DEADBEEF 0xdeadbeef

#define PRIORITY1 0x08000000U
#define PRIORITY3 0x0c000000U


class MPSControl
{

public:

    MPSControl() {}
    ~MPSControl() {}

    void start();
    void stop();
    void run(void *ptr);
    void kill();

    // CAN Sockets
    int initCAN(char * interface);
    int openCAN();
    int closePGN();
    void send_brake_command();
    void send_can_heartbeat();
    // TCP sockets
    int openTCPServer(int port);
    int sendTCPData( int sockfd, unsigned char * buff, int length);
    int sendPGN(unsigned int PGN, ssize_t priority, unsigned char *data, int length);
    int receivePGN(unsigned char PGN[], unsigned char *data);
    int startThreads();
    void setMaxQSize(int parm);
    bool killthreads=false;
    int socktcp;
    int clientsocket;
    void setListenerUSB(char * usb);
    char *usbConnection;
    typedef struct msg_header
    {
        unsigned char length;
        unsigned char PGN[2];
    } __attribute__((packed)) msg_header_t ;


    dwSerial serial;


    /*
    std::queue<can_q_item *> sendCANQ;
    std::queue<can_q_item *> rcvCANQ;
    std::queue<tcp_q_item *> sendTCPQ;
    std::queue<tcp_q_item *> receiveTCPQ;

    bool addToQueue(std::queue<q_item *> *workq, q_item *item,  pthread_mutex_t qlock, bool &available);
    q_item * removeFromQueue(std::queue<q_item *> *workq,  pthread_mutex_t qlock, bool &available);
    */

private:
    /*    pthread_mutex_t tcp_qlock;
        size_t max_q_size=5;
        size_t max=10;
        pthread_mutex_t tcp_send_lock;

        struct q_item
        {

        } ;

        struct can_q_item : q_item
        {
            char *fwd;
        } ;

        struct tcp_q_item : q_item
        {
            char *fwd;
        } ;


        bool can_send_available;
        bool can_rcv_available;
        bool tcp_send_available;
        bool tcp_rcv_available;
    */

    int flags;
    int sockcan;
    char *iface;
    struct sockaddr_can addr;
    struct ifreq ifr;

    struct can_msg
    {
        struct bcm_msg_head msg_head;
        struct can_frame frame[NFRAMES];

    } msg;


    int recv_frame(struct can_frame *frame);
    int send_frame(struct can_frame *frame);
    //struct can_frame frame;
    int processTCPStream(int datalength, unsigned char data[]);
    int getTCPData( int sockfd, unsigned char *buff, int size);
    int checkTCPMessage(int datalength, unsigned char data[]);
    void send_ping_response(int clientsocket, unsigned char *data, int length);
    bool sendCANmsg(struct msg_header hdr, unsigned char data[], int length);

    pthread_t threadCANread, threadCANsend;
    pthread_t threadTCP;


    static void context_run(void *context)
    {
        return((MPSControl *)context)->run(nullptr);
    }


};

#endif // _MPSCONTROL_H_
