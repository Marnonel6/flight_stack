// Client side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <sys/time.h>
	
#include <time.h>
#include <sys/shm.h>

#define PORT	 8080
#define MAXLINE 10

 struct data
{
    int keypress;
    int pitch;
    int roll;
    int yaw;
    int thrust;
    int sequence_num;
};

struct timespec te;
// Driver code
int main(int argc, char *argv[]) {
	int sockfd;
	char buffer[MAXLINE];
	char *hello = "1";
	struct sockaddr_in	 servaddr;
	
	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
	
	memset(&servaddr, 0, sizeof(servaddr));
		
	// Filling server information
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
 
     if(inet_pton(AF_INET, argv[1], &servaddr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return 1;
    }

  //shared memory init
    int segment_id;
    data* shared_memory;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey=33222;
    int ch = 0;
    
    /* Allocate a shared memory segment.  */
    segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
    /* Attach the shared memory segment.  */
    shared_memory = (data*) shmat (segment_id, 0, 0);
    printf ("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl (segment_id, IPC_STAT, &shmbuffer);
    segment_size  =               shmbuffer.shm_segsz;
    printf ("segment size: %d\n", segment_size);


    float roll=0;
    float pitch=0;
    float yaw=0;
    float thrust=1500;
    int key=0;
    shared_memory->keypress=0;

    int n, len;
    char recvBuff[10];
    long time1;
    long time2;
    while(1)
    {

        //get current time in nanoseconds
        timespec_get(&te,TIME_UTC);
        time2=te.tv_nsec;
        
        float time_diff;
        time_diff=time2-time1;
        //check for rollover
  if(time_diff<=0)
  {
    time_diff+=1000000000;
  }
  //convert to seconds
  time_diff=time_diff/1000000000;

  if(time_diff>0.03)
  {
        sendto(sockfd, (const char *)hello, strlen(hello),MSG_CONFIRM, (const struct sockaddr *) &servaddr,	sizeof(servaddr));
        n = recvfrom(sockfd, (char *)recvBuff, MAXLINE,	MSG_WAITALL, (struct sockaddr *) &servaddr,	(socklen_t*)&len);
        recvBuff[n] = '\0';


        time1=time2;
        //printf("time diff=%f\n\r",time_diff);
        
        printf("time update =%f Data rx size %d : %d %d %d %d %d %d %d %d %d  \n",time_diff, sizeof(recvBuff),recvBuff[0],recvBuff[1],recvBuff[2],recvBuff[3],recvBuff[4],recvBuff[5],recvBuff[6],recvBuff[7],recvBuff[8]);


        thrust=recvBuff[0];
        roll=recvBuff[2];
        pitch=recvBuff[3];
        yaw=recvBuff[1];
        if(recvBuff[4]>1)
        {
            shared_memory->keypress=32;
            //printf("key kill pressed\n\r");
        }
        else if(recvBuff[5]>1)
        {
            shared_memory->keypress=33;
        //	printf("key pause pressed\n\r");
        }
        else if(recvBuff[6]>1)
        {
            shared_memory->keypress=34;
        //	printf("key un- pause pressed\n\r");
        }
        else if(recvBuff[7]>1)
        {
            shared_memory->keypress=35;
        //	printf("calibration pressed\n\r");
        }
        else//no key press
        {
            shared_memory->keypress=36;
        }

        shared_memory->thrust=thrust;
        shared_memory->pitch=pitch;
        shared_memory->roll=roll;
        shared_memory->yaw=yaw;
    
        shared_memory->sequence_num=recvBuff[8];
        }
    }
    close(sockfd);
    shmdt (shared_memory);


    return 0;
}
