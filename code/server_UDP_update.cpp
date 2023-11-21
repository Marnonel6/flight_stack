// Server side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
//#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"
	
#define PORT	 8080
#define MAXLINE 100

	
// Driver code
int main() {
	int sockfd;
	char buffer[MAXLINE];
//	char *hello = "Hello from server";
  char sendBuff[10];
	struct sockaddr_in servaddr, cliaddr;
		
	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
		
	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));
		
	// Filling server information
	servaddr.sin_family = AF_INET; // IPv4
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(PORT);
		
	// Bind the socket with the server address
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,
			sizeof(servaddr)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	
	int len, n;
	
	len = sizeof(cliaddr); //len is value/result
 
 
  //joystick code
  int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];
	struct js_event js;

	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		printf( "Couldn't open joystick\n" );
		return -1;
	}

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	button = (char *) calloc( num_of_buttons, sizeof( char ) );

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
		, name_of_joystick
		, num_of_axis
		, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */
        read(joy_fd, &js, sizeof(struct js_event));
      	 printf("X: %6d  Y: %6d  X1: %6d Y1: %6d b1%d b2%d\n\r", axis[0], axis[1],axis[3],axis[4],button[0],button[1]);

	//end joystick code
	
  int heartbeat=0;
  while(1)
  {
   

  
	n = recvfrom(sockfd, (char *)buffer, MAXLINE,	MSG_WAITALL, ( struct sockaddr *) &cliaddr,	(socklen_t*)&len);
	buffer[n] = '\0';
 
 
  while (read(joy_fd, &js, sizeof(struct js_event))>0)
  {

	switch(js.type &~JS_EVENT_INIT)
	{
        case JS_EVENT_AXIS:
            axis[js.number]=js.value;
        break;
        case JS_EVENT_BUTTON:
            button [js.number]=js.value;
        break;
        default :
        break;
  }
 }

    int a=-axis[1]/290+128;	
    int b=axis[0]/290+128;
    int c=axis[3]/290+128;	
    int d=-axis[4]/290+128;
    int e=(int)button[0]+1;
    int f=(int)button[1]+1;	
    int g=(int)button[2]+1;	
    int h=(int)button[3]+1;	
    snprintf(sendBuff, sizeof(sendBuff), "%c%c%c%c%c%c%c%c%c\r\n ", a,b,c,d,e,f,g,h,heartbeat++);

	sendto(sockfd, (const char *)sendBuff, strlen(sendBuff),MSG_CONFIRM, (const struct sockaddr *) &cliaddr,len);
   }
	return 0;
}
