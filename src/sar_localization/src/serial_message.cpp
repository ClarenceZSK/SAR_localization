#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "sar_localization/Imu.h"
#include <sstream>
#include <ros/console.h>
//#include "/opt/eigen/Eigen"
// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;

struct timeval tv;		  ///< System time

// Settings
//int sysid = 42;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
//int compid = 110;
//int serial_compid = 0;

//uart reicer flag
#define b_uart_head  0x80 
#define b_rx_over    0x40  
// USART Receiver buffer
#define RX_BUFFER_SIZE 100
void Decode_frame(unsigned char data);
volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; 
volatile unsigned char rx_wr_index; 
volatile unsigned char RC_Flag;  
float yaw, pitch, roll, alt, tempr, press;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t hx, hy, hz;
float GPS_Altitude, Latitude_GPS, Longitude_GPS, Speed_GPS, Course_GPS;
unsigned char GPS_STA_Num;

bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
int fd;

ros::Publisher      	imu_pub;

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(const char* port)
{
	int fd; /* File descriptor for the port */
	
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}
	
	return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;
	
	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	                     ONOCR | OFILL | OPOST);

	#ifdef OLCUC 
  		config.c_oflag &= ~OLCUC; 
	#endif

  	#ifdef ONOEOT
  		config.c_oflag &= ~ONOEOT;
  	#endif

	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
        case 230400:
            if (cfsetispeed(&config, B230400) < 0 || cfsetospeed(&config, B230400) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;


		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;
			
			break;
	}
	
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

void close_port(int fd)
{
    printf("Stop the device!\n");
	close(fd);
}

void UART2_Get_IMU(void)
{
	int16_t temp;
	temp = 0;
	temp = rx_buffer[2];
	temp <<= 8;
	temp |= rx_buffer[3];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	yaw=(float)temp / 10.0f;
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	pitch=(float)temp / 10.0f;

	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	roll=(float)temp / 10.0f;

	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	alt=(float)temp / 10.0f;

	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	tempr=(float)temp / 10.0f;

	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	press=(float)temp * 10.0f;
}

void UART2_Get_Motion(void)
{
	int16_t temp;

	temp = 0;
	temp = rx_buffer[2];
	temp <<= 8;
	temp |= rx_buffer[3];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	ax=temp;

	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	ay=temp;

	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000)
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	az=temp;
	
	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000) 
	{
		temp = 0-(temp&0x7fff);
	}
	else temp = (temp&0x7fff);
	gx=temp;

	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
		temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gy=temp;

	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
		temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gz=temp;

	temp = 0;
	temp = rx_buffer[14];
	temp <<= 8;
	temp |= rx_buffer[15];
	if(temp&0x8000){
		temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hx=temp;

	temp = 0;
	temp = rx_buffer[16];
	temp <<= 8;
	temp |= rx_buffer[17];
	if(temp&0x8000){
		temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hy=temp;

	temp = 0;
	temp = rx_buffer[18];
	temp <<= 8;
	temp |= rx_buffer[19];
	if(temp&0x8000){
		temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hz=temp;
}

unsigned char Sum_check(void)
{ 
	unsigned char i;
	unsigned int checksum=0; 
	for(i=0;i<rx_buffer[0]-2;i++)
		checksum+=rx_buffer[i];
        if((checksum%256)==rx_buffer[rx_buffer[0]-2])
		return(0x01); //Checksum successful
	else
	return(0x00); //Checksum error
}

void UART2_CommandRoute(void)
{
	if(RC_Flag&b_rx_over)
	{
		RC_Flag&=~b_rx_over;
		if(Sum_check()){
			if(rx_buffer[1]==0xA1){
				UART2_Get_IMU();
			}
			if(rx_buffer[1]==0xA2){
				UART2_Get_Motion();
			}
		}
	}
}

void Decode_frame(uint8_t data)
{
	if(data==0xa5)
	{
		RC_Flag|=b_uart_head;
		rx_buffer[rx_wr_index++]=data;
	}
	else if(data==0x5a)
	{
		if(RC_Flag&b_uart_head)
		{
			rx_wr_index=0;
			RC_Flag&=~b_rx_over;
		}
		else
		{
			rx_buffer[rx_wr_index++]=data;
		}
		RC_Flag&=~b_uart_head;
	}
	else
	{
		rx_buffer[rx_wr_index++]=data;
		RC_Flag&=~b_uart_head;
		if(rx_wr_index==rx_buffer[0])
		{
			RC_Flag|=b_rx_over;
			UART2_CommandRoute();
		}
	}
	if(rx_wr_index==RX_BUFFER_SIZE)
	{
		rx_wr_index--;
	}
}

/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * and forwards the data once received.
 */
void serial_wait( void* serial_ptr )
{
	int fd = *((int*)serial_ptr);
	ros::Rate listen_rate(100);
	while (ros::ok())
	{
		uint8_t cp;
		if (read(fd, &cp, 1) > 0)
		{
			Decode_frame(cp);
		}
		else
		{
                	fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
		}
		//std_msgs::Float32 msg;
		sar_localization::Imu msg;
		//ros::ROS_WARN("The yaw angle: ");
		msg.header.stamp = ros::Time::now();
		msg.yaw = yaw;
		msg.pitch = pitch;
		msg.roll = roll;
		imu_pub.publish(msg);
		// If a message could be decoded, handle it
		if(debug)
		{
			fprintf(stderr,"Received serial data: ");
			unsigned int i;
			uint8_t buffer[128];
			for (i=0; i<128; i++)
			{
				unsigned char v=buffer[i];
				fprintf(stderr,"%02x ", v);
			}
			fprintf(stderr,"\n");
		}

        	ros::spinOnce();
		//listen_rate.sleep();
	}
}



int main(int argc, char **argv) 
{

	ros::init(argc, argv, "serial_publisher");
	ros::NodeHandle handle;
	imu_pub = handle.advertise<sar_localization::Imu>("imu", 1000);
	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 115200;
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";
	/* read program arguments */
	int i;
	for (i = 1; i < argc; i++) 
	{ /* argv[0] is "serial" */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf(commandline_usage, argv[0], uart_name, baudrate);
			return 0;
		}
		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}
		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}
		/* terminating MAVLink is allowed - yes/no */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
		if (strcmp(argv[i], "--debug") == 0) {
			debug = true;
		}
	}

	// SETUP SERIAL PORT
	// Exit if opening port failed
	// Open the serial port.
	if (!silent) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);
	fd = open_port(uart_name);
	if (fd == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	if (!silent) printf("Trying to configure %s.. ", uart_name);
	bool setup = setup_port(fd, baudrate, 8, 1, false, false);
	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	}
	if(fd < 0)
	{
		exit(noErrors);
	}
	
    
    	// Run indefinitely while the serial loop handles data
	if (!silent) printf("\nREADY, waiting for serial data.\n");
        int* fd_ptr = &fd;
   	/* GThread* serial_thread;
   	GError* err;
    	const gchar* mav_rcv = mav_rcv;
    	if (!g_thread_supported())
    	{
        	g_thread_init(NULL);
    	}
    	if (!silent)
    		printf("\nREADY, waiting for serial/ROS data.\n");
    	if ((serial_thread = g_thread_new(mav_rcv, (GThreadFunc)serial_wait, (void *)fd_ptr)) == NULL)
    	{
        	printf("Failed to create serial handling thread: %s!!\n", err->message);
        	g_error_free(err);
    	}*/

    	if(fd < 0)
    	{
       		exit(noErrors);
    	}
  
    	printf("\nSERIAL TO ROS BRIDGE STARTED - RUNNING..\n\n");
/**
     * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
*/ 
   // ros::spin();
  //
    	serial_wait(fd_ptr);
    
	close_port(fd);

	return 0;
}
