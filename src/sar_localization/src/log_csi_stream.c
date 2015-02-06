/*
 * (c) 2008-2011 Daniel Halperin <dhalperi@cs.washington.edu>
 * (c) 2015 Shengkai Zhang <shengkai.zhang@gmail.com> modified
 */
#include <arpa/inet.h>
#include <sys/socket.h>
#include <linux/socket.h>
#include <linux/netlink.h>
#include <linux/connector.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "/opt/eigen/Eigen/Eigen"
#include <complex>
#include <vector>
#include <math.h>

#define MAX_PAYLOAD 2048
#define SLOW_MSG_CNT 1

int sock_fd = -1;							// the socket
//FILE* out = NULL;

//void check_usage(int argc, char** argv);

//FILE* open_file(char* filename, char* spec);

void caught_signal(int sig);

void exit_program(int code);
void exit_program_err(int code, char* func);

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
	/* Local variables */
	struct sockaddr_nl proc_addr, kern_addr;	// addrs for recv, send, bind
	struct cn_msg *cmsg;
	char buf[4096];
	int ret;
	unsigned short l;	//l2;
	int count = 0;

	/* Make sure usage is correct */
	//check_usage(argc, argv);

	/* Open and check log file */
	//out = open_file(argv[1], "w");

	/* Setup the socket */
	sock_fd = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_CONNECTOR);
	if (sock_fd == -1)
		exit_program_err(-1, "socket");

	/* Initialize the address structs */
	memset(&proc_addr, 0, sizeof(struct sockaddr_nl));
	proc_addr.nl_family = AF_NETLINK;
	proc_addr.nl_pid = getpid();			// this process' PID
	proc_addr.nl_groups = CN_IDX_IWLAGN;
	memset(&kern_addr, 0, sizeof(struct sockaddr_nl));
	kern_addr.nl_family = AF_NETLINK;
	kern_addr.nl_pid = 0;					// kernel
	kern_addr.nl_groups = CN_IDX_IWLAGN;

	/* Now bind the socket */
	if (bind(sock_fd, (struct sockaddr *)&proc_addr, sizeof(struct sockaddr_nl)) == -1)
		exit_program_err(-1, "bind");

	/* And subscribe to netlink group */
	{
		int on = proc_addr.nl_groups;
		ret = setsockopt(sock_fd, 270, NETLINK_ADD_MEMBERSHIP, &on, sizeof(on));
		if (ret)
			exit_program_err(-1, "setsockopt");
	}

	/* Set up the "caught_signal" function as this program's sig handler */
	signal(SIGINT, caught_signal);

	/* Poll socket forever waiting for a message */
	while (1)
	{
		/* Receive from socket with infinite timeout */
		ret = recv(sock_fd, buf, sizeof(buf), 0);
		if (ret == -1)
			exit_program_err(-1, "recv");
		/* Pull out the message portion and print some stats */
		cmsg = NLMSG_DATA(buf);
		if (count % SLOW_MSG_CNT == 0)
			printf("received %d bytes: id: %d val: %d seq: %d clen: %d\n", cmsg->len, cmsg->id.idx, cmsg->id.val, cmsg->seq, cmsg->len);
		/* Log the data to file */
		l = (unsigned short) cmsg->len;
		//l2 = htons(l);
		//fwrite(&l2, 1, sizeof(unsigned short), out);
		//ret = fwrite(cmsg->data, 1, l, out);
		char *pt = cmsg->data;
		unsigned short data_code = (unsigned short) *pt;
		++pt;
		
		if(data_code == 187)	//Get beamforming
		{
			++count;
			unsigned long timestamp_low = pt[0] + (pt[1]<<8) + (pt[2]<<16) + (pt[3]<<24);
			unsigned short bfee_count = pt[4] + (pt[5]<<8);
			unsigned int Nrx = pt[8];
			unsigned int Ntx = pt[9];
			unsigned int rssi_a = pt[10];
			unsigned int rssi_b = pt[11];
			unsigned int rssi_c = pt[12];
			char noise = pt[13];
			unsigned int agc = pt[14];
			unsigned int antenna_sel = pt[15];
			unsigned int len = pt[16] + (pt[17]<<8);
			unsigned int fake_rate_n_flags = pt[18] + (pt[19]<<8);
			unsigned int calc_len = (30 * (Nrx * Ntx * 8 * 2 + 3) + 7)/8;
			unsigned int i, j, k;
			unsigned int index = 0, remainder;
			unsigned char *payload = &pt[20];
			char tmp;
			int size[] = {Ntx, Nrx, 30};
			//Eigen::Matrix<std::vector<complex<double> >, Ntx, Nrx> csi_entry;
			//Note!!! Here we only log one transmitter's csi_entry, i.e., the first transmitter	
			Eigen::MatrixXcd csi_entry(Nrx, 30);
			//Check that length matches what it should
			if(len != calc_len)
			{
				perror("Wrong beamforming matrix size");
				exit(0);	
			}
			
			for(i = 0; i < 30; ++i)
			{
				index += 3;
				remainder = index % 8;
				//complex<double> tmpComplex;
				for(j = 0; j < Nrx; ++j)
				{
					for(k = 0; k < Ntx; ++k)
					{
						tmp = (payload[index/8]>>remainder) | (payload[index/8+1] << (8-remainder));
						double real = (double) tmp;
						tmp = (payload[index/8+1]>>remainder) | (payload[index/8+2]<<(8-remainder));
						double image = (double) tmp;
						complex<double> tmpCpmlex(real, image);
						if(k==0)
						{
							csi_entry(j,i) = tmpCpmlex;
						}
						index += 16;
					}
				}
			}
			//Get scaled entry
	                Eigen::MatrixXcd csi_entry_conj(Nrx, 30);
	                Eigen::MatrixXd csi_entry_sq(Nrx, 30);  //dot product with csi_entry's conjugate
        	        csi_entry_conj = csi_entry.conjugate();
                	csi_entry_sq = csi_entry.array() * csi_entry_conj.array();
	                double csi_entry_pwr = csi_entry_sq.sum();
	                //Compute total rssi
	                double rssi_mag = 0;
	                if(rssi_a != 0)
	                {
	                        rssi_mag = rssi_mag + pow(10, (rssi_a/10.0) );
	                }
	                if(rssi_b != 0)
	                {
	                        rssi_mag = rssi_mag + pow(10, (rssi_b/10.0) );
	                }
	                if(rssi_c != 0)
	                {
	                        rssi_mag = rssi_mag + pow(10, (rssi_c/10.0) );
	                }
	                double rssi_total = 10*log10(rssi_mag) - 44 - agc;
	                double rssi_pwr = pow(10, (rssi_total/10.0) );
	                double scale = rssi_pwr / (csi_entry_pwr / 30);
	                double noise_db = 0;
	                if( (double) noise == -127)
	                {
	                        noise_db = -92;
	                }
	                else
	                {
	                        noise_db = (double) noise;
	                }
	                double thermal_noise_pwr = pow(10, (noise_db/10) );
	                double quant_error_pwr = scale * (Nrx * Ntx);
	                double total_noise_pwr = thermal_noise_pwr + quant_error_pwr;
			Eigen::MatrixXcd csi(Nrx, 30);
			csi = csi_entry * sqrt(scale / total_noise_pwr);
			if(Ntx == 2)
			{
				csi = csi*sqrt(2);
			}
			else if(Ntx == 3)
			{
				csi = csi*1.6788;
			}
			//End of CSI extraction
			//Next
			//Start to publish CSI by ROS Msg

		}
		
		if (count % 100 == 0)
			printf("receive %d bytes [msgcnt=%u]\n", ret, count);
		//++count;
		if (ret != l)
			exit_program_err(1, "fwrite");
	}

	exit_program(0);
	return 0;
}

/*
void check_usage(int argc, char** argv)
{
	if (argc != 2)
	{
		fprintf(stderr, "Usage: log_to_file <output_file>\n");
		exit_program(1);
	}
}

FILE* open_file(char* filename, char* spec)
{
	FILE* fp = fopen(filename, spec);
	if (!fp)
	{
		perror("fopen");
		exit_program(1);
	}
	return fp;
}
*/
void caught_signal(int sig)
{
	fprintf(stderr, "Caught signal %d\n", sig);
	exit_program(0);
}

void exit_program(int code)
{
	//if (out)
	//{
	//	fclose(out);
	//	out = NULL;
	//}
	if (sock_fd != -1)
	{
		close(sock_fd);
		sock_fd = -1;
	}
	exit(code);
}

void exit_program_err(int code, char* func)
{
	perror(func);
	exit_program(code);
}
