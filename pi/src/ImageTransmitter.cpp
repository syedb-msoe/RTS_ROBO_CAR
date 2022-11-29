/**
 * @file ImageTransmitter.cpp
 * @author  Walter Schilling (schilling@msoe.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 *
 * This code is developed as part of the MSOE SE3910 Real Time Systems course,
 * but can be freely used by others.
 *
 * SE3910 Real Time Systems is a required course for students studying the
 * discipline of software engineering.
 *
 * This Software is provided under the License on an "AS IS" basis and
 * without warranties of any kind concerning the Software, including
 * without limitation merchantability, fitness for a particular purpose,
 * absence of defects or errors, accuracy, and non-infringement of
 * intellectual property rights other than copyright. This disclaimer
 * of warranty is an essential part of the License and a condition for
 * the grant of any rights to this Software.
 *
 * @section DESCRIPTION
 *      This class will transmit an image to a remote device.  The image will be transmitted as a set of UDP datagrams.
 */

#include "ImageTransmitter.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdint.h>
#include "time_util.h"
#include <string.h>

/**
 * This will instantiate a new instance of this class. It will copy the machine name into a heap allocated string and update the port.
 * @param machineName This is the name of the machine that the image is to be streamed to.
 * @param port This is the udp port number that the machine is to connect to.
 */
ImageTransmitter::ImageTransmitter(char *machineName, int port) {
	this->destinationMachineName = machineName;
	this->myPort = port;
}

/**
 * This is the destructor. It will free all allocated memory.
 */
ImageTransmitter::~ImageTransmitter() {
// TODO
}

/**
 * This method will stream via udp the image to the remote device.
 * @param image This is the image that is to be sent.
 * @return The return will be 0 if successful or -1 if there is a failure.
 */
int ImageTransmitter::streamImage(Mat *image) {
	/**
	 * 1.0 If the image and destination machine are not null,
	 */
	if ((image != NULL) && (destinationMachineName != NULL)) {
		/**
		 * 1.1 Increment the image count.
		 */
		imageCount++;

		/**
		 * 1.2 Initialize the socket sockfd to be a DGRAM.
		 */
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);

		/**
		 * 1.3 If there is an error with 1b, abort with an error message and return -1.
		 */
		if (sockfd < 0) {
			perror("ERROR opening socket");
			return -1;
		}

		/**
		 * 1.4 Get the host by name and set up a server instance.
		 */
		struct hostent *server;
		server = gethostbyname(destinationMachineName);

		/**
		 * 1.5 If the server is NULL, print out an error and return -1.
		 */
		if (server == NULL) {
			fprintf(stderr, "ERROR, no such host\n");
			return -1;;
		}

		/**
		 * 1.6 Set up the rest of the UDP parameters.
		 * */
		struct sockaddr_in serv_addr;
		// Zero out the structure.
		bzero((char*) &serv_addr, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		// Now copy the appropriate data over.
		memcpy((char*) &serv_addr.sin_addr.s_addr, (char*) server->h_addr, server->h_length);

		/**
		 * 1.7 Set the port for the system.
		 */
		serv_addr.sin_port = htons(myPort);

		/**
		 * 1.8 Obtain the image rows, columns, message size, channels in the image, and required buffer allocation size (which is ((3 * columns + 24) * linesPerUDPDatagram) + 4).
		 */
		int rows = image->rows;
		int cols = image->cols;
		int channels = image->channels();
		int allocationSize = (channels * cols + 24) + 4;

		/**
		 * 1.9 Allocate a buffer of message size length on the heap.
		 */
		uchar *buffer = new uchar[allocationSize];

		/**
		 * 1.10 Obtain the current timestamp in ms using the time_util library.
		 */
		int timeStamp = current_timestamp();

		/**
		 * 1.12 Declare a variable that will keep track of the index into the image (i.e. which row is being packed right now),
		 * as well as another variable which counts how many rows have been packed into the given UDP datagram.
		 */
		int index = 0;

		/**
		 * 1.11 Iterate over the rows, sending one udp datagram per row.
		 * In constructing your loop, first iterate over the even rows then over the odd rows to minimize the impact of a loss of UDP datagrams.
		 */

		while (index < rows) {
			/**
			 * 1.11.1.1 Starting at the beginning of the UDP datagram, pack it with the following information:
			 * Integer 0: The number of bytes per pixel or # of channels in the image.
			 * Integer 0: The start time for the transmission
			 * Integer 1: The current timestamp for the current portion of the image
			 * Integer 2: The count of the image.
			 * Integer 3: The number of rows in the image.
			 * Integer 4: The number of columns in the image
			 * Integer 5: The index being sent
			 * Followed by: An array of columns * (numberOfChannels) bytes, representing the pixels in the current row.
			 *
			 * The integers all need to have their endianess corrected before being sent.  The data array does not.
			 *
			 * Overall, there will be linesPerUDPDatagram instances of this in the allocated space, representing linesPerUDPDatagram lines being sent out at a time.
			 */
            ((int *)buffer)[0] = htonl(channels);
            ((int *)buffer)[1] = htonl(timeStamp);
            ((int *)buffer)[2] = htonl(current_timestamp());
            ((int *)buffer)[3] = htonl(imageCount);
            ((int *)buffer)[4] = htonl(rows);
            ((int *)buffer)[5] = htonl(cols);
            ((int *)buffer)[6] = htonl(index);

			/**
			 * * 1.11.1.2 Obtain the row of the of image Matrix.
			 * */
			Mat row = image->row(index);

			/**
			 * 1.11.1.3 Now copy the data from the row into the UDP datagram for transmission.
			 */
			memcpy(buffer + 28, row.data, cols * channels);

			/**
			 * 1.11.2 Send to message to the destination as a UDP datagram.
			 */
			int serverlen = sizeof(serv_addr);
			int lres = sendto(sockfd, buffer, allocationSize, 0,
					(struct sockaddr*) &serv_addr, serverlen);

			if (lres < 0) {
				printf("%d c %d ", lres, errno);
				perror("Transmit:");
				exit(-1);
			}
			index++;
		}
		/**
		 * 1.12	Free the message that was allocated on the heap.
		 */
		delete buffer;
		/**
		 * 1.13 Close the socket down.
		 */
		close(sockfd);
	}
	return 0;
}
