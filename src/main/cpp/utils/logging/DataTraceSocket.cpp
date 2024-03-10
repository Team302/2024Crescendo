//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// FRC includes

// Team 302 includes
#include "utils/logging/DataTrace.h"
#include <utils/logging/DataTraceSocket.h>

// Third Party Includes

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#ifdef INCLUDE_DATA_TRACE
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h>
#endif

#define PORT 30200

DataTraceSocket::DataTraceSocket() : m_timer()
{
}

void DataTraceSocket::Connect(void)
{
#ifdef INCLUDE_DATA_TRACE
    short octets[10];
    char host[256];
    char *IP;
    struct hostent *host_entry;

    gethostname(host, sizeof(host));                                 // find the host name
    host_entry = gethostbyname(host);                                // find host information
    IP = inet_ntoa(*((struct in_addr *)host_entry->h_addr_list[0])); // Convert into IP string
    printf("================== Current Host Name: %s\n", host);
    printf("================== Host IP: %s\n", IP);
    extractIpAddress(IP, octets);
    printf("================== Host IP(reconstructed): %d.%d.%d.%d\n", octets[0], octets[1], octets[2], octets[3]);

    // since the DataTrace server is on a laptop that is getting its IP address through DHCP
    // we need to scan a range of IP addresses starting with 10.3.2.3 to for example 10.3.2.10
    // note that 10.3.2.1 is the compbot radio and 10.3.2.2 is the compbot roborio
    // While this code is looking for the socket server, it will block the robot code
    // maybe we need to find a better way
    for (int i = 166; i < 167; i++)
    {
        if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
            printf("================== DataTrace Socket creation error \n");
        else
        {
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(PORT);

            char ipAddressBuffer[30] = {0};
            // Convert IPv4 and IPv6 addresses from text to binary form

            sprintf(ipAddressBuffer, "%d.%d.%d.%d", octets[0], octets[1], octets[2], i);
            printf("================== Trying to connect to ip address %s \n", ipAddressBuffer);

            if (inet_pton(AF_INET, ipAddressBuffer, &serv_addr.sin_addr) <= 0)
            {
                printf("================== Invalid address or Address not supported \n");
            }

            if ((status = connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
            {
                printf("================== Connection Failed \n");
                Disconnect();
            }
            else
            {
                printf("================== Connection established with %s \n", ipAddressBuffer);
                isConnected = true;
                m_timer.Restart();
                break;
            }
        }
    }
#endif
}

void DataTraceSocket::Disconnect(void)
{
#ifdef INCLUDE_DATA_TRACE
    if (client_fd >= 0)
        close(client_fd);

    isConnected = false;
    m_timer.Stop();
#endif
}

void DataTraceSocket::SendData(void)
{
#ifdef INCLUDE_DATA_TRACE
    if (isConnected)
        send(client_fd, sendBuffer, strlen(sendBuffer), 0);
#endif
}

void DataTraceSocket::extractIpAddress(const char *sourceString, short *ipAddress)
{
    unsigned short len = 0;
    unsigned char oct[4] = {0}, cnt = 0, cnt1 = 0, i, buf[5];

    len = strlen(sourceString);
    for (i = 0; i < len; i++)
    {
        if (sourceString[i] != '.')
        {
            buf[cnt++] = sourceString[i];
        }
        if (sourceString[i] == '.' || i == len - 1)
        {
            buf[cnt] = '\0';
            cnt = 0;
            oct[cnt1++] = atoi((const char *)(buf));
        }
    }
    ipAddress[0] = oct[0];
    ipAddress[1] = oct[1];
    ipAddress[2] = oct[2];
    ipAddress[3] = oct[3];
}
