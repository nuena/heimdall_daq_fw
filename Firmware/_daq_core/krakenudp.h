#pragma once
#include <sys/socket.h>
#include <stdbool.h>
#include <netinet/in.h>

typedef struct
{
    int sockfd;
    struct sockaddr_in addr;
    bool isConnected;
} netconf_t;

/**
 * Try to open and configure an UDP socket for data output.
 * @param netconf empty struct, will be populated
 * @param port destination port
 * @param debug_info String that will be added to the debug outputs.
 * @return true on success, false on fail
 */
bool open_socket(netconf_t * netconf,  const char * udp_addr, unsigned short port, const char * debug_info);

/**
 * Send raw data to specified network destination
 * @param netconf network destination configuration
 * @param data raw data
 * @param n_elem number of elements
 * @param elem_size size of each element
 */
void send_data(netconf_t * netconf, void * data,  unsigned int n_elem, size_t elem_size);
