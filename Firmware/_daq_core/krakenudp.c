#include "krakenudp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include "log.h"

#define UDPDEST "127.0.0.1"
#define UDP_MAXLEN 1024  // UDP allowes just under 65kB/packet but to ensure correct fragementation it's probably best to just use a power of 2 here


bool open_socket(netconf_t * netconf, const uint16_t port, const char * debug_info)
{
    log_info("Trying to connect to %s:%d", UDPDEST, port);
    if ((netconf->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
        log_error("Could not create UDP output socket, %s", debug_info);
        netconf->isConnected = false;

        return false;
    } else {
        log_info("Successfully initialized socket");
    }
    netconf->isConnected = true;
    memset(&netconf->addr, 0, sizeof(netconf->addr));
    // Filling server information

    netconf->addr.sin_family = AF_INET;
    netconf->addr.sin_port = htons(port);
    netconf->addr.sin_addr.s_addr = inet_addr(UDPDEST);

    if (netconf->sockfd > 0) {
        log_info("Sending UDP packets to %s, port %d, debug_info: %s", UDPDEST, port, debug_info);
        log_warn("Sending UDP Test packet");
        sendto(netconf->sockfd, "test", 4,
               MSG_DONTWAIT, (const struct sockaddr*) &netconf->addr, sizeof(netconf->addr));

    } else {
        log_warn("Cannot send data because socket is closed, %s", debug_info);
        return false;
    }


    return true;
}

void send_data(netconf_t * netconf, void * data, const unsigned int n_elem, const size_t elem_size) {
    /*
     * send n_elem * elem_size bytes to the specified socket in potentially several packets.
     */
    if (netconf->sockfd > 0 && netconf->isConnected) {
        int remaining_bytes = n_elem * elem_size;
        log_trace("Sending %d bytes via UDP from %p", remaining_bytes);
        while (remaining_bytes > 0) { // send the first packets as big as allowed

            if (0 > sendto(netconf->sockfd, (const char *) data,
                           remaining_bytes > UDP_MAXLEN ? UDP_MAXLEN : remaining_bytes,
                           MSG_DONTWAIT, (const struct sockaddr *) &netconf->addr, sizeof(netconf->addr)))
                log_warn("Sendto encountered error: %s", strerror(errno));

            remaining_bytes -= UDP_MAXLEN;
            data += UDP_MAXLEN;
        }
    }
}