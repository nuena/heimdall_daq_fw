#pragma once
#include <sys/socket.h>
#include <stdbool.h>
#include <netinet/in.h>
#include "iq_header.h"

typedef enum 
{
    NOOUTPUT, 
    UDP, 
    SQLITE, 
    WAV
} mode; 

typedef enum
{
    COMPLEX_INT8 = 2, 
    COMPLEX_FLOAT = 8
} complex_t; 

typedef struct 
{
    mode opmode; 
    const char * udp_addr; 
    unsigned short port; 
    const char * filename; 
} settings_t; 
    

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
 *
 * TODO make private?
 */
//bool open_socket(settings_t * settings, const char * debug_info);

/**
 * Send raw data to specified network destination
 * @param netconf network destination configuration
 * @param data raw data
 * @param n_elem number of elements
 * @param elem_size size of each element
 */
void emit_data(
        const settings_t * settings, 
        const char * data, 
        const unsigned int n_elem, 
        const unsigned int n_ch, 
        complex_t elem_size, 
        const iq_header_struct * header);


/**
 * Open SQLite database. 
 * TODO: check if this should be in header or "private"
 */
//bool open_db(settings_t * settings, const char * debug_info); 


/**
 * Initialize the output module.
 * @param settings required settings (mode, UDP address, UDP port, sqlite database). Set to placeholder values if not required
 * @param debug_info String that will be added to the debug outputs
 * @return true on success, false on fail
 */
bool init_data_output(settings_t * settings, const char * debug_info);
