#include "krakenudp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>

#include <sqlite3.h>

#include "log.h"


// UDP allowes just under 65kB/packet but to ensure correct fragementation 
// it's probably best to just use a power of 2 here. This is in bytes. 
#define UDP_MAXLEN 1024 
#define NUM_CHANNEL 4

static netconf_t netconf; 
sqlite3 * db; 

bool open_db(settings_t * settings, const char * debug_info) 
{
    int retval; 
    char *zErrMsg = 0; 

    retval = sqlite3_open(settings->db_filename, &db); 
    if(retval) {
        log_error("Cannot open output database %s, message: %s\n", settings->db_filename, sqlite3_errmsg(db)); 
        return false; 
    } 
    log_info("Successfully opened output database %s\n", settings->db_filename); 
    log_info("Creating database structure\n");
    // TODO: change to adaptive number of channels 
    const char * sql = "CREATE TABLE data (id INTEGER PRIMARY KEY AUTOINCREMENT, " \
                        "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP, " \
                        "header BLOB," \
                        "ch1 BLOB," \
                        "ch2 BLOB," \
                        "ch3 BLOB," \
                        "ch4 BLOB)";
    retval = sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);

    if(retval != SQLITE_OK) {
        log_error("Could not create SQLite database! Error message: %s. %s\n", zErrMsg, debug_info);
        sqlite3_free(zErrMsg); 
        sqlite3_close(db);
        return false;
    } 
    return true; 
}



bool open_socket(settings_t * settings, const char * debug_info)
{
    const char * udp_addr = settings->udp_addr; 
    unsigned short port = settings->port;  
    log_info("Trying to connect to %s:%d", udp_addr, port);
    if ((netconf.sockfd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
        log_error("Could not create UDP output socket, %s", debug_info);
        netconf.isConnected = false;

        return false;
    } else {
        log_info("Successfully initialized socket");
    }
    memset(&netconf.addr, 0, sizeof(netconf.addr));
    // Filling server information

    netconf.addr.sin_family = AF_INET;
    netconf.addr.sin_port = htons(port);
    netconf.addr.sin_addr.s_addr = inet_addr(udp_addr);

    if (netconf.sockfd <= 0) {
        log_warn("Cannot send data because socket is closed, %s", debug_info);
        netconf.isConnected = false;
        return false;
    } else if (netconf.addr.sin_addr.s_addr <= 0) {
        log_warn("Cannot send data because IP address %s is invalid, %s", udp_addr, debug_info);
        netconf.isConnected = false;
        return false;
    }
    else {
        log_info("Sending UDP packets to %s, port %d, debug_info: %s", udp_addr, port, debug_info);
        netconf.isConnected = true;
    }
    return true;
}

bool init_data_output(settings_t * settings, const char * debug_info) {
    log_info("Initializiing output interface %s", debug_info); 
    switch (settings->opmode) {
        case NOOUTPUT: 
            log_info("Output is disabled. Nothing to do. %s\n", debug_info); 
            return true; 
        case UDP: 
            log_info("Opening UDP connection. %s\n", debug_info); 
            return open_socket(settings, debug_info); 
        case SQLITE: 
            log_info("Opening SQLite database. %s\n", debug_info); 
            return open_db(settings, debug_info); 
    }
    log_error("Couldn't initialize data output. Operation mode not recognized\n"); 
    return false; 
}


void emit_data(
        const settings_t * settings, 
        const char * data, 
        const unsigned int n_elem, 
        const unsigned int n_ch, 
        complex_t elem_size, 
        const iq_header_struct * header) {
    //  TODO: move n_ch and elem_size to init_data_output?
    switch (settings->opmode) {
        case NOOUTPUT: 
            // nothing to do
            return; 
        case UDP: 
            if (netconf.isConnected) {
                int remaining_bytes = n_elem * n_ch * elem_size;
                log_trace("Sending %d bytes via UDP from %p", remaining_bytes);
                while (remaining_bytes > 0) { // send the first packets as big as allowed

                    if (0 > sendto(netconf.sockfd, (const char *) data,
                                remaining_bytes > UDP_MAXLEN ? UDP_MAXLEN : remaining_bytes,
                                0, (const struct sockaddr *) &netconf.addr, sizeof(netconf.addr)))
                        log_warn("Sendto encountered error: %s", strerror(errno));

                    remaining_bytes -= UDP_MAXLEN;
                    data += UDP_MAXLEN;
                }
            }
            return; 
        case SQLITE: 
            if (n_ch != NUM_CHANNEL) {
                // TODO allow adjusting the amount of channels. 
                log_error("You're trying to save %d channels to the database. Currently only %d channels are supported, this "
                        "will most likely cause unexpected behaviour!", n_ch, NUM_CHANNEL); 
            } 
            const char * sql = "INSERT INTO data (header, ch1, ch2, ch3, ch4) VALUES (?, ?, ?, ?, ?)"; 
            sqlite3_stmt * stmt = NULL; 
            int retval; 
            retval = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL); 

            if (retval != SQLITE_OK) {
                log_error("SQLite error encountered. Error message: %s\n", sqlite3_errmsg(db)); 
            }

            // deinterleave data:

            char * ch1 = calloc(n_elem * elem_size, sizeof(char)); 
            char * ch2 = calloc(n_elem * elem_size, sizeof(char)); 
            char * ch3 = calloc(n_elem * elem_size, sizeof(char)); 
            char * ch4 = calloc(n_elem * elem_size, sizeof(char)); 
/* as it turns out, the data isn't actually interleaved, it's just concatenated. Stupid me.
            for(int s = 0; s < n_elem * elem_size; s += elem_size) {
                memcpy(&ch1[s], &data[4 * s + 0 * elem_size], elem_size); 
                memcpy(&ch2[s], &data[4 * s + 1 * elem_size], elem_size); 
                memcpy(&ch3[s], &data[4 * s + 2 * elem_size], elem_size); 
                memcpy(&ch4[s], &data[4 * s + 3 * elem_size], elem_size); 
            }
*/
            memcpy(&ch1, &data[0 * n_elem * elem_size], elem_size * n_elem); 
            memcpy(&ch2, &data[1 * n_elem * elem_size], elem_size * n_elem); 
            memcpy(&ch3, &data[2 * n_elem * elem_size], elem_size * n_elem); 
            memcpy(&ch4, &data[3 * n_elem * elem_size], elem_size * n_elem); 

            // TODO: finer error handling?
            retval = sqlite3_bind_blob(stmt, 1, header, sizeof(iq_header_struct), SQLITE_STATIC); 
            retval = sqlite3_bind_blob(stmt, 2, ch1, n_elem * elem_size, SQLITE_STATIC);  
            retval = sqlite3_bind_blob(stmt, 3, ch2, n_elem * elem_size, SQLITE_STATIC);  
            retval = sqlite3_bind_blob(stmt, 4, ch3, n_elem * elem_size, SQLITE_STATIC);  
            retval = sqlite3_bind_blob(stmt, 5, ch4, n_elem * elem_size, SQLITE_STATIC);  
            if (retval != SQLITE_OK) {
                log_error("SQLite error encountered. Error message: %s\n", sqlite3_errmsg(db)); 
            }

            retval = sqlite3_step(stmt);
            if (retval != SQLITE_DONE) {
                log_error("SQLite error encountered. Error message: %s\n", sqlite3_errmsg(db)); 
            }
            sqlite3_finalize(stmt);
            free(ch1); 
            free(ch2); 
            free(ch3); 
            free(ch4); 

            return;  

        default: 
            log_error("Cannot parse operation mode %s. Unable to save data", settings->opmode); 
            return; 
    }
}
