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

#include <stdio.h>
#include <sndfile.h>

#include "log.h"


// UDP allowes just under 65kB/packet but to ensure correct fragementation 
// it's probably best to just use a power of 2 here. This is in bytes. 
#define UDP_MAXLEN 1024 
#define NUM_CHANNEL 4

bool open_db(settings_t * settings, const char * debug_info) 
{
    int retval; 
    char *zErrMsg = 0; 

    retval = sqlite3_open(settings->filename, &settings->out.sqlite); 
    if(retval) {
        log_error("Cannot open output database %s, message: %s", settings->filename, sqlite3_errmsg(settings->out.sqlite)); 
        return false; 
    } 
    log_info("Successfully opened output database %s", settings->filename); 
    log_info("Creating database structure");
    // TODO: change to adaptive number of channels 
    const char * sql = "CREATE TABLE IF NOT EXISTS data (id INTEGER PRIMARY KEY AUTOINCREMENT, " \
                        "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP, " \
                        "header BLOB," \
                        "ch1 BLOB," \
                        "ch2 BLOB," \
                        "ch3 BLOB," \
                        "ch4 BLOB)";
    retval = sqlite3_exec(settings->out.sqlite, sql, NULL, NULL, &zErrMsg);

    if(retval != SQLITE_OK) {
        log_error("Could not create SQLite table! Error message: %s. %s", zErrMsg, debug_info);
        sqlite3_free(zErrMsg); 
        sqlite3_close(settings->out.sqlite);
        return false;
    } 
    return true; 
}

bool open_wav(settings_t * settings, const char * debug_info) 
{
    SF_INFO wavFileInfo; 

    wavFileInfo.channels = 2 * NUM_CHANNEL;  // one for I, one for Q
    wavFileInfo.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT; // TODO change to uint8 for the first steps in the chain
    wavFileInfo.samplerate = 1024000;

    settings->out.wavfile = sf_open(settings->filename, SFM_WRITE, &wavFileInfo); 

    if (settings->out.wavfile == NULL) {
        log_error("Could not open WAV file"); 
        return false; 
    }

    return true; 
}



bool open_socket(settings_t * settings, const char * debug_info)
{
    const char * udp_addr = settings->udp_addr; 
    unsigned short port = settings->port;  
    log_info("Trying to connect to %s:%d", udp_addr, port);
    if ((settings->out.udp_data.sockfd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
        log_error("Could not create UDP output socket, %s", debug_info);
        settings->out.udp_data.isConnected = false;

        return false;
    } else {
        log_info("Successfully initialized socket");
    }
    memset(&settings->out.udp_data.addr, 0, sizeof(settings->out.udp_data.addr));
    // Filling server information

    settings->out.udp_data.addr.sin_family = AF_INET;
    settings->out.udp_data.addr.sin_port = htons(port);
    settings->out.udp_data.addr.sin_addr.s_addr = inet_addr(udp_addr);

    if (settings->out.udp_data.sockfd <= 0) {
        log_warn("Cannot send data because socket is closed, %s", debug_info);
        settings->out.udp_data.isConnected = false;
        return false;
    } else if (settings->out.udp_data.addr.sin_addr.s_addr <= 0) {
        log_warn("Cannot send data because IP address %s is invalid, %s", udp_addr, debug_info);
        settings->out.udp_data.isConnected = false;
        return false;
    }
    else {
        log_info("Sending UDP packets to %s, port %d, debug_info: %s", udp_addr, port, debug_info);
        settings->out.udp_data.isConnected = true;
    }
    return true;
}

bool init_data_output(settings_t * settings, const char * debug_info) {
    log_info("Initializiing output interface %s", debug_info); 
    switch (settings->opmode) {
        case NOOUTPUT: 
            log_info("Output is disabled. Nothing to do. %s", debug_info); 
            return true; 
        case UDP: 
            log_info("Opening UDP connection. %s", debug_info); 
            return open_socket(settings, debug_info); 
        case SQLITE: 
            log_info("Opening SQLite database. %s", debug_info); 
            return open_db(settings, debug_info); 
        case WAV:
            log_info("Opening WAV file. %s", debug_info); 
            return open_wav(settings, debug_info); 
    }
    log_error("Couldn't initialize data output. Operation mode not recognized"); 
    return false; 
}


void emit_data(
        const settings_t * settings, 
        const void * data, 
        const unsigned int n_elem, 
        const unsigned int n_ch, 
        const iq_header_struct * header) {
    //  TODO: move n_ch and settings->data_type to init_data_output?
    log_trace("Emitting data, data_p %p, n_elem: %d, n_ch %d", data, n_elem, n_ch);
    switch (settings->opmode) {
        case NOOUTPUT: 
            // nothing to do
            return; 
        case UDP: 
            if (settings->out.udp_data.isConnected) {
                int remaining_bytes = n_elem * n_ch * settings->data_type;
                log_trace("Sending %d bytes via UDP from %p", remaining_bytes);
                while (remaining_bytes > 0) { // send the first packets as big as allowed

                    if (0 > sendto(settings->out.udp_data.sockfd, (const char *) data,
                                remaining_bytes > UDP_MAXLEN ? UDP_MAXLEN : remaining_bytes,
                                0, (const struct sockaddr *) &settings->out.udp_data.addr, sizeof(settings->out.udp_data.addr)))
                        log_warn("Sendto encountered error: %s", strerror(errno));

                    remaining_bytes -= UDP_MAXLEN;
                    data += UDP_MAXLEN;
                }
            }
            return; 
        case SQLITE: 
            {   // curly braces for scoping the variables
                if (n_ch != NUM_CHANNEL) {
                    // TODO allow adjusting the amount of channels. 
                    log_error("You're trying to save %d channels to the database. Currently only %d channels are supported, this "
                            "will most likely cause unexpected behaviour!", n_ch, NUM_CHANNEL); 
                } 
                log_trace("Emmiting data via SQLite to file %s, database struct %p", settings->filename, settings->out.sqlite); 
                const char * sql = "INSERT INTO data (header, ch1, ch2, ch3, ch4) VALUES (?, ?, ?, ?, ?)"; 
                sqlite3_stmt * stmt; 
                int retval; 
                retval = sqlite3_prepare_v2(settings->out.sqlite, sql, -1, &stmt, NULL); 

                if (retval != SQLITE_OK) {
                    log_error("SQLite error encountered. Error message: %s", sqlite3_errmsg(settings->out.sqlite)); 
                } else
                {
                    log_trace("Statement prepared");
                }

                // data is not interleaved, just concatenated. 
                retval = sqlite3_bind_blob(stmt, 1, header, sizeof(iq_header_struct), SQLITE_TRANSIENT); 
                for (int i = 0; i < NUM_CHANNEL; i++) {
                    // calculate legth in bytes from begin of data array: 
                    const int len_frame_1ch = n_elem * settings->data_type; 
                    // typecast data pointer to byte array: 
                    char * data_byte = (char *) data;
                    retval = sqlite3_bind_blob(stmt, i+2, &data_byte[i * len_frame_1ch], len_frame_1ch, SQLITE_STATIC);  
                    if (retval != SQLITE_OK) {
                        log_error("SQLite error encountered. Error message: %s", sqlite3_errmsg(settings->out.sqlite)); 
                    } else {
                        log_trace("Channel data for channel %d bound, retval was %d, data pointer %p, data_length %d", i, retval, &data_byte[i * len_frame_1ch], len_frame_1ch);
                    }
                }
                log_trace("Pre-step");
                retval = sqlite3_step(stmt);
                log_trace("post-step"); 
                if (retval != SQLITE_DONE) {
                    log_error("SQLite error encountered. Error message: %s", sqlite3_errmsg(settings->out.sqlite)); 
                } else {
                    log_trace("statement executed");
                }

                retval = sqlite3_reset(stmt);
                
                if (retval != SQLITE_OK) {
                    log_error("SQLite error encountered. Error message: %s", sqlite3_errmsg(settings->out.sqlite)); 
                } else {
                    log_trace("statement reset");
                }

                retval = sqlite3_finalize(stmt);
                if (retval != SQLITE_OK) {
                    log_error("SQLite error encountered. Error message: %s", sqlite3_errmsg(settings->out.sqlite)); 
                } else {
                    log_trace("statement finalized");
                }
                sqlite3_exec(settings->out.sqlite, "Commit", NULL, NULL, NULL); 
                log_trace("Successfully inserted %d byte of data into SQLite DB", NUM_CHANNEL * n_elem * settings->data_type);
                
                return; 
            }

        case WAV:
            {
                // deinterleave real and complex to concatenated channels
                // we have 4 channels concatenated: re1,im1,re1,im (interleaved) | re2/im2 (interleaved) | ...
                // and want them re1|im1|re2|im2|...

                char * deinterleaved  = calloc(n_elem * NUM_CHANNEL, settings->data_type);
                int nbyteswritten; 

                // settings->data_type gives the size of one IQ sample (complex). 
                // the real and imaginary parts are therefore half as big: 
                const size_t data_type_real = settings->data_type / 2; 
                for (int c = 0; c < NUM_CHANNEL; c++) {
                    for (int s = 0; s < n_elem; s++) {
                        // calculate current offset in bytes: 
                        int offset_src      = (c * n_elem + s) * settings->data_type; 
                        int offset_dst_real = (2 * c * n_elem + s) * data_type_real; 
                        int offset_dst_imag = ((2 * c + 1) * n_elem + s) * data_type_real; 
                        // copy real and imag:  
                        memcpy(&deinterleaved[offset_dst_real], &((const char *)data)[offset_src], data_type_real); 
                        memcpy(&deinterleaved[offset_dst_imag], &((const char * )data)[offset_src + data_type_real], data_type_real); 
                    }
                } 
                // we have now 8 "channels" of data - ch1 real | ch1 imag | ch2 real | ch2 imag | ... 
                // save this to file. 
                if(settings->data_type == COMPLEX_INT8)
                {
                    // convert to float in [-1, 1]-interval: 
                    float * float_data = (float*) calloc(n_elem * NUM_CHANNEL * 2, sizeof(float)); 
                    for (int i = 0; i < n_elem * NUM_CHANNEL * 2; i++) {
                        float_data[i] =  ((float) deinterleaved[i] - 128.0f) / 128.0f; 
                    }

                    nbyteswritten = sf_writef_float(settings->out.wavfile, float_data, n_elem); 
                    free(float_data); 
                } else if (settings->data_type == COMPLEX_FLOAT) {
                    nbyteswritten = sf_writef_float(settings->out.wavfile, (const float*) deinterleaved, n_elem); 
                }
                if (nbyteswritten == 0) {
                    log_error("Could not write to wav file %s. Error Message: %s", settings->filename,  sf_strerror(settings->out.wavfile)); 
                }
                // flush buffers: 
                sf_write_sync(settings->out.wavfile); 
                free(deinterleaved);

                return; 
            }
        default: 
            log_error("Cannot parse operation mode %s. Unable to save data", settings->opmode); 
            return; 
    }
}
