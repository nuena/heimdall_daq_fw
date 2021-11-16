/*
 *
 * Description :
 *  Implements squelching on the data stream. The built-in trigger function checks
 *  the amplitude of the input signal to capture short signal bursts.
 *
 * Project : HeIMDALL DAQ Firmware
 * Author  : Tamas Peto
 * License : GNU GPL V3
 *
 * Copyright (C) 2018-2020  Tamás Pető
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/*
 *  Implementation note:
 *  Output buffer size must be the multiple of the input buffer size!
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include "rtl_daq.h"
#include "log.h"
#include "ini.h"
#include "iq_header.h"
#include "sh_mem_util.h"
#include "krakenudp.h"
#include <math.h>


#define INI_FNAME "daq_chain_config.ini"
#define SQC_FNAME "_data_control/squelch_control_fifo"
#define FATAL_ERR(l) log_fatal(l); return -1;

#define CHECK_INTERVAL_MS 5000 // TODO: move to configuration parameter!

#define POWER // define "POWER" to use power approximation
// #define NAIVE // define "NAIVE" to use simple approximation
// todo improve that instead of using macros...



static float squelch_threshold = 0; // Configure the amplitude threshold level 0..1
static float min_th            = 0;
static float max_th            = 0;
static int   scan_start        = 0; // Trigger scanning will be started at this sample offset

static sem_t config_sem;
pthread_t fifo_read_thread;
static int exit_flag           = 0;
static int control_signal; /*0- nothing to do, 1- refres threshold */

static int checked_batches = 0;
static int triggered_batches = 0;

static long triggered_samples_count = 0;
static long nontriggered_samples_count = 0;

/*
 * This structure stores the configuration parameters,
 * that are loaded from the ini file
 */
typedef struct
{
    int num_ch;
    int cpi_size;
    int log_level;
    float squelch_threshold;
    const char * udp_addr;
    uint16_t udp_port;
    uint32_t threshold_update_interval_ms;
    int upper_utilization_level_percent;
    int lower_utilization_level_percent;
    bool squelch_auto;
} configuration;

/**
 * Returns the current wall time with (up to) nanosecond precision
 * @return time in seconds
 */
double get_time_double()
{
    // copied from https://stackoverflow.com/a/49360677
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return now.tv_sec + now.tv_nsec*1e-9;
}

static int handler(void* conf_struct, const char* section, const char* name, const char* value)
/*
 * Ini configuration parser callback function
 *
 */
{
    configuration* pconfig = (configuration*) conf_struct;

    #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (MATCH("hw", "num_ch"))
    {
        pconfig->num_ch = atoi(value);
    }
    else if (MATCH("pre_processing", "cpi_size"))
    {
        pconfig->cpi_size = atoi(value);
    }
    else if (MATCH("squelch", "amplitude_threshold"))
    {
        pconfig->squelch_threshold = atof(value);
    }
    else if (MATCH("daq", "log_level"))
    {
        pconfig->log_level = atoi(value);
        pconfig->log_level = LOG_DEBUG;
        log_warn("Overriding log level to Debug!");
    }
    else if (MATCH("squelch", "udp_addr"))
    {
        pconfig->udp_addr = strdup(value);
    }
    else if (MATCH("squelch", "udp_port"))
    {
        pconfig->udp_port = atoi(value);
    }
    else if (MATCH("squelch", "threshold_update_interval_ms"))
    {
        pconfig->threshold_update_interval_ms = atoi(value);
    }
    else if (MATCH("squelch", "upper_utilization_level_percent"))
    {
        pconfig->upper_utilization_level_percent = atoi(value);
    }
    else if (MATCH("squelch", "lower_utilization_level_percent"))
    {
        pconfig->lower_utilization_level_percent = atoi(value);
    }
    else if (MATCH("squelch", "squelch_auto"))
    {
        pconfig->squelch_auto = atoi(value);
    }
    else {
        return 0;  /* unknown section/name, error */
    }
    return 0;
}

void * fifo_read_tf(void* arg)
{
   /*
    *  Control FIFO read thread function
    *
    *  This thread function handles the external requests using an external FIFO file.
    *  Upon receipt of a command, this thread infroms the main thread on the requested operation.
    *
    *  The valid (1 byte) commands are the followings:
    *       't': Set threshold value
    *
    *  Return values:
    *  --------------
    *       NULL
    */
    uint8_t signal;
    int read_size;
    (void)arg;

    /* Opening control FIFO */
    FILE * fd = fopen(SQC_FNAME, "r");
    if(fd!=0)
        log_info("Control FIFO opened succesfully");
    else{
        log_fatal("Control FIFO open error");
        exit_flag = 1;
    }
    /* Main command processing loop */
    while(!exit_flag){
        read_size=fread(&signal, sizeof(signal), 1, fd);
        CHK_CMD_READ(read_size,1);

        if( (char) signal == 't')
        {
            log_info("Signal 't': Updating threshold value");
            read_size=fread(&squelch_threshold, sizeof(float), 1, fd);
            CHK_CMD_READ(read_size,1);
            control_signal=1; // Set control_signal for updating threshold values
            sem_wait(&config_sem);
        }
    }
    fclose(fd);
    return NULL;
}


int check_trigger(struct iq_frame_struct_32* iq_frame, configuration * pconfig)
/*
 *  Trigger condition is met, when the amplitude of the input signal
 *  rises above a threshold level. This very simple trigger function
 *  can be used to catch pulsed signals.
 *
 *  Implementation notes:
 *  ---------------------
 *
 *  The trigger condition is checked only on the first antenna chennel.
 *
 *  Parameters:
 *  -----------
 *  iq_frame           : iq_frame structure whoese payload is checked
 *  [global]scan_start : Scanning of trigger condition will be started
 *                       from this sample position.
 *                       (Must be the multiple of two, and less than
 *                        2 x CPI size)
 *  [global] min_th    : Minimum threshold level
 *  [global] max_th    : Maximum threshold level
 *
 *  Return values:
 *  --------------
 *      -1: In case the trigger condition is not met.
 *       n: In case the trigger condition is met, n represents the
 *          start sample index of the captured burst.
 *
 */
{
    if(min_th == max_th)
    {
        return 0;
    }
#ifdef POWER
    static double last_check = 0;
    static double noise_avg = 0;
    static float sq_th = 0.05f; // only initially, will be updated by auto squelch!
    // Recalculate power levels if necessary:
    if(pconfig->squelch_auto)  // only if enabled
        if((get_time_double() - last_check) > CHECK_INTERVAL_MS / 1000 // and enough time has passed
            && (triggered_samples_count + nontriggered_samples_count) > 25 // and enough packets were processed
            )
        {   // only then update the automatic squelch
            last_check = get_time_double();
            float trigger_ratio =
                    (float) triggered_samples_count / (float) (triggered_samples_count + nontriggered_samples_count);
            log_info("Updating trigger. %d values over threshold, %d under threshold (%f %%), current threshold %f",
                     triggered_samples_count, nontriggered_samples_count, trigger_ratio * 100, sq_th);
            // get how many triggers we had the last interval. Reset counter.

            triggered_samples_count = 0;
            nontriggered_samples_count = 0;
            if (trigger_ratio < 0.01) {
                // trigger ratio very low, decrease threshold significantly:
                sq_th /= 2;
                log_info("Trigger ratio is very low at %f, setting threshold to %f", trigger_ratio, sq_th);
            } else if (trigger_ratio < 0.5) {
                // trigger ratio seems reasonable, update using noise_avg if the deviation from the current value is not too big:
                float candidate = 1.3f * noise_avg;
                /*if (candidate < 0.8 * sq_th) {
                    sq_th = 0.8 * sq_th;
                    log_info("Trigger ratio is %f, setting threshold to %f (80%)", trigger_ratio, sq_th);
                }
                else if (candidate > 1.2 * sq_th) {
                    sq_th = 1.2 * sq_th;
                    log_info("Trigger ratio is %f, setting threshold to %f (120%)", trigger_ratio, sq_th);
                }
                else {*/
                sq_th = candidate;
                log_info("Trigger ratio is %f, setting threshold to %f (1.3*noise_avg)", trigger_ratio, sq_th);
                /*
                }
                 */
            } else {
                // Threshold way to low, increase significantly:
                sq_th = 1.5 * sq_th;
                log_info("Trigger ratio is very high at %f, setting threshold to %f", trigger_ratio, sq_th);

            }
        }
    else  // if no automatic squelch is enabled, get value from manually set max_th
    {
        sq_th = max_th;
    }

    // calculate signal power and trigger if power level is bigger than threshold
    double acc = 0;
    int samples_over_thr_count = 0;
    const int block_len = iq_frame->header->cpi_length; // TODO: block length <= CPI length, also: sub-packet squelch [Task 2]
    log_debug("Received %d samples per channel", block_len);
    for (int i = 0; i < block_len; i++) {
      // Fast approximation of the magnitude of this sample
      float mag = (fabsf(iq_frame->payload[i*2]  ) +
                      fabsf(iq_frame->payload[i*2+1]));
      if (mag > sq_th) {
        samples_over_thr_count++;
      }
      acc += mag;
    }

    float trigger_ratio = (float) samples_over_thr_count / (float)block_len;
    // Did this block have enough samples over the threshold? Arbitrarily set to 50% of block length
    if (trigger_ratio > 20.0f/100.0f) {
        // Triggered!
        triggered_samples_count++;
      log_debug("Output triggered. %d samples (%f %%) over threshold (%f, %f dB). Average magnitude: %f, long term average: %f",
                samples_over_thr_count, trigger_ratio * 100, sq_th, 10 * log10f(sq_th), acc / block_len, noise_avg);

      // Write the previous block, if configured
      /* todo: reenable previous/following block squelch [Task 2]
      if (options.padding_blocks) {
        if (!triggered) {
          fwrite(data == data_a?data_b:data_a, sizeof(uint8_t)*2,
                 options.block_size, options.output_file);
        }
      }*/

      // Even though the noise_avg should only contain information about background nosie, change with triggered values
      // too, in case the trigger ratio is way off. However, limit influence by setting a low reduction factor.
        if (pconfig->squelch_auto) {
            // implement a moving average where previous values are slowly purged.
            const float reduction_factor = 0.3f;
            noise_avg = noise_avg + reduction_factor * acc / (double) block_len;
            noise_avg /= (1 + reduction_factor);
        }

      // TODO: Sub-block squelch
      return 0; // mark entire block as data!

    } else { // Block was not over the threshold
        nontriggered_samples_count++;
        log_debug("Block not triggered. %d samples (%f %%) over threshold (%f, %f dB). Average magnitude: %f, long term average: %f",
                  samples_over_thr_count, trigger_ratio * 100, sq_th, 10 * log10f(sq_th), acc / block_len, noise_avg);

      // Write the block following the event, if configured
      /* Todo Task 2
      if (triggered) {
        if (options.padding_blocks) {
          fwrite(data, sizeof(uint8_t)*2, options.block_size, options.output_file);
        }
      }
       */

      // We try to only include blocks below the threshold in the average
      // to understand the background noise level

      if (pconfig->squelch_auto) {
          // implement a moving average where previous values are slowly purged.
          const float avg_factor = 0.8f;
          noise_avg = avg_factor * noise_avg + acc / (double) block_len;
          noise_avg /= (1 + avg_factor);
      }

      // triggered = false; TODO: Task 2
    }
#endif
#ifdef NAIVE
    // naive algorithm: trigger if one sample > threshold
    for(int n=scan_start;n<iq_frame->header->cpi_length*2;n+=2)
    {
        if( (iq_frame->payload[n] < min_th) | (iq_frame->payload[n] > max_th))
        {
            return n;
        }
        if( (iq_frame->payload[n+1] < min_th) | (iq_frame->payload[n+1] > max_th))
        {
            return n;
        }
    }
#endif
    scan_start = iq_frame->header->cpi_length*2;
    return -1;
}

void update_threshold_values()
/*
 *   Updates the minimum and maximum threshold values
 *
 *   Parameters:
 *   -----------
 *   [global] squelch_threshold: float value between 1..0
 */
{
    log_info("Updating threshold level: %f",squelch_threshold);
    min_th = -squelch_threshold;
    max_th = squelch_threshold;
}
void move_iq_frame_payload(struct iq_frame_struct_32* iq_frame_in, struct iq_frame_struct_32* iq_frame_out, int offset)
/*
 *  Copies the burst portion of the data from the input shared memory region to the output shared memory region.
 *
 *  Parameters:
 *  -----------
 *  iq_frame_in        : Input iq_frame structure - data will be copied from this frame
 *  iq_frame_out       : Output iq_frame structure - data will be copied into this frame
 *  [global]scan_start : Scanning of trigger condition will be started
 *                       from this sample position.
 *                       (Must be the multiple of two, and less than
 *                        2 x CPI size)
 *  [global] min_th    : Minimum threshold level
 *  [global] max_th    : Maximum threshold level
 *
 *  Notes:
 *  ------
 *      - This function updates the [global] scan_start position variable that controls the trigger start position
 *
 *
 */
{
    int M = iq_frame_in->header->active_ant_chs;
    int N = iq_frame_in->header->cpi_length;

    int copy_size = 0; // Number of samples to copy per channel 2 -> 1 IQ sample
    if (iq_frame_out->payload_size)
         copy_size = (N*2-iq_frame_out->payload_size/M);
    else copy_size = (N*2-offset);

    // Copy payload from the specified sample offset
    for(int m=0; m<M; m++)
    {
        memcpy(iq_frame_out->payload+m*N*2+(iq_frame_out->payload_size/M), iq_frame_in->payload+m*2*N+offset, copy_size*sizeof(*iq_frame_in->payload));
    }
    iq_frame_out->payload_size+=copy_size*M;
    scan_start = offset+copy_size;
}
int main(int argc, char* argv[])
/*
 *
 * Parameters:
 * -----------
 * argv[1]: Drop mode [int]
 *
 */
{
    log_set_level(LOG_TRACE);
    configuration config;

    /* Initializing */
    int ch_num;
    uint32_t adc_overdrive_flags = 0; // Used to accumulate the overdrive flags in a CPI
    int trigger_sample_offset    = -1;
    bool triggered               = 0;
    int succ                     = 0; // Used in general to check the success of an operation
    bool drop_mode               = true;
    bool read_new_frame          = true; // Internally used state variable
    uint32_t cpi_tracker         = -1; // Tracks the CPI of the last transmitted frame


    /* Set drop mode from the command prompt*/
    if (argc == 2){drop_mode = atoi(argv[1]);}

    /* Set parameters from the config file*/
    if (ini_parse(INI_FNAME, handler, &config) < 0) {
        log_fatal("Configuration could not be loaded, exiting ..");
        return -1;
    }
    ch_num = config.num_ch;
    squelch_threshold = config.squelch_threshold;
    log_set_level(config.log_level);

    log_info("Config succesfully loaded from %s",INI_FNAME);
    log_info("Channel number: %d", ch_num);
    log_info("Squelch threshold: %f ", squelch_threshold);

    netconf_t netconf;
    open_socket(&netconf, config.udp_addr, config.udp_port, "");

    /* Prepare input and output IQ frames */
    struct iq_frame_struct_32 * iq_frame_out = calloc(1, sizeof(struct iq_frame_struct_32));
    struct iq_frame_struct_32 * iq_frame_in  = calloc(1, sizeof(struct iq_frame_struct_32));

    /* Initializing input shared memory interface */
    struct shmem_transfer_struct* input_sm_buff = calloc(1, sizeof(struct shmem_transfer_struct));
    input_sm_buff->shared_memory_size = MAX_IQFRAME_PAYLOAD_SIZE*config.num_ch*4*2+IQ_HEADER_LENGTH;
    input_sm_buff->io_type = 1; // Input type
    strcpy(input_sm_buff->shared_memory_names[0], DECIMATOR_OUT_SM_NAME_A);
    strcpy(input_sm_buff->shared_memory_names[1], DECIMATOR_OUT_SM_NAME_B);
    strcpy(input_sm_buff->fw_ctr_fifo_name, DECIMATOR_OUT_FW_FIFO);
    strcpy(input_sm_buff->bw_ctr_fifo_name, DECIMATOR_OUT_BW_FIFO);
    int active_buff_ind_in = -1;

	succ= init_in_sm_buffer(input_sm_buff);
    if (succ !=0) {FATAL_ERR("Failed to init shared memory interface, exiting.")}
	else{log_info("Input shared memory interface succesfully initialized");}

    /* Initializing output shared memory interface */
    struct shmem_transfer_struct* output_sm_buff = calloc(1, sizeof(struct shmem_transfer_struct));
    output_sm_buff->shared_memory_size = MAX_IQFRAME_PAYLOAD_SIZE*ch_num*sizeof(uint32_t)*2+IQ_HEADER_LENGTH;
    output_sm_buff->io_type = 0; // Output type
    output_sm_buff->drop_mode = drop_mode;
    strcpy(output_sm_buff->shared_memory_names[0], SQUELCH_OUT_SM_NAME_A);
    strcpy(output_sm_buff->shared_memory_names[1], SQUELCH_OUT_SM_NAME_B);
    strcpy(output_sm_buff->fw_ctr_fifo_name, SQUELCH_OUT_FW_FIFO);
    strcpy(output_sm_buff->bw_ctr_fifo_name, SQUELCH_OUT_BW_FIFO);
    int active_buff_ind_out = -1;

    succ = init_out_sm_buffer(output_sm_buff);
    if(succ !=0){FATAL_ERR("Shared memory initialization failed, exiting.")}
    else{log_info("Output shared memory interface succesfully initialized");}

    /* Configuring squelch parameters */
    update_threshold_values();

    /* Initializing signal thread */
    sem_init(&config_sem, 0, 0);  // Semaphore is unlocked

    pthread_create(&fifo_read_thread, NULL, fifo_read_tf, NULL);
    /*
     *
     * ---> Main Processing Loop <---
     *
     */
    while(!exit_flag)
    {
        /*Acquire buffer from the sink block*/
        active_buff_ind_out = wait_buff_free(output_sm_buff);
        /* Implementation note: In case the system had to wait for the output buffer and thus it was not able to hand over the
        the input data frame in time, backpressure is applied automatically and the preeciding block drops the frame */
        if ((active_buff_ind_out == 0) | (active_buff_ind_out == 1))
        {
            iq_frame_out->header = (struct iq_header_struct*) output_sm_buff->shm_ptr[active_buff_ind_out];
            iq_frame_out->payload = ((float *) output_sm_buff->shm_ptr[active_buff_ind_out])+ IQ_HEADER_LENGTH/sizeof(float);
            iq_frame_out->payload_size = 0;
            read_new_frame = true;

            while (read_new_frame)
            {
                /*
                *------------------
                *  IQ Frame Reading
                *------------------
                */
                if (scan_start == 0)
                {
                    /* Acquire data buffer on the shared memory interface */
                    active_buff_ind_in = wait_buff_ready(input_sm_buff);
                    if (active_buff_ind_in == 255){exit_flag = active_buff_ind_in; break;}
                    iq_frame_in->header = (struct iq_header_struct*) input_sm_buff->shm_ptr[active_buff_ind_in];
                    iq_frame_in->payload = ((float *) input_sm_buff->shm_ptr[active_buff_ind_in] )+ IQ_HEADER_LENGTH/sizeof(float);
                    iq_frame_in->payload_size=iq_frame_in->header->cpi_length*2 * iq_frame_in->header->active_ant_chs;
                    CHK_SYNC_WORD(check_sync_word(iq_frame_in->header));
                    log_trace("--> Frame received: type: %d, daq ind:[%d]",iq_frame_in->header->frame_type, iq_frame_in->header->daq_block_index);
                }

                /*
                *---------------------------------------------------
                *  Data handling depending on the current frame type
                *---------------------------------------------------
                */
                switch(iq_frame_in->header->frame_type)
                {
                    case FRAME_TYPE_DATA:
                    {
                        if (triggered == 0)
                        {
                            trigger_sample_offset = check_trigger(iq_frame_in, &config);
                            if (trigger_sample_offset >= 0)
                            {
                                log_debug("Triggered, sample offset: %d, [%d]",trigger_sample_offset, iq_frame_in->header->daq_block_index);
                                triggered = 1;
                                adc_overdrive_flags |= iq_frame_in->header->adc_overdrive_flags; // Accumulate ADC overdrive flags
                                move_iq_frame_payload(iq_frame_in, iq_frame_out, trigger_sample_offset);
                            }
                        }
                        else move_iq_frame_payload(iq_frame_in, iq_frame_out, 0);
                    }
                    break;
                    case FRAME_TYPE_DUMMY:
                    {
                        adc_overdrive_flags = 0; // Reset ADC overdrive flags
                        triggered = 0; // Reset trigger condition
                        trigger_sample_offset=0;
                    }
                    break;
                    case FRAME_TYPE_CAL:
                    {
                        move_iq_frame_payload(iq_frame_in, iq_frame_out, 0);
                    }
                    break;
                    default:
                        log_error("Unidentified frame type, forward header only..");
                    break;
                }
                /*
                *------------------
                *  IQ Frame Writing
                *------------------
                */
                if (cpi_tracker != iq_frame_in->header->cpi_index) // We have not sent any frame for this CPI
                {
                    memcpy(iq_frame_out->header, iq_frame_in->header,IQ_HEADER_LENGTH); // Copy header

                    if(iq_frame_in->header->frame_type==FRAME_TYPE_DUMMY)
                    {
                        read_new_frame = false;
                        scan_start = iq_frame_in->header->cpi_length*2;
                    }
                    else if(iq_frame_out->payload_size == iq_frame_in->payload_size) // DATA or CAL frames
                    {
                        iq_frame_out->header->adc_overdrive_flags = adc_overdrive_flags;
                        adc_overdrive_flags = 0;
                        triggered     = 0;
                        trigger_sample_offset = -1;
                        read_new_frame = false;
                    }
                    else if (triggered == 0)// TrigW frame will be generated
                    {
                        iq_frame_out->header->frame_type = FRAME_TYPE_TRIGW;
                        iq_frame_out->header->cpi_length = 0;
                        read_new_frame = false;
                    }
                }

                // There are no more samples left in the frame that could belong to an unprocessed bust
                if(scan_start == iq_frame_in->header->cpi_length*2)
                {
                    scan_start = 0;
                    send_ctr_buff_free(input_sm_buff, active_buff_ind_in);
                }

            }
            if (!exit_flag)
            {
                log_trace("<--Transfering frame type: %d, daq ind:[%d]",iq_frame_out->header->frame_type, iq_frame_out->header->daq_block_index);
                cpi_tracker = iq_frame_out->header->cpi_index;
                send_ctr_buff_ready(output_sm_buff, active_buff_ind_out);

                send_data(&netconf, iq_frame_out->payload, iq_frame_out->header->cpi_length, 2 * sizeof(float));
            }

        }
        /* Update threshold value*/
        if(control_signal == 1)
        {
            update_threshold_values();
            control_signal = 0;
            sem_post(&config_sem);
        }
    } // End of the main processing loop

    error_code_log(exit_flag);
    /* Free up resources */
    log_info("Send terminate and wait..");
    send_ctr_terminate(output_sm_buff);
    sleep(3);
    destory_sm_buffer(output_sm_buff);
    destory_sm_buffer(input_sm_buff);
    free(iq_frame_in);
    free(iq_frame_out);

    log_info("Squelch module exited");
    return 0;
}


