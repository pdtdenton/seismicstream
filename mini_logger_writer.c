/***************************************************************************
 * mini_logger_writer.c
 *
 * Read serial data from a serial port and write 512-byte Mini-SEED data records in mseed files.
 *
 * Written by Anthony Lomax
 *   ALomax Scientific www.alomax.net
 *
 * created: 2013.10.28
 ***************************************************************************/



#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
//#include <sys/resource.h>
#include <errno.h>

#ifndef WIN32
#include <signal.h>
static void term_handler(int sig);
#endif

#include "libmseed.h"
#include "mini_logger_writer.h"
#include "minisepdevice.h"
#include "settings/settings.h"

#define DEBUG 0

#define STANDARD_STRLEN 4096


#ifdef EXTERN_MODE
#define	EXTERN_TXT extern
#else
#define EXTERN_TXT
#endif

EXTERN_TXT int verbose;
EXTERN_TXT char propfile[STANDARD_STRLEN];

// properties
// Logging
EXTERN_TXT char port_path_hint[STANDARD_STRLEN];
#define PORT_PATH_HINT_DEFAULT "/dev/ttyACM0"   // rasberry pi
//#define PORT_PATH_HINT_DEFAULT "/dev/usbdev1.1" // iMac
EXTERN_TXT int allow_set_interface_attribs;
#define ALLOW_SET_INTERFACE_ATTRIBS_DEFAULT 0
EXTERN_TXT int writing_enabled;
#define LOGGING_ENABLED_DEFAULT 1
EXTERN_TXT char mswrite_dir[STANDARD_STRLEN];
#define LOG_DIR_DEFAULT "./data_current"
EXTERN_TXT int num_records_in_file;
#define NUM_REC_IN_FILE_DEFAULT 10
EXTERN_TXT double mswrite_buffer_size;
#define LOG_BUFFER_SIZE_DEFAULT 6

EXTERN_TXT double mswrite_header_sample_rate;
#define MSWRITE_HEADER_SAMPLE_RATE_DEFAULT -1
EXTERN_TXT char mswrite_data_encoding_type[STANDARD_STRLEN];
#define MSWRITE_DATA_ENCODING_TYPE_DEFAULT "DE_INT32"
EXTERN_TXT int mswrite_data_encoding_type_code;

EXTERN_TXT char mswrite_script[STANDARD_STRLEN];
#define MSWRITE_SCRIPT_NONE "NONE"
EXTERN_TXT char msremove_script[STANDARD_STRLEN];
#define MSREMOVE_SCRIPT_NONE "NONE"
// Station
EXTERN_TXT char station_network[STANDARD_STRLEN];
#define STA_NETWORK_DEFAULT "UK"
EXTERN_TXT char station_name[STANDARD_STRLEN];
#define STA_NAME_DEFAULT "TEST"
EXTERN_TXT char channel_prefix[STANDARD_STRLEN];
#define STA_CHANNEL_PREFIX_DEFAULT "BH"
EXTERN_TXT char component[STANDARD_STRLEN];
#define STA_COMPONENT_DEFAULT "Z"
//
EXTERN_TXT int nominal_sample_rate;
#define STA_NOMINAL_SAMPLE_RATE_DEFAULT 20
//Nominal sample rate in seconds, one of 20, 40 or 80. type=string default=20
//   For SEISMOMETER INTERFACE (USB) (CODE: SEP 064):
// 	The SPS can be adjusted by sending single characters to the Virtual Com Port:
//	‘a’: 20.032 SPS
//	‘b’: 39.860 SPS
//	‘c’: 79.719 SPS
//
EXTERN_TXT int nominal_gain;
#define STA_NOMINAL_GAIN_DEFAULT 1
//Nominal gain, one of 1, 2 or 4. type=string default=1
//   For SEISMOMETER INTERFACE (USB) (CODE: SEP 064):
// 	The gain can be adjusted by sending single characters to the Virtual Com Port:
//	‘1’: ×1 = 0.64μV/count
//	‘2’: ×2 = 0.32μV/count
//	‘4’: ×2 = 0.32μV/count
//
EXTERN_TXT int do_settings_sep064;
#define DO_SETTINGS_SEP064_DEFAULT 0


#define PROP_FILE_NAME_DEFAULT "mini_logger_writer.prop"
static Settings *settings = NULL;

#define STATE_FILE_NAME "mini_logger_writer.state"
static int file_index_modulo;
static int file_index = 0;
static int state_file_index = 0;
static hptime_t* filetime_list;

static char* port_path;

#define SLRECSIZE 512   // Mini-SEED record size
#define SLREC_DATA_SIZE 456     // apparent size of time-seris data in a 512 byte mseed record written by libmseed
//static int num_samples_in_record = SLRECSIZE / sizeof (int);
static int num_samples_in_record = -1;

#define MSFILEPATH_WRITE_TMP "msfile_write.tmp"


int find_device_and_connect(char *port_path_hint);
int collect_and_write();

/***************************************************************************
 * usage():
 * Print the usage message and exit.
 ***************************************************************************/
static void usage(void) {

    fprintf(stdout, "%s version: %s (%s)\n", PACKAGE, VERSION, VERSION_DATE);
    fprintf(stdout, "Usage: %s [options]\n", PACKAGE);
    fprintf(stdout,
            " Options:\n"
            " -p propfile    properties file name (default: config.prop)\n"
            " -V             Report program version\n"
            " -h             Show this usage message\n"
            " -v             Be more verbose, multiple flags can be used\n"
            );

} /* End of usage() */

/***************************************************************************
 * parameter_proc():
 * Process the command line parameters.
 *
 * Returns 0 on success, and -1 on failure
 ***************************************************************************/
static int parameter_proc(int argcount, char **argvec) {

    int optind = 1;
    int error = 0;

    if (argcount <= 1)
        error++;

    // Process all command line arguments
    for (optind = 1; optind < argcount; optind++) {

        if (strcmp(argvec[optind], "-V") == 0) {
            fprintf(stdout, "%s version: %s (%s)\n", PACKAGE, VERSION, VERSION_DATE);
            exit(0);
        } else if (strcmp(argvec[optind], "-h") == 0) {
            (*usage)();
            exit(0);
        } else if (strncmp(argvec[optind], "-v", 2) == 0) {
            verbose += strspn(&argvec[optind][1], "v");
        } else if (strcmp(argvec[optind], "-p") == 0) {
            strcpy(propfile, argvec[++(optind)]);
        } else if (strncmp(argvec[optind], "-", 1) == 0) {
            fprintf(stdout, "%s: Unknown option: %s\n", PACKAGE, argvec[optind]);
            exit(1);
        }
    }

    // Report the program version
    if (verbose) {
        logprintf(MSG_FLAG, "%s version: %s (%s)\n", PACKAGE, VERSION, VERSION_DATE);
    }

    return 0;

} /* End of parameter_proc() */

/***************************************************************************
 * init_properties():
 * Initialize properties from properties file
 ***************************************************************************/
int init_properties(char *propfile) {

    // read properties file
    FILE *fp_prop = fopen(propfile, "r");
    if (fp_prop == NULL) {
        logprintf(MSG_FLAG, "Info: Cannot open application properties file: %s\n", propfile);
        settings = NULL;
        return (0);
    }
    settings = settings_open(fp_prop);
    fclose(fp_prop);
    if (settings == NULL) {
        logprintf(ERROR_FLAG, "Reading application properties file: %s\n", propfile);
        return (-1);
    }

    //
    if (settings_get_helper(settings,
            "Logging", "port_path_hint", port_path_hint, sizeof (port_path_hint), PORT_PATH_HINT_DEFAULT,
            verbose) == 0) {
        ; // handle error
    }
    //
    if (settings_get_int_helper(settings,
            "Logging", "allow_set_interface_attribs", &allow_set_interface_attribs, ALLOW_SET_INTERFACE_ATTRIBS_DEFAULT,
            verbose) == 0) {
        ; // handle error
    }

    //
    if (settings_get_helper(settings,
            "Logging", "mswrite_dir", mswrite_dir, sizeof (mswrite_dir), LOG_DIR_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_int_helper(settings,
            "Logging", "mswrite_nrec_in_file", &num_records_in_file, NUM_REC_IN_FILE_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_int_helper(settings,
            "Logging", "writing_enabled", &writing_enabled, LOGGING_ENABLED_DEFAULT,
            verbose
            ) == INT_INVALID) {
        ; // handle error
    }
    //
    if (settings_get_double_helper(settings,
            "Logging", "mswrite_buffer_size", &mswrite_buffer_size, LOG_BUFFER_SIZE_DEFAULT,
            verbose
            ) == DBL_INVALID) {
        ; // handle error
    }
    //
    if (settings_get_double_helper(settings,
            "Logging", "mswrite_header_sample_rate", &mswrite_header_sample_rate, MSWRITE_HEADER_SAMPLE_RATE_DEFAULT,
            verbose
            ) == DBL_INVALID) {
        ; // handle error
    }
    //
    if (settings_get_helper(settings,
            "Logging", "mswrite_data_encoding_type", mswrite_data_encoding_type, sizeof (mswrite_data_encoding_type), MSWRITE_DATA_ENCODING_TYPE_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_helper(settings,
            "Logging", "mswrite_script", mswrite_script, sizeof (mswrite_script), MSWRITE_SCRIPT_NONE,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_helper(settings,
            "Logging", "msremove_script", msremove_script, sizeof (msremove_script), MSREMOVE_SCRIPT_NONE,
            verbose
            ) == 0) {
        ; // handle error
    }

    //
    if (settings_get_helper(settings,
            "Station", "station_network", station_network, sizeof (station_network), STA_NETWORK_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_helper(settings,
            "Station", "station_name", station_name, sizeof (station_name), STA_NAME_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_helper(settings,
            "Station", "channel_prefix", channel_prefix, sizeof (channel_prefix), STA_CHANNEL_PREFIX_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_helper(settings,
            "Station", "component", component, sizeof (component), STA_COMPONENT_DEFAULT,
            verbose
            ) == 0) {
        ; // handle error
    }
    //
    if (settings_get_int_helper(settings,
            "Station", "nominal_sample_rate", &nominal_sample_rate, STA_NOMINAL_SAMPLE_RATE_DEFAULT,
            verbose
            ) == DBL_INVALID) {
        ; // handle error
    }
    //
    if (settings_get_int_helper(settings,
            "Station", "nominal_gain", &nominal_gain, STA_NOMINAL_GAIN_DEFAULT,
            verbose
            ) == DBL_INVALID) {
        ; // handle error
    }
    //
    if (settings_get_int_helper(settings,
            "Station", "do_settings_sep064", &do_settings_sep064, DO_SETTINGS_SEP064_DEFAULT,
            verbose) == 0) {
        ; // handle error
    }

    return (0);

}

#ifndef WIN32

/** *************************************************************************
 * term_handler:
 * Signal handler routine.
 ************************************************************************* **/
static void term_handler(int sig) {
    disconnect(verbose);
    FILE* fp_state = fopen(STATE_FILE_NAME, "w");
    if (fp_state != NULL) {
        fprintf(fp_state, "%d ", file_index);
        int ifile = file_index;
        int count = 0;
        while (count < file_index_modulo) {
            fprintf(fp_state, "%lld ", (long long) filetime_list[ifile]);
            ifile = (ifile + 1) % file_index_modulo;
            count++;
        }
        fclose(fp_state);
        if (verbose)
            logprintf(MSG_FLAG, "State saved to: %s\n", STATE_FILE_NAME);
    }
    free(filetime_list);
    exit(0);
}
#endif

int main(int argc, char **argv) {

#ifndef WIN32
    // Signal handling, use POSIX calls with standardized semantics
    struct sigaction sa;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sa.sa_handler = term_handler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sa.sa_handler = SIG_IGN;
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);
#endif

    /*
    printf("LONG_MIN %ld\n", LONG_MIN);
    printf("LONG_MAX %ld\n", LONG_MAX);
    printf("INT_MIN %d\n", INT_MIN);
    printf("INT_MAX %d\n", INT_MAX);
     */

    // set default error message prefix
    ms_loginit(NULL, NULL, NULL, "ERROR: ");

    // defaults
    verbose = 0;
    strcpy(port_path_hint, "/dev/usbdev1.1");
    strcpy(propfile, PROP_FILE_NAME_DEFAULT);

    // Process input parameters
    if (parameter_proc(argc, argv) < 0)
        return 1;
    init_properties(propfile);

    // set encoding type
    // possible: DE_ASCII, DE_INT16, DE_INT32, DE_FLOAT32, DE_FLOAT64, DE_STEIM1, DE_STEIM2
    // supported: DE_INT16, DE_INT32, DE_STEIM1, DE_STEIM2
    if (strcmp(mswrite_data_encoding_type, "DE_INT16") == 0) {
        mswrite_data_encoding_type_code = DE_INT16;
        num_samples_in_record = (SLREC_DATA_SIZE) / 2;
    } else if (strcmp(mswrite_data_encoding_type, "DE_INT32") == 0) {
        mswrite_data_encoding_type_code = DE_INT32;
        num_samples_in_record = (SLREC_DATA_SIZE) / 4;
        /*
    } else if (strcmp(mswrite_data_encoding_type, "DE_ASCII") == 0) {
        mswrite_data_encoding_type_code = DE_ASCII;
    } else if (strcmp(mswrite_data_encoding_type, "DE_FLOAT32") == 0) {
        mswrite_data_encoding_type_code = DE_FLOAT32;
    } else if (strcmp(mswrite_data_encoding_type, "DE_FLOAT64") == 0) {
        mswrite_data_encoding_type_code = DE_FLOAT64;
         */
    } else if (strcmp(mswrite_data_encoding_type, "DE_STEIM1") == 0) {
        mswrite_data_encoding_type_code = DE_STEIM1;
        num_samples_in_record = (SLREC_DATA_SIZE) / 4; // estimate, inefficient, assumes int32 data
    } else if (strcmp(mswrite_data_encoding_type, "DE_STEIM2") == 0) {
        mswrite_data_encoding_type_code = DE_STEIM2;
        num_samples_in_record = (SLREC_DATA_SIZE) / 4; // estimate, inefficient, assumes int32 data
    }

    // set file index max/modulo based on mswrite_buffer_size
    file_index_modulo = (int) (mswrite_buffer_size * 3600.0 * nominal_sample_rate / (double) (num_samples_in_record * num_records_in_file));
    if (verbose)
        logprintf(MSG_FLAG, "File buffer: size=%d files, duration=%lf hours\n", file_index_modulo, mswrite_buffer_size);
    // allocate list of used filename times (initialized to zero)
    filetime_list = calloc(file_index_modulo, sizeof (hptime_t));
    int n;
    for (n = 0; n < file_index_modulo; n++) {
        filetime_list[n] = 0;
    }

    // set file index and filetimes based on saved state index and filetimes
    file_index = 0;
    FILE* fp_state = fopen(STATE_FILE_NAME, "r");
    if (fp_state != NULL) {
        fscanf(fp_state, "%d", &state_file_index);
        if (state_file_index >= 0 && state_file_index < file_index_modulo) {
            file_index = state_file_index;
            int ifile = file_index;
            long long filetime;
            while (fscanf(fp_state, "%lld", &filetime) == 1) {
                filetime_list[ifile] = (hptime_t) filetime;
                ifile = (ifile + 1) % file_index_modulo;
            }
        }
        fclose(fp_state);
    }
    //if (verbose)
    //    logprintf(MSG_FLAG, "File buffer: starting file index=%d\n", file_index);

    // enter infinite loop, term_handler() performs cleanup
    while (1) {

        // find device and connect
        find_device_and_connect(port_path_hint);

        // set sample rate and gain for SEP064
        if (do_settings_sep064) {
            if (set_seo064_sample_rate_and_gain(nominal_sample_rate, nominal_gain, verbose, TIMEOUT_SMALL)) {
                continue;
            }
        }

        // collect data and write
        if (collect_and_write()) { // collect_and_write() returned error
            logprintf(ERROR_FLAG, "Reading from %s, will try reconnecting...\n", port_path);
            disconnect(verbose);
        } else {
            break; // collect_and_write() returned normally
        }

    }

    return (0);

} /* End of main() */

/***************************************************************************
 * find_device_and_connect:
 *
 * Attempt to connect to a device, slows down the loop checking
 * after 20 attempts with a larger delay to reduce pointless
 * work being done.
 *
 * Returns 0 on success and -1 otherwise.
 ***************************************************************************/
int find_device_and_connect(char *port_path_hint) {

    int counter = 0;
    int slow_mode = 0;
    if (verbose)
        logprintf(MSG_FLAG, "Searching for device...\n");
    if (!find_device(port_path_hint, verbose, &port_path, allow_set_interface_attribs)) {
        if (verbose)
            while (!find_device(port_path_hint, verbose, &port_path, allow_set_interface_attribs)) {
                //logprintf(MSG_FLAG, "Still searching for device...\n");
                logprintf(ERROR_FLAG, "Still searching for device.   Try reconnecting (unplug and plug in) the USB Seismometer Interface device.\n");
                if (counter < 20) {
                    counter++;
                    sleep(5);
                } else if (!slow_mode) {
                    if (verbose)
                        logprintf(MSG_FLAG, "Entering slow mode after 20 attempts.\n");
                    slow_mode = 1;
                    sleep(30);
                } else {
                    sleep(30);
                }
            }
    }
    if (verbose)
        logprintf(MSG_FLAG, "Device connected successfully: %s\n", port_path);

    return (0);

}

/***************************************************************************
 * hptime2timestr:
 *
 * Build a time string in filename format from a high precision
 * epoch time.
 *
 * The provided isostimestr must have enough room for the resulting time
 * string of 24 characters, i.e. '2001.07.29.12.38.00.000' + NULL.
 *
 * The 'subseconds' flag controls whether the sub second portion of the
 * time is included or not.
 *
 * Returns a pointer to the resulting string or NULL on error.
 ***************************************************************************/
char *
hptime2timestr(hptime_t hptime, char *timestr, flag subseconds, char* datepath) {
    struct tm tms;
    time_t isec;
    int ifract;
    int ret;

    if (timestr == NULL)
        return NULL;

    /* Reduce to Unix/POSIX epoch time and fractional seconds */
    isec = MS_HPTIME2EPOCH(hptime);
    ifract = (int) (hptime - (isec * HPTMODULUS));

    /* Adjust for negative epoch times */
    if (hptime < 0 && ifract != 0) {
        isec -= 1;
        ifract = HPTMODULUS - (-ifract);
    }

    if (!(gmtime_r(&isec, &tms)))
        return NULL;

    if (subseconds) {
        ifract /= (HPTMODULUS / 1000); // tp milliseconds (assumes HPTMODULUS mulitple of 1000)
        /* Assuming ifract has millisecond precision */
        ret = snprintf(timestr, 24, "%4d.%02d.%02d.%02d.%02d.%02d.%03d",
                tms.tm_year + 1900, tms.tm_mon + 1, tms.tm_mday,
                tms.tm_hour, tms.tm_min, tms.tm_sec, ifract);
    } else {
        ret = snprintf(timestr, 20, "%4d.%02d.%02d.%02d.%02d.%02d",
                tms.tm_year + 1900, tms.tm_mon + 1, tms.tm_mday,
                tms.tm_hour, tms.tm_min, tms.tm_sec);
    }
    //printf("DEBUG: timestr= %s\n", timestr);

    if (ret != 23 && ret != 19)
        return NULL;

    ret = snprintf(datepath, 17, "%4d/%02d/%02d/%02d/%02d",
            tms.tm_year + 1900, tms.tm_mon + 1, tms.tm_mday,
            tms.tm_hour, tms.tm_min);

    if (ret != 16)
        return NULL;

    return timestr;

} /* End of ms_hptime2isotimestr() */



static char sys_command[16384] = "\0";

/***************************************************************************
 * run external script for case when miniseed file written:
 *
 ***************************************************************************/
void run_mswrite_script(char *msfilepath_write, char *msfilepath_remove) {

    if (strcmp(mswrite_script, MSWRITE_SCRIPT_NONE) != 0) {
        //char* filepath = msfilepath_write + strlen(mswrite_dir);
        //if (strlen(mswrite_dir) > 0)
        //    filepath++;    // skip leading '/'
        sprintf(sys_command, "%s %s %s &", mswrite_script, msfilepath_write, msfilepath_remove);
        if (verbose > 1)
            logprintf(MSG_FLAG, "Running command: %s\n", sys_command);
        int process_status = system(sys_command);
        if (verbose > 2)
            logprintf(MSG_FLAG, "Return %d from running command: %s\n", process_status, sys_command);
    }

}

/***************************************************************************
 * run external script for case when miniseed file or directory removed:
 *
 ***************************************************************************/
void run_msremove_script(char *msfilepath) {

    if (strcmp(msremove_script, MSREMOVE_SCRIPT_NONE) != 0) {
        //char* filepath = msfilepath + strlen(mswrite_dir);
        //if (strlen(mswrite_dir) > 0)
        //    filepath++;    // skip leading '/'
        sprintf(sys_command, "%s %s &", msremove_script, msfilepath);
        if (verbose > 1)
            logprintf(MSG_FLAG, "Running command: %s\n", sys_command);
        int process_status = system(sys_command);
        if (verbose > 2)
            logprintf(MSG_FLAG, "Return %d from running command: %s\n", process_status, sys_command);
    }

}



#define DEBUG_FILEPATH 0

/** construct miniseed path and filename, create required sub-directories
 */
char* make_mseed_filepath(MSRecord *pmsrecord, char* base_dir, hptime_t filetime, char* msfilepath, int iremove) {

    static char pathfrag[STANDARD_STRLEN];
#ifdef __APPLE__
    static char pathfrag2[STANDARD_STRLEN];
#endif
    static char datepath[64];
    static char timestr[64];

    // construct filename and date path
    hptime2timestr(filetime, timestr, 1, datepath);
    sprintf(msfilepath, "%s/%s/%s.%s.%s.%s.%s.mseed", base_dir, datepath,
            pmsrecord->network, pmsrecord->station, pmsrecord->location, pmsrecord->channel,
            timestr);

    // create date path if requested and needed
    if (!iremove) {
        int base_dir_len = strlen(base_dir);
        int len = base_dir_len + strlen(datepath) + 1;
        strncpy(pathfrag, msfilepath, len); // path to msfile
        pathfrag[len] = '\0';
        if (DEBUG_FILEPATH) logprintf(MSG_FLAG, ">>> pathfrag: %s\n", pathfrag);
        struct stat statstruct;
        int ireturn = stat(pathfrag, &statstruct);
        if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "stat ireturn: %d (-1)\n", ireturn);
        if (ireturn == -1) {
            if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "errno: %d, ENOENT: %d\n", errno, ENOENT);
            if (errno == ENOENT) {
                // does not exist
                len = base_dir_len;
                len += 5;
                strncpy(pathfrag, msfilepath, len); // year path
                pathfrag[len] = '\0';
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag: %s\n", pathfrag);
                ireturn = mkdir(pathfrag, 0755);
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "mkdir ireturn: %d\n", ireturn);
                //
                len += 3;
                strncpy(pathfrag, msfilepath, len); // month path
                pathfrag[len] = '\0';
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag: %s\n", pathfrag);
                ireturn = mkdir(pathfrag, 0755);
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "mkdir ireturn: %d\n", ireturn);
                //
                len += 3;
                strncpy(pathfrag, msfilepath, len); // day path
                pathfrag[len] = '\0';
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag: %s\n", pathfrag);
                ireturn = mkdir(pathfrag, 0755);
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "mkdir ireturn: %d\n", ireturn);
                //
                len += 3;
                strncpy(pathfrag, msfilepath, len); // hour path
                pathfrag[len] = '\0';
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag: %s\n", pathfrag);
                ireturn = mkdir(pathfrag, 0755);
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "mkdir ireturn: %d\n", ireturn);
                if (DEBUG_FILEPATH && ireturn == 0)
                    logprintf(MSG_FLAG, "File buffer: created directory: %s\n", pathfrag);
                //
                len += 3;
                strncpy(pathfrag, msfilepath, len); // min path
                pathfrag[len] = '\0';
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag: %s\n", pathfrag);
                ireturn = mkdir(pathfrag, 0755);
                if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "mkdir ireturn: %d\n", ireturn);
                if ((DEBUG_FILEPATH || verbose) && ireturn == 0)
                    logprintf(MSG_FLAG, "File buffer: created directory: %s\n", pathfrag);
            }
        }
    }// remove date path if requested and possible (path empty)
    else if (iremove) {
        int ireturn = remove(msfilepath);
        if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "remove %s: ireturn: %d : %s\n", msfilepath, ireturn, ireturn == 0 ? "" : strerror(errno));
        int base_dir_len = strlen(base_dir);
        int len;
        // remove special files
        len = base_dir_len + 17;
        int icount;
        for (icount = 0; icount < 5; icount++) {
            strncpy(pathfrag, msfilepath, len); // complete to: min, hour, day, month, year path
            pathfrag[len] = '\0';
            if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag: %s\n", pathfrag);
#ifdef __APPLE__
            strcpy(pathfrag2, pathfrag);
            strcat(pathfrag2, "/.DS_Store");
            if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "pathfrag2: %s\n", pathfrag2);
            ireturn = remove(pathfrag2);
            if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "remove ireturn: %d\n", ireturn);
#endif
            ireturn = rmdir(pathfrag);
            if (DEBUG_FILEPATH) logprintf(MSG_FLAG, "rmdir ireturn: %d : %s\n", ireturn, strerror(errno));
            if (ireturn == -1)
                break;
            //run_msremove_script(pathfrag);
            len -= 3;
        }
    }

    return (msfilepath);

}


#define DOUBLE long double
//#define DOUBLE double

/***************************************************************************
 * collect_and_write:
 *
 * Attempt to connect to a device, slows down the loop checking
 * after 20 attempts with a larger delay to reduce pointless
 * work being done.
 *
 * Returns 0 on success and -1 otherwise.
 ***************************************************************************/
int collect_and_write() {

    int32_t idata[num_samples_in_record];
    hptime_t hptime;
    hptime_t start_hptime_est = 0;
    hptime_t last_hptime;
    DOUBLE dt, dt_est, sample_rate_est;
    DOUBLE start_hptime_current, record_window_current, record_window_est;
    DOUBLE prev_start_hptime_est = -1;
    int n_start_hptime_est;
    int num_records_written;

    // debug
    hptime_t start_hptime_nominal = 0;
    hptime_t prev_start_next_hptime_est = 0;
    double diff_end, diff_end_cumul = 0.0;

    char msfilepath_write[STANDARD_STRLEN];
    char msfilepath_remove[STANDARD_STRLEN];

    char seedtimestr[64];

    // decay constant depends on required decay time and sample rate
    //double decay_minutes = 60.0; // 1 hour
    double decay_minutes = 1.0;
    double decay_consant = 1.0 / (decay_minutes * 60.0 * (double) nominal_sample_rate);


    // initialize last_hptime to current time
    last_hptime = current_utc_hptime();

    // initialize dt_est based on nominal sample rate
    dt_est = (nominal_sample_rate == 80) ? 1.0 / SAMP_PER_SEC_80 : (nominal_sample_rate == 40) ? 1.0 / SAMP_PER_SEC_40 : 1.0 / SAMP_PER_SEC_20;
    //	‘a’: 20.032 SPS
    //	‘b’: 39.860 SPS
    //	‘c’: 79.719 SPS
    // initialize record_window_est based on  nominal sample rate and record length
    record_window_est = dt_est * num_samples_in_record;

    if (DEBUG) {
        logprintf(MSG_FLAG, "Initialize: last_hptime=%lld, dt_est=%lld, dt=%lf, dt_end=%lf, dt_end_cumul=%lf)\n",
                last_hptime, dt_est, record_window_est);
    }

    int first = 1;
    int is_new_file;
    num_records_written = 0;
    while (1) {

        // load data up to SLRECSIZE
        long ivalue;
        int nsamp = 0;
        start_hptime_current = 0;
        n_start_hptime_est = 0;
        while (nsamp < num_samples_in_record) {
            ivalue = read_next_value(&hptime, TIMEOUT_LARGE);
            if (ivalue == READ_ERROR || ivalue < MIN_DATA || ivalue > MAX_DATA) {
                logprintf(MSG_FLAG, "READ_ERROR: port=%s, nsamp=%d, ivalue=%ld\n", port_path, nsamp, ivalue);
                return (-1);
            }
            if (DEBUG && nsamp == 0) {
                start_hptime_nominal = hptime;
            }
            idata[nsamp] = (int32_t) ivalue;
            dt = (DOUBLE) (hptime - last_hptime) / (DOUBLE) HPTMODULUS;
            last_hptime = hptime;
            if (verbose > 3) {
                logprintf(MSG_FLAG, "%d %ld %s (dt=%lf)\n", nsamp, ivalue, ms_hptime2seedtimestr(hptime, seedtimestr, 1), (double) dt);
            }
            // estimate start time and dt
            // use only later samples in record since writing previous record may delay reading of first samples of this record
            if (nsamp >= num_samples_in_record / 2) {
                // 20131107 AJL - use all samples, may give better start time estimate, since buffering should compensate for any delay of first samples
                //if (1) {
                // start time estimate is timestamp of current data minus dt_est*nsamp
                start_hptime_current += (hptime - (hptime_t) ((DOUBLE) 0.5 + dt_est * (DOUBLE) HPTMODULUS * (DOUBLE) nsamp));
                n_start_hptime_est++;
                // accumulate dt_est using low-pass filter
                //dt_est = dt_est + (DOUBLE) decay_consant * (dt - dt_est);
            }
            nsamp++;
        }
        start_hptime_current /= n_start_hptime_est;
        if (prev_start_hptime_est > 0) {
            record_window_current = (DOUBLE) (start_hptime_current - prev_start_hptime_est) / (DOUBLE) HPTMODULUS;
        } else {
            record_window_current = record_window_est;
        }
        // accumulate record_window_est using low-pass filter
        record_window_est = record_window_est + (DOUBLE) decay_consant * (record_window_current - record_window_est);
        if (prev_start_hptime_est > 0) {
            start_hptime_est = prev_start_hptime_est + (hptime_t) ((DOUBLE) 0.5 + record_window_est * (DOUBLE) HPTMODULUS);
        } else {
            start_hptime_est = start_hptime_current;
        }
        prev_start_hptime_est = start_hptime_est;
        // test - truncate dt to 1/10000 s to match precision of miniseed btime
        //logprintf(MSG_FLAG, "0 sample_rate_est=%lf (dt=%lfs)\n", (double) ((DOUBLE) 1.0 / dt_est), (double) dt_est);
        dt_est = record_window_est / (DOUBLE) num_samples_in_record;
        sample_rate_est = (DOUBLE) 1.0 / dt_est;
        if (DEBUG) {
            diff_end = (double) (start_hptime_est - prev_start_next_hptime_est) / (double) HPTMODULUS;
            if (!first)
                diff_end_cumul += diff_end;
            logprintf(MSG_FLAG, "sample_rate_est=%lf (dt=%lfs)\n", (double) sample_rate_est, (double) dt_est);
            logprintf(MSG_FLAG, "start_hptime_est=%lld, start_hptime_nominal=%lld, dt=%lf, dt_end=%lf, dt_end_cumul=%lf)\n", start_hptime_est, start_hptime_nominal,
                    (double) ((DOUBLE) (start_hptime_est - start_hptime_nominal) / (DOUBLE) HPTMODULUS), diff_end, diff_end_cumul);
            prev_start_next_hptime_est = start_hptime_est + (hptime_t) ((DOUBLE) 0.5 + dt_est * (DOUBLE) HPTMODULUS * (DOUBLE) nsamp);
        }

        // construct mseed record
        MSRecord *pmsrecord = msr_init(NULL);
        strcpy(pmsrecord->network, station_network);
        strcpy(pmsrecord->station, station_name);
        strcpy(pmsrecord->location, "");
        sprintf(pmsrecord->channel, "%s%s", channel_prefix, component);
        pmsrecord->starttime = start_hptime_est;
        pmsrecord->samprate = mswrite_header_sample_rate > 0.0 ? mswrite_header_sample_rate : sample_rate_est;
        pmsrecord->samplecnt = nsamp;
        pmsrecord->numsamples = nsamp;
        pmsrecord->datasamples = idata;
        pmsrecord->sampletype = 'i';

        // write record to file
        is_new_file = 0;
        // if first record in file, remove previous file and set new file name
        if (num_records_written == 0) {
            // remove previous file with this file_index
            //logprintf(MSG_FLAG, "Removed mseed file: %d %lld\n", file_index, filetime_list[file_index]);
            strcpy(msfilepath_remove, "-");
            if (filetime_list[file_index] > 0) {
                make_mseed_filepath(pmsrecord, mswrite_dir, filetime_list[file_index], msfilepath_remove, 1);
                //sprintf(msfilepath_remove, MSFILENAME_FORMAT, mswrite_dir,
                //        pmsrecord->network, pmsrecord->station, pmsrecord->location, pmsrecord->channel,
                //        hptime2timestr(filetime_list[file_index], timestr, 1));
                if (verbose > 2) {
                    logprintf(MSG_FLAG, "Removed mseed file: %s\n", msfilepath_remove);
                }
                // run external remove script if requested
                run_msremove_script(msfilepath_remove);
            }
            // set file time of this record in file time list
            filetime_list[file_index] = pmsrecord->starttime;
            // set file name and write
            make_mseed_filepath(pmsrecord, mswrite_dir, pmsrecord->starttime, msfilepath_write, 0);
            //sprintf(msfilepath_write, MSFILENAME_FORMAT, mswrite_dir,
            //        pmsrecord->network, pmsrecord->station, pmsrecord->location, pmsrecord->channel,
            //        hptime2timestr(pmsrecord->starttime, timestr, 1));
            is_new_file = 1;
        }
        // write to temporary file so ringserver or mseedscan2dali do not detect partially written files
        int nrecords = msr_writemseed(pmsrecord, MSFILEPATH_WRITE_TMP, is_new_file, SLRECSIZE, mswrite_data_encoding_type_code, 1, verbose > 1);
        if (nrecords != 1) {
            logprintf(ERROR_FLAG, "Writing mseed record to file: %s: %s\n", strerror(errno), MSFILEPATH_WRITE_TMP);
        } else {
            if (verbose > 1) {
                logprintf(MSG_FLAG, "Writing mseed record to file: %s, header_sps=%f, est_sps=%f\n", MSFILEPATH_WRITE_TMP,
                        mswrite_header_sample_rate > 0.0 ? (double) mswrite_header_sample_rate : (double) sample_rate_est, (double) sample_rate_est);
            }
        }

        num_records_written++;
        // check for complete file
        if (num_records_written >= num_records_in_file) {
            // rename/move temporary file to permanent file
            //    temporary file used so ringserver or mseedscan2dali do not detect partially written files
            logprintf(MSG_FLAG, "Moving: %s to: %s\n", MSFILEPATH_WRITE_TMP, msfilepath_write);
            if (rename(MSFILEPATH_WRITE_TMP, msfilepath_write) != 0) {
                logprintf(ERROR_FLAG, "Error: %d moving temporary mseed file: %s to file: %s: %s\n", strerror(errno), MSFILEPATH_WRITE_TMP, msfilepath_write);
            }
            // run external write script if requested
            run_mswrite_script(msfilepath_write, msfilepath_remove);
            num_records_written = 0;
        }

        pmsrecord->datasamples = NULL;
        msr_free(&pmsrecord);

        file_index = (file_index + 1) % file_index_modulo;
        /*
        if (file_index == state_file_index) {
            if (verbose)
                logprintf(MSG_FLAG, "File buffer: returned to beginning of file buffer ring.\n");
        }
         */

        first = 0;

    }

    return (0);

}

