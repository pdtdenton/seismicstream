#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <libmseed.h>

#define DEVICE "/dev/ttyACM0"
#define BAUDRATE B115200
#define BUFFER_SIZE 512
#define MINI_SEED_FILE "output.mseed"
#define RECORD_LENGTH 256

// Function to configure the serial port
int configure_serial_port(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

// Function to write MiniSEED records to a file
void write_miniseed_records(MSRecord *msr) {
    FILE *fp = fopen(MINI_SEED_FILE, "ab");
    if (fp == NULL) {
        perror("Failed to open MiniSEED file");
        return;
    }

    int reclen = msr->reclen;
    uint8_t *record = (uint8_t *) malloc(reclen);
    if (record == NULL) {
        perror("Failed to allocate memory for MiniSEED record");
        fclose(fp);
        return;
    }

    msr_pack(msr, &record, &reclen, 0, 0);
    fwrite(record, 1, reclen, fp);
    free(record);
    fclose(fp);
}

int main(int argc, char *argv[]) {
    int fd = configure_serial_port(DEVICE);
    if (fd == -1) {
        return -1;
    }

    MSRecord *msr = msr_init(NULL);
    if (msr == NULL) {
        perror("Failed to initialize MiniSEED record");
        close(fd);
        return -1;
    }

    msr->reclen = RECORD_LENGTH;
    msr->samprate = 100.0; // Set the sample rate as per your requirement
    msr->encoding = DE_INT32; // Assuming 32-bit integer samples
    strncpy(msr->network, "XX", 2);
    strncpy(msr->station, "STATION", 5);
    strncpy(msr->location, "LOC", 2);
    strncpy(msr->channel, "BHZ", 3);

    int32_t samples[BUFFER_SIZE];
    int sample_count = 0;
    char buffer[BUFFER_SIZE];
    ssize_t bytes_read;

    while (1) {
        bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            for (ssize_t i = 0; i < bytes_read; i += sizeof(int32_t)) {
                if (sample_count < BUFFER_SIZE) {
                    samples[sample_count++] = *((int32_t *)(buffer + i));
                }

                if (sample_count == BUFFER_SIZE) {
                    msr->datasamples = samples;
                    msr->numsamples = sample_count;
                    write_miniseed_records(msr);
                    sample_count = 0;
                }
            }
        }
    }

    msr_free(&msr);
    close(fd);
    return 0;
}
