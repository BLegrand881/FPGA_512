#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "ftd3xx.h"

#define CHUNK_SIZE  (64 * 1024)
#define DEFAULT_MB  1
#define FLUSH_BYTES (64 * 1024)

static const char *pin_names[] = {
    "", "B15", "B16", "C15", "C16", "J15", "K15", "K14", "J14"
};

typedef struct {
    int    channel;
    double mean;
    double stddev;
    unsigned short vmin;
    unsigned short vmax;
    size_t total_samples;
    size_t zeros;
    size_t ch_mismatch;
    int    pass;
} result_t;

static int send_channel(FT_HANDLE handle, int ch) {
    UCHAR cmd[2] = { (UCHAR)ch, 0 };
    ULONG written = 0;
    FT_STATUS st = FT_WritePipe(handle, 0x02, cmd, 2, &written, NULL);
    if (st != FT_OK) {
        printf("FT_WritePipe failed: %d\n", st);
        return -1;
    }
    return 0;
}

static result_t test_channel(FT_HANDLE handle, int ch, int total_mb) {
    result_t r = {0};
    r.channel = ch;
    r.vmin = 0xFFFF;

    /* Send channel selection */
    if (send_channel(handle, ch) < 0) return r;

    /* Wait for deserializer to re-sync */
    usleep(100000);

    /* Flush stale data */
    UCHAR buf[CHUNK_SIZE];
    ULONG transferred;
    size_t flushed = 0;
    while (flushed < FLUSH_BYTES) {
        transferred = 0;
        FT_STATUS st = FT_ReadPipe(handle, 0x82, buf, CHUNK_SIZE, &transferred, 500);
        if (st != FT_OK || transferred == 0) break;
        flushed += transferred;
    }

    /* Read and analyze */
    size_t total_bytes = (size_t)total_mb * 1024 * 1024;
    size_t total_read = 0;
    double sum_val = 0, sum_sq = 0;

    struct timespec t_start, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    while (total_read < total_bytes) {
        transferred = 0;
        FT_STATUS st = FT_ReadPipe(handle, 0x82, buf, CHUNK_SIZE, &transferred, 3000);
        if (st != FT_OK) break;
        if (transferred == 0) { usleep(10000); continue; }
        total_read += transferred;

        int n_words = transferred / 2;
        for (int i = 0; i < n_words; i++) {
            unsigned short w = buf[i*2] | (buf[i*2+1] << 8);
            unsigned short upper = (w >> 12) & 0xF;
            unsigned short val = w & 0x0FFF;

            if (upper != (unsigned short)ch && r.total_samples > 2048) r.ch_mismatch++;
            if (val == 0) r.zeros++;
            if (val < r.vmin) r.vmin = val;
            if (val > r.vmax) r.vmax = val;
            sum_val += val;
            sum_sq += (double)val * val;
            r.total_samples++;
        }

        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                         (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
        double rate = (total_read / 1e6) / elapsed;
        printf("\r  Ch %d (%s): %.1f MB, %zu samples, %.1f MB/s    ",
               ch, pin_names[ch], total_read / 1e6, r.total_samples, rate);
        fflush(stdout);
    }
    printf("\n");

    if (r.total_samples > 0) {
        r.mean = sum_val / r.total_samples;
        double variance = (sum_sq / r.total_samples) - (r.mean * r.mean);
        r.stddev = sqrt(variance > 0 ? variance : 0);
    }

    r.pass = (r.total_samples > 0 &&
              r.mean > 1900 && r.mean < 2200 &&
              r.stddev < 100 &&
              r.zeros < r.total_samples / 100);

    return r;
}

static void print_header(void) {
    printf("\n%-4s %-5s %8s %8s %6s %6s %8s %8s  %s\n",
           "Ch", "Pin", "Mean", "StdDev", "Min", "Max", "Zeros", "ChErr", "Result");
    printf("---- ----- -------- -------- ------ ------ -------- --------  ------\n");
}

static void print_result(result_t *r) {
    printf("%-4d %-5s %8.1f %8.1f %6u %6u %8zu %8zu  %s\n",
           r->channel, pin_names[r->channel],
           r->mean, r->stddev, r->vmin, r->vmax,
           r->zeros, r->ch_mismatch,
           r->pass ? "PASS" : "FAIL");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("  ./channel_test <ch>       — test channel 1-8 (1 MB)\n");
        printf("  ./channel_test <ch> <MB>  — test channel, N MB\n");
        printf("  ./channel_test all        — test all channels (1 MB each)\n");
        printf("  ./channel_test all <MB>   — test all channels, N MB each\n");
        printf("\nChannels: 1=B15  2=B16  3=C15  4=C16  5=J15  6=K15  7=K14  8=J14\n");
        return 1;
    }

    int do_all = (strcmp(argv[1], "all") == 0);
    int ch_arg = do_all ? 0 : atoi(argv[1]);
    int total_mb = DEFAULT_MB;

    if (argc > 2) total_mb = atoi(argv[2]);
    if (total_mb < 1) total_mb = 1;

    if (!do_all && (ch_arg < 1 || ch_arg > 8)) {
        printf("Channel must be 1-8\n");
        return 1;
    }

    /* Open device */
    FT_STATUS st;
    FT_HANDLE handle = NULL;
    DWORD count = 0;

    st = FT_CreateDeviceInfoList(&count);
    if (count == 0) { printf("No devices\n"); return 1; }

    st = FT_Create(0, FT_OPEN_BY_INDEX, &handle);
    if (st != FT_OK) { printf("FT_Create failed: %d\n", st); return 1; }

    FT_SetPipeTimeout(handle, 0x82, 3000);
    st = FT_SetStreamPipe(handle, FALSE, FALSE, 0x82, CHUNK_SIZE);
    if (st != FT_OK) {
        printf("SetStreamPipe failed: %d\n", st);
        FT_Close(handle);
        return 1;
    }

    if (do_all) {
        printf("Testing all 8 channels, %d MB each...\n", total_mb);
        result_t results[8];

        for (int ch = 1; ch <= 8; ch++)
            results[ch-1] = test_channel(handle, ch, total_mb);

        print_header();
        for (int i = 0; i < 8; i++)
            print_result(&results[i]);
        printf("\n");
    } else {
        printf("Testing channel %d (%s), %d MB...\n", ch_arg, pin_names[ch_arg], total_mb);
        result_t r = test_channel(handle, ch_arg, total_mb);
        print_header();
        print_result(&r);
        printf("\n");
    }

    FT_ClearStreamPipe(handle, FALSE, FALSE, 0x82);
    FT_Close(handle);
    return 0;
}
