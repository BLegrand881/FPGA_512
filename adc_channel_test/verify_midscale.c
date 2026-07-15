#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "ftd3xx.h"

#define CHUNK_SIZE  (64 * 1024)
#define DEFAULT_MB  1

int main(int argc, char *argv[]) {
    int total_mb = (argc > 1) ? atoi(argv[1]) : DEFAULT_MB;
    size_t total_bytes = (size_t)total_mb * 1024 * 1024;

    FT_STATUS st;
    FT_HANDLE handle = NULL;
    DWORD count = 0;

    st = FT_CreateDeviceInfoList(&count);
    if (count == 0) { printf("No devices\n"); return 1; }

    st = FT_Create(0, FT_OPEN_BY_INDEX, &handle);
    if (st != FT_OK) { printf("FT_Create failed: %d\n", st); return 1; }

    FT_60XCONFIGURATION cfg;
    memset(&cfg, 0, sizeof(cfg));
    FT_GetChipConfiguration(handle, &cfg);
    printf("FT600 opened. FIFOMode=%d ChannelConfig=%d\n", cfg.FIFOMode, cfg.ChannelConfig);

    FT_SetPipeTimeout(handle, 0x82, 3000);
    st = FT_SetStreamPipe(handle, FALSE, FALSE, 0x82, CHUNK_SIZE);
    if (st != FT_OK) { printf("SetStreamPipe failed: %d\n", st); FT_Close(handle); return 1; }
    usleep(100000);

    printf("Reading %d MB...\n\n", total_mb);

    UCHAR buf[CHUNK_SIZE];
    ULONG transferred;
    size_t total_read = 0;
    size_t total_samples = 0;
    size_t upper_nonzero = 0;
    size_t near_2048 = 0;      /* within ±100 */
    size_t zeros = 0;
    double sum_val = 0, sum_sq = 0;
    unsigned short vmin = 0xFFFF, vmax = 0;
    size_t histogram[8] = {0};  /* 512-wide buckets */

    struct timespec t_start, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    while (total_read < total_bytes) {
        transferred = 0;
        st = FT_ReadPipe(handle, 0x82, buf, CHUNK_SIZE, &transferred, 3000);
        if (st != FT_OK) {
            printf("\nRead error: status=%d\n", st);
            break;
        }
        if (transferred == 0) { usleep(10000); continue; }

        total_read += transferred;
        int n_words = transferred / 2;

        for (int i = 0; i < n_words; i++) {
            unsigned short w = buf[i*2] | (buf[i*2+1] << 8);
            unsigned short upper = w >> 12;
            unsigned short val = w & 0x0FFF;

            if (upper != 0) upper_nonzero++;
            if (val == 0) zeros++;
            if (val < vmin) vmin = val;
            if (val > vmax) vmax = val;
            sum_val += val;
            sum_sq += (double)val * val;
            if (val >= 1948 && val <= 2148) near_2048++;
            if (val < 4096) histogram[val / 512]++;
            total_samples++;
        }

        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                         (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
        double rate = (total_read / 1e6) / elapsed;
        printf("\r  %.1f MB, %zu samples, %.1f MB/s",
               total_read / 1e6, total_samples, rate);
        fflush(stdout);
    }

    clock_gettime(CLOCK_MONOTONIC, &t_now);
    double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                     (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
    double rate = (total_read / 1e6) / elapsed;
    double mean = total_samples > 0 ? sum_val / total_samples : 0;
    double variance = total_samples > 0 ? (sum_sq / total_samples) - (mean * mean) : 0;
    double stddev = sqrt(variance > 0 ? variance : 0);

    printf("\n\n============================================================\n");
    printf("  Read:           %.2f MB in %.2f s (%.1f MB/s)\n", total_read / 1e6, elapsed, rate);
    printf("  Samples:        %zu\n", total_samples);
    printf("============================================================\n");
    printf("  Min:            %u\n", vmin);
    printf("  Max:            %u\n", vmax);
    printf("  Mean:           %.1f\n", mean);
    printf("  StdDev:         %.1f\n", stddev);
    printf("  Zeros:          %zu (%.1f%%)\n", zeros, 100.0 * zeros / total_samples);
    printf("  Upper bits != 0:%zu (%.1f%%)\n", upper_nonzero, 100.0 * upper_nonzero / total_samples);
    printf("  Near 2048 ±100: %zu (%.1f%%)\n", near_2048, 100.0 * near_2048 / total_samples);
    printf("============================================================\n");

    const char *labels[] = {"0-511","512-1023","1024-1535","1536-2047",
                            "2048-2559","2560-3071","3072-3583","3584-4095"};
    printf("\n  Distribution:\n");
    size_t hmax = 0;
    for (int i = 0; i < 8; i++) if (histogram[i] > hmax) hmax = histogram[i];
    for (int i = 0; i < 8; i++) {
        int bar = hmax > 0 ? (int)(40.0 * histogram[i] / hmax) : 0;
        printf("    %10s: %8zu ", labels[i], histogram[i]);
        for (int j = 0; j < bar; j++) printf("#");
        printf("\n");
    }

    printf("\n");
    if (mean > 1900 && mean < 2200 && near_2048 > total_samples * 99 / 100)
        printf("  PASS — ADC outputs near mid-scale\n");
    else if (total_samples == 0)
        printf("  NO DATA\n");
    else
        printf("  FAIL — values not centered on 2048\n");

    FT_ClearStreamPipe(handle, FALSE, FALSE, 0x82);
    FT_Close(handle);
    return 0;
}
