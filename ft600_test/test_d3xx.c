#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
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

    // Config
    FT_60XCONFIGURATION cfg;
    memset(&cfg, 0, sizeof(cfg));
    FT_GetChipConfiguration(handle, &cfg);
    printf("FT600 opened. FIFOMode=%d ChannelConfig=%d\n", cfg.FIFOMode, cfg.ChannelConfig);

    // Setup
    FT_SetPipeTimeout(handle, 0x82, 3000);
    st = FT_SetStreamPipe(handle, FALSE, FALSE, 0x82, CHUNK_SIZE);
    if (st != FT_OK) { printf("SetStreamPipe failed: %d\n", st); FT_Close(handle); return 1; }

    usleep(100000);  // let TXE_N assert and FPGA start writing

    printf("Reading %d MB...\n\n", total_mb);

    UCHAR buf[CHUNK_SIZE];
    ULONG transferred;
    size_t total_read = 0;
    size_t total_words = 0;
    size_t total_errors = 0;
    int synced = 0;
    unsigned short expected = 0;

    struct timespec t_start, t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    while (total_read < total_bytes) {
        transferred = 0;
        st = FT_ReadPipe(handle, 0x82, buf, CHUNK_SIZE, &transferred, 3000);
        if (st != FT_OK) {
            printf("\nRead error: status=%d transferred=%u\n", st, transferred);
            break;
        }
        if (transferred == 0) {
            usleep(10000);
            continue;
        }

        total_read += transferred;
        int n_words = transferred / 2;

        if (!synced && transferred >= 2) {
            expected = buf[0] | (buf[1] << 8);
            printf("Synced to counter: 0x%04X\n", expected);
            synced = 1;
        }

        for (int i = 0; i < n_words; i++) {
            unsigned short w = buf[i*2] | (buf[i*2+1] << 8);
            if (w != expected) {
                if (total_errors < 20) {
                    printf("ERROR at word %zu: expected 0x%04X got 0x%04X\n",
                           total_words + i, expected, w);
                }
                total_errors++;
                expected = w;
            }
            expected = (expected + 1) & 0xFFFF;
        }
        total_words += n_words;

        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                         (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
        double rate = (total_read / 1e6) / elapsed;
        printf("\r  %.1f MB, %zu words, %zu errors, %.1f MB/s",
               total_read / 1e6, total_words, total_errors, rate);
        fflush(stdout);
    }

    clock_gettime(CLOCK_MONOTONIC, &t_now);
    double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                     (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
    double rate = (total_read / 1e6) / elapsed;

    printf("\n\n============================================================\n");
    printf("  Total read:   %zu bytes (%.2f MB)\n", total_read, total_read / 1e6);
    printf("  Total words:  %zu\n", total_words);
    printf("  Errors:       %zu\n", total_errors);
    printf("  Duration:     %.2f s\n", elapsed);
    printf("  Throughput:   %.1f MB/s\n", rate);
    printf("============================================================\n");

    if (total_errors == 0 && total_words > 0)
        printf("  PASS — counter sequence verified!\n");
    else if (total_words == 0)
        printf("  NO DATA\n");
    else
        printf("  FAIL — %zu errors\n", total_errors);

    FT_ClearStreamPipe(handle, FALSE, FALSE, 0x82);
    FT_Close(handle);
    return (total_errors == 0 && total_words > 0) ? 0 : 1;
}
