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
    const char *outfile = (argc > 2) ? argv[2] : "raw.bin";
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

    FILE *fp = fopen(outfile, "wb");
    if (!fp) { printf("Cannot open %s\n", outfile); FT_Close(handle); return 1; }

    printf("Reading %d MB → %s ...\n", total_mb, outfile);

    UCHAR buf[CHUNK_SIZE];
    ULONG transferred;
    size_t total_read = 0;

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

        fwrite(buf, 1, transferred, fp);
        total_read += transferred;

        clock_gettime(CLOCK_MONOTONIC, &t_now);
        double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                         (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
        double rate = (total_read / 1e6) / elapsed;
        printf("\r  %.1f MB / %d MB, %.1f MB/s",
               total_read / 1e6, total_mb, rate);
        fflush(stdout);
    }

    fclose(fp);

    clock_gettime(CLOCK_MONOTONIC, &t_now);
    double elapsed = (t_now.tv_sec - t_start.tv_sec) +
                     (t_now.tv_nsec - t_start.tv_nsec) / 1e9;
    printf("\n\nDone: %.2f MB in %.2f s (%.1f MB/s) → %s\n",
           total_read / 1e6, elapsed, (total_read / 1e6) / elapsed, outfile);

    FT_ClearStreamPipe(handle, FALSE, FALSE, 0x82);
    FT_Close(handle);
    return 0;
}
