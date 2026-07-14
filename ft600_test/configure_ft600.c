#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ftd3xx.h"

int main(int argc, char *argv[]) {
    FT_STATUS st;
    FT_HANDLE handle = NULL;
    DWORD count = 0;

    st = FT_CreateDeviceInfoList(&count);
    if (count == 0) { printf("No devices\n"); return 1; }

    st = FT_Create(0, FT_OPEN_BY_INDEX, &handle);
    if (st != FT_OK) { printf("FT_Create failed: %d\n", st); return 1; }

    // Read current config
    FT_60XCONFIGURATION cfg;
    memset(&cfg, 0, sizeof(cfg));
    st = FT_GetChipConfiguration(handle, &cfg);
    if (st != FT_OK) { printf("GetChipConfig failed: %d\n", st); FT_Close(handle); return 1; }

    printf("Current configuration:\n");
    printf("  FIFOMode      = %d (%s)\n", cfg.FIFOMode,
           cfg.FIFOMode == 0 ? "245" : "600");
    printf("  ChannelConfig = %d (%s)\n", cfg.ChannelConfig,
           cfg.ChannelConfig == 0 ? "4ch" :
           cfg.ChannelConfig == 1 ? "2ch" :
           cfg.ChannelConfig == 2 ? "1ch" :
           cfg.ChannelConfig == 3 ? "1ch OUT only" :
           cfg.ChannelConfig == 4 ? "1ch IN only" : "unknown");
    printf("  FIFOClock     = %d (%s)\n", cfg.FIFOClock,
           cfg.FIFOClock == 0 ? "100MHz" : "66MHz");

    if (argc > 1 && strcmp(argv[1], "set600") == 0) {
        printf("\nSetting FT600 mode, 1 channel IN only...\n");
        cfg.FIFOMode = 1;       // CONFIGURATION_FIFO_MODE_600
        cfg.ChannelConfig = 4;  // CONFIGURATION_CHANNEL_CONFIG_1_INPIPE
        cfg.FIFOClock = 0;      // 100 MHz

        st = FT_SetChipConfiguration(handle, &cfg);
        if (st != FT_OK) {
            printf("SetChipConfig failed: %d\n", st);
            FT_Close(handle);
            return 1;
        }
        printf("Configuration written. Device will restart.\n");
        printf("Unplug/replug USB cable, then verify with: ./configure_ft600\n");
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "set245") == 0) {
        printf("\nSetting 245 mode, 1 channel...\n");
        cfg.FIFOMode = 0;       // CONFIGURATION_FIFO_MODE_245
        cfg.ChannelConfig = 2;  // CONFIGURATION_CHANNEL_CONFIG_1
        cfg.FIFOClock = 0;      // 100 MHz

        st = FT_SetChipConfiguration(handle, &cfg);
        if (st != FT_OK) {
            printf("SetChipConfig failed: %d\n", st);
            FT_Close(handle);
            return 1;
        }
        printf("Configuration written. Device will restart.\n");
        printf("Unplug/replug USB cable, then verify with: ./configure_ft600\n");
        return 0;
    }

    printf("\nUsage:\n");
    printf("  ./configure_ft600          — show current config\n");
    printf("  ./configure_ft600 set600   — FT600 mode, 1ch IN only (USB 3.0)\n");
    printf("  ./configure_ft600 set245   — 245 mode, 1ch (USB 2.0 compatible)\n");

    FT_Close(handle);
    return 0;
}
