/* ftd3xx.h — Minimal header matching FTDI AN_379 struct layout */
#ifndef FTD3XX_H
#define FTD3XX_H

#include <stdint.h>

typedef uint8_t   UCHAR;
typedef uint16_t  USHORT;
typedef uint32_t  ULONG;
typedef uint32_t  DWORD;
typedef int       BOOL;
typedef void*     FT_HANDLE;
typedef uint32_t  FT_STATUS;

#define FT_OK               0
#define FT_OPEN_BY_INDEX    0x10
#define FALSE               0
#define TRUE                1

/* FT_60XCONFIGURATION — from AN_379 pages 58-59 */
typedef struct {
    /* Device Descriptor */
    USHORT VendorID;
    USHORT ProductID;

    /* String Descriptors */
    UCHAR  StringDescriptors[128];

    /* Configuration Descriptor */
    UCHAR  bInterval;
    UCHAR  PowerAttributes;
    USHORT PowerConsumption;

    /* Data Transfer Configuration */
    UCHAR  Reserved2;
    UCHAR  FIFOClock;
    UCHAR  FIFOMode;
    UCHAR  ChannelConfig;

    /* Optional Feature Support */
    USHORT OptionalFeatureSupport;
    UCHAR  BatteryChargingGPIOConfig;
    UCHAR  FlashEEPROMDetection;

    /* MSIO and GPIO Configuration */
    ULONG  MSIO_Control;
    ULONG  GPIO_Control;
} FT_60XCONFIGURATION;

/* Functions */
FT_STATUS FT_CreateDeviceInfoList(DWORD *count);
FT_STATUS FT_Create(DWORD index, DWORD flags, FT_HANDLE *handle);
FT_STATUS FT_Close(FT_HANDLE handle);
FT_STATUS FT_GetChipConfiguration(FT_HANDLE handle, void *cfg);
FT_STATUS FT_SetChipConfiguration(FT_HANDLE handle, void *cfg);
FT_STATUS FT_SetPipeTimeout(FT_HANDLE handle, UCHAR pipe, DWORD timeout);
FT_STATUS FT_SetStreamPipe(FT_HANDLE handle, BOOL all, BOOL all2, UCHAR pipe, DWORD size);
FT_STATUS FT_ClearStreamPipe(FT_HANDLE handle, BOOL all, BOOL all2, UCHAR pipe);
FT_STATUS FT_ReadPipe(FT_HANDLE handle, UCHAR pipe, UCHAR *buf, DWORD len, ULONG *transferred, DWORD timeout);
FT_STATUS FT_WritePipe(FT_HANDLE handle, UCHAR pipe, UCHAR *buf, DWORD len, ULONG *transferred, void *overlapped);
FT_STATUS FT_ResetDevicePort(FT_HANDLE handle);

#endif
