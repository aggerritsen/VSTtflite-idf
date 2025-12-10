#include "diag_dump.h"
#include <WiFi.h>
#include <esp_camera.h>
#include "XPowersLib.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "utilities.h"

// If not defined elsewhere, assume 20 MHz XCLK as in main.cpp
#ifndef CAM_XCLK_FREQ_HZ
#define CAM_XCLK_FREQ_HZ 20000000UL
#endif

// PMU instance is defined in main.cpp
extern XPowersAXP2101 PMU;


// ======================================================
// OV2640 register description structures
// ======================================================

struct Ov2640RegInfo {
    uint8_t     addr;       // register address
    const char *name;       // symbolic name from table
    const char *defval;     // default hex from table
    const char *rw;         // R/W or -
    const char *desc;       // short description
};

// --------- Bank 0 (0xFF = 0x00) = TABLE 12 ----------

static const Ov2640RegInfo ov2640_bank0_regs[] = {
    {0x00, "RSVD", "XX", "-", "Reserved"},
    {0x01, "RSVD", "XX", "-", "Reserved"},
    {0x02, "RSVD", "XX", "-", "Reserved"},
    {0x03, "RSVD", "XX", "-", "Reserved"},
    {0x04, "RSVD", "XX", "-", "Reserved"},
    {0x05, "R_BYPASS", "0x1", "RW", "Bypass DSP Bit[7:1]:    Res...t[0]:       Bypass DSP select 0:    DSP 1:    Bypass DSP, ..."},
    {0x06, "RSVD", "XX", "-", "Reserved"},
    {0x07, "RSVD", "XX", "-", "Reserved"},
    {0x08, "RSVD", "XX", "-", "Reserved"},
    {0x09, "RSVD", "XX", "-", "Reserved"},
    {0x0A, "RSVD", "XX", "-", "Reserved"},
    {0x0B, "RSVD", "XX", "-", "Reserved"},
    {0x0C, "RSVD", "XX", "-", "Reserved"},
    {0x0D, "RSVD", "XX", "-", "Reserved"},
    {0x0E, "RSVD", "XX", "-", "Reserved"},
    {0x0F, "RSVD", "XX", "-", "Reserved"},
    {0x10, "RSVD", "XX", "-", "Reserved"},
    {0x11, "RSVD", "XX", "-", "Reserved"},
    {0x12, "RSVD", "XX", "-", "Reserved"},
    {0x13, "RSVD", "XX", "-", "Reserved"},
    {0x14, "RSVD", "XX", "-", "Reserved"},
    {0x15, "RSVD", "XX", "-", "Reserved"},
    {0x16, "RSVD", "XX", "-", "Reserved"},
    {0x17, "RSVD", "XX", "-", "Reserved"},
    {0x18, "RSVD", "XX", "-", "Reserved"},
    {0x19, "RSVD", "XX", "-", "Reserved"},
    {0x1A, "RSVD", "XX", "-", "Reserved"},
    {0x1B, "RSVD", "XX", "-", "Reserved"},
    {0x1C, "RSVD", "XX", "-", "Reserved"},
    {0x1D, "RSVD", "XX", "-", "Reserved"},
    {0x1E, "RSVD", "XX", "-", "Reserved"},
    {0x1F, "RSVD", "XX", "-", "Reserved"},
    {0x20, "RSVD", "XX", "-", "Reserved"},
    {0x21, "RSVD", "XX", "-", "Reserved"},
    {0x22, "RSVD", "XX", "-", "Reserved"},
    {0x23, "RSVD", "XX", "-", "Reserved"},
    {0x24, "RSVD", "XX", "-", "Reserved"},
    {0x25, "RSVD", "XX", "-", "Reserved"},
    {0x26, "RSVD", "XX", "-", "Reserved"},
    {0x27, "RSVD", "XX", "-", "Reserved"},
    {0x28, "RSVD", "XX", "-", "Reserved"},
    {0x29, "RSVD", "XX", "-", "Reserved"},
    {0x2A, "RSVD", "XX", "-", "Reserved"},
    {0x2B, "RSVD", "XX", "-", "Reserved"},
    {0x2C, "RSVD", "XX", "-", "Reserved"},
    {0x2D, "RSVD", "XX", "-", "Reserved"},
    {0x2E, "RSVD", "XX", "-", "Reserved"},
    {0x2F, "RSVD", "XX", "-", "Reserved"},
    {0x30, "RSVD", "XX", "-", "Reserved"},
    {0x31, "RSVD", "XX", "-", "Reserved"},
    {0x32, "RSVD", "XX", "-", "Reserved"},
    {0x33, "RSVD", "XX", "-", "Reserved"},
    {0x34, "RSVD", "XX", "-", "Reserved"},
    {0x35, "RSVD", "XX", "-", "Reserved"},
    {0x36, "RSVD", "XX", "-", "Reserved"},
    {0x37, "RSVD", "XX", "-", "Reserved"},
    {0x38, "RSVD", "XX", "-", "Reserved"},
    {0x39, "RSVD", "XX", "-", "Reserved"},
    {0x3A, "RSVD", "XX", "-", "Reserved"},
    {0x3B, "RSVD", "XX", "-", "Reserved"},
    {0x3C, "RSVD", "XX", "-", "Reserved"},
    {0x3D, "RSVD", "XX", "-", "Reserved"},
    {0x3E, "RSVD", "XX", "-", "Reserved"},
    {0x3F, "RSVD", "XX", "-", "Reserved"},
    {0x40, "RSVD", "XX", "-", "Reserved"},
    {0x41, "RSVD", "XX", "-", "Reserved"},
    {0x42, "RSVD", "XX", "-", "Reserved"},
    {0x43, "RSVD", "XX", "-", "Reserved"},
    {0x44, "Qs", "0C", "RW", "Quantization Scale Factor"},
    {0x45, "RSVD", "XX", "-", "Reserved"},
    {0x46, "RSVD", "XX", "-", "Reserved"},
    {0x47, "RSVD", "XX", "-", "Reserved"},
    {0x48, "RSVD", "XX", "-", "Reserved"},
    {0x49, "RSVD", "XX", "-", "Reserved"},
    {0x4A, "RSVD", "XX", "-", "Reserved"},
    {0x4B, "RSVD", "XX", "-", "Reserved"},
    {0x4C, "RSVD", "XX", "-", "Reserved"},
    {0x4D, "RSVD", "XX", "-", "Reserved"},
    {0x4E, "RSVD", "XX", "-", "Reserved"},
    {0x4F, "RSVD", "XX", "-", "Reserved"},
    {0x50, "CTRLl[7:0]", "00", "RW", "CTRL1: LP_DP, Round, V_DIVIDER, H_DIVIDER"},
    {0x51, "HSIZE[7:0]", "40", "RW", "H_SIZE[7:0] (real/4)"},
    {0x52, "VSIZE[7:0]", "F0", "RW", "V_SIZE[7:0] (real/4)"},
    {0x53, "XOFFL[7:0]", "00", "RW", "OFFSET_X[7:0]"},
    {0x54, "YOFFL[7:0]", "00", "RW", "OFFSET_Y[7:0]"},
    {0x55, "VHYX[7:0]", "08", "RW", "V_SIZE[8], OFFSET_Y[10:8], H_SIZE[8], OFFSET_X[10:8]"},
    {0x56, "DPRP[7:0]", "00", "RW", "DP_SELY, DP_SELX"},
    {0x57, "TEST[3:0]", "00", "RW", "H_SIZE[9], test bits"},
    {0x58, "RSVD", "XX", "-", "Reserved"},
    {0x59, "RSVD", "XX", "-", "Reserved"},
    {0x5A, "ZMOW[7:0]", "58", "RW", "OUTW[7:0] (real/4)"},
    {0x5B, "ZMOH[7:0]", "48", "RW", "OUTH[7:0] (real/4)"},
    {0x5C, "ZMHH[1:0]", "00", "RW", "ZMSPD, OUTH[8], OUTW[9:8]"},
    {0x5D, "RSVD", "XX", "-", "Reserved"},
    {0x5E, "RSVD", "XX", "-", "Reserved"},
    {0x5F, "RSVD", "XX", "-", "Reserved"},
    {0x60, "RSVD", "XX", "-", "Reserved"},
    {0x61, "RSVD", "XX", "-", "Reserved"},
    {0x62, "RSVD", "XX", "-", "Reserved"},
    {0x63, "RSVD", "XX", "-", "Reserved"},
    {0x64, "RSVD", "XX", "-", "Reserved"},
    {0x65, "RSVD", "XX", "-", "Reserved"},
    {0x66, "RSVD", "XX", "-", "Reserved"},
    {0x67, "RSVD", "XX", "-", "Reserved"},
    {0x68, "RSVD", "XX", "-", "Reserved"},
    {0x69, "RSVD", "XX", "-", "Reserved"},
    {0x6A, "RSVD", "XX", "-", "Reserved"},
    {0x6B, "RSVD", "XX", "-", "Reserved"},
    {0x6C, "RSVD", "XX", "-", "Reserved"},
    {0x6D, "RSVD", "XX", "-", "Reserved"},
    {0x6E, "RSVD", "XX", "-", "Reserved"},
    {0x6F, "RSVD", "XX", "-", "Reserved"},
    {0x70, "RSVD", "XX", "-", "Reserved"},
    {0x71, "RSVD", "XX", "-", "Reserved"},
    {0x72, "RSVD", "XX", "-", "Reserved"},
    {0x73, "RSVD", "XX", "-", "Reserved"},
    {0x74, "RSVD", "XX", "-", "Reserved"},
    {0x75, "RSVD", "XX", "-", "Reserved"},
    {0x76, "RSVD", "XX", "-", "Reserved"},
    {0x77, "RSVD", "XX", "-", "Reserved"},
    {0x78, "RSVD", "XX", "-", "Reserved"},
    {0x79, "RSVD", "XX", "-", "Reserved"},
    {0x7A, "RSVD", "XX", "-", "Reserved"},
    {0x7B, "RSVD", "XX", "-", "Reserved"},
    {0x7C, "BPADDR[3:0]", "00", "RW", "SDE indirect register access address"},
    {0x7D, "BPDATA[7:0]", "00", "RW", "SDE indirect register access data"},
    {0x7E, "RSVD", "XX", "-", "Reserved"},
    {0x7F, "RSVD", "XX", "-", "Reserved"},
    {0x80, "RSVD", "XX", "-", "Reserved"},
    {0x81, "RSVD", "XX", "-", "Reserved"},
    {0x82, "RSVD", "XX", "-", "Reserved"},
    {0x83, "RSVD", "XX", "-", "Reserved"},
    {0x84, "RSVD", "XX", "-", "Reserved"},
    {0x85, "RSVD", "XX", "-", "Reserved"},
    {0x86, "CTRL2", "0D", "RW", "Module enable: DCW, SDE, UV_ADJ, UV_AVG, CMX"},
    {0x87, "CTRL3", "50", "RW", "Module enable: BPC, WPC"},
    {0x88, "RSVD", "XX", "-", "Reserved"},
    {0x89, "RSVD", "XX", "-", "Reserved"},
    {0x8A, "RSVD", "XX", "-", "Reserved"},
    {0x8B, "RSVD", "XX", "-", "Reserved"},
    {0x8C, "SIZEL[5:0]", "00", "RW", "{HSIZE[11], HSIZE[2:0], VSIZE[2:0]}"},
    {0x8D, "RSVD", "XX", "-", "Reserved"},
    {0x8E, "RSVD", "XX", "-", "Reserved"},
    {0x8F, "RSVD", "XX", "-", "Reserved"},
    {0x90, "RSVD", "XX", "-", "Reserved"},
    {0x91, "RSVD", "XX", "-", "Reserved"},
    {0x92, "RSVD", "XX", "-", "Reserved"},
    {0x93, "RSVD", "XX", "-", "Reserved"},
    {0x94, "RSVD", "XX", "-", "Reserved"},
    {0x95, "RSVD", "XX", "-", "Reserved"},
    {0x96, "RSVD", "XX", "-", "Reserved"},
    {0x97, "RSVD", "XX", "-", "Reserved"},
    {0x98, "RSVD", "XX", "-", "Reserved"},
    {0x99, "RSVD", "XX", "-", "Reserved"},
    {0x9A, "RSVD", "XX", "-", "Reserved"},
    {0x9B, "RSVD", "XX", "-", "Reserved"},
    {0x9C, "RSVD", "XX", "-", "Reserved"},
    {0x9D, "RSVD", "XX", "-", "Reserved"},
    {0x9E, "RSVD", "XX", "-", "Reserved"},
    {0x9F, "RSVD", "XX", "-", "Reserved"},
    {0xA0, "RSVD", "XX", "-", "Reserved"},
    {0xA1, "RSVD", "XX", "-", "Reserved"},
    {0xA2, "RSVD", "XX", "-", "Reserved"},
    {0xA3, "RSVD", "XX", "-", "Reserved"},
    {0xA4, "RSVD", "XX", "-", "Reserved"},
    {0xA5, "RSVD", "XX", "-", "Reserved"},
    {0xA6, "RSVD", "XX", "-", "Reserved"},
    {0xA7, "RSVD", "XX", "-", "Reserved"},
    {0xA8, "RSVD", "XX", "-", "Reserved"},
    {0xA9, "RSVD", "XX", "-", "Reserved"},
    {0xAA, "RSVD", "XX", "-", "Reserved"},
    {0xAB, "RSVD", "XX", "-", "Reserved"},
    {0xAC, "RSVD", "XX", "-", "Reserved"},
    {0xAD, "RSVD", "XX", "-", "Reserved"},
    {0xAE, "RSVD", "XX", "-", "Reserved"},
    {0xAF, "RSVD", "XX", "-", "Reserved"},
    {0xB0, "RSVD", "XX", "-", "Reserved"},
    {0xB1, "RSVD", "XX", "-", "Reserved"},
    {0xB2, "RSVD", "XX", "-", "Reserved"},
    {0xB3, "RSVD", "XX", "-", "Reserved"},
    {0xB4, "RSVD", "XX", "-", "Reserved"},
    {0xB5, "RSVD", "XX", "-", "Reserved"},
    {0xB6, "RSVD", "XX", "-", "Reserved"},
    {0xB7, "RSVD", "XX", "-", "Reserved"},
    {0xB8, "RSVD", "XX", "-", "Reserved"},
    {0xB9, "RSVD", "XX", "-", "Reserved"},
    {0xBA, "RSVD", "XX", "-", "Reserved"},
    {0xBB, "RSVD", "XX", "-", "Reserved"},
    {0xBC, "RSVD", "XX", "-", "Reserved"},
    {0xBD, "RSVD", "XX", "-", "Reserved"},
    {0xBE, "RSVD", "XX", "-", "Reserved"},
    {0xBF, "RSVD", "XX", "-", "Reserved"},
    {0xC0, "HSIZE8[7:0]", "80", "RW", "Image horizontal size HSIZE[10:3]"},
    {0xC1, "VSIZE8[7:0]", "60", "RW", "Image vertical size VSIZE[10:3]"},
    {0xC2, "CTRL0", "0C", "RW", "AEC_EN, AEC_SEL, STAT_SEL, VFIRST, YUV422, YUV_EN, RGB_EN, RAW_EN"},
    {0xC3, "CTRL1", "FF", "RW", "CIP, DMY, RAW_GMA, DG, AWB, AWB_GAIN, LENC, PRE"},
    {0xC4, "RSVD", "XX", "-", "Reserved"},
    {0xC5, "RSVD", "XX", "-", "Reserved"},
    {0xC6, "RSVD", "XX", "-", "Reserved"},
    {0xC7, "RSVD", "XX", "-", "Reserved"},
    {0xC8, "RSVD", "XX", "-", "Reserved"},
    {0xC9, "RSVD", "XX", "-", "Reserved"},
    {0xCA, "RSVD", "XX", "-", "Reserved"},
    {0xCB, "RSVD", "XX", "-", "Reserved"},
    {0xCC, "RSVD", "XX", "-", "Reserved"},
    {0xCD, "RSVD", "XX", "-", "Reserved"},
    {0xCE, "RSVD", "XX", "-", "Reserved"},
    {0xCF, "RSVD", "XX", "-", "Reserved"},
    {0xD0, "RSVD", "XX", "-", "Reserved"},
    {0xD1, "RSVD", "XX", "-", "Reserved"},
    {0xD2, "RSVD", "XX", "-", "Reserved"},
    {0xD3, "R_DVP_SP", "82", "RW", "Auto mode, DVP output speed control, PCLK divider"},
    {0xD4, "RSVD", "XX", "-", "Reserved"},
    {0xD5, "RSVD", "XX", "-", "Reserved"},
    {0xD6, "RSVD", "XX", "-", "Reserved"},
    {0xD7, "RSVD", "XX", "-", "Reserved"},
    {0xD8, "RSVD", "XX", "-", "Reserved"},
    {0xD9, "RSVD", "XX", "-", "Reserved"},
    {0xDA, "IMAGE_MODE", "00", "RW", "Image output format select, JPEG enable, byte swap, DVP mode"},
    {0xDB, "RSVD", "XX", "-", "Reserved"},
    {0xDC, "RSVD", "XX", "-", "Reserved"},
    {0xDD, "RSVD", "XX", "-", "Reserved"},
    {0xDE, "RSVD", "XX", "-", "Reserved"},
    {0xDF, "RSVD", "XX", "-", "Reserved"},
    {0xE0, "RESET", "04", "RW", "Reset bits: SCCB, JPEG, DVP, IPU, CIF"},
    {0xE1, "RSVD", "XX", "-", "Reserved"},
    {0xE2, "RSVD", "XX", "-", "Reserved"},
    {0xE3, "RSVD", "XX", "-", "Reserved"},
    {0xE4, "RSVD", "XX", "-", "Reserved"},
    {0xE5, "RSVD", "XX", "-", "Reserved"},
    {0xE6, "RSVD", "XX", "-", "Reserved"},
    {0xE7, "RSVD", "XX", "-", "Reserved"},
    {0xE8, "RSVD", "XX", "-", "Reserved"},
    {0xE9, "RSVD", "XX", "-", "Reserved"},
    {0xEA, "RSVD", "XX", "-", "Reserved"},
    {0xEB, "RSVD", "XX", "-", "Reserved"},
    {0xEC, "RSVD", "XX", "-", "Reserved"},
    {0xED, "REGED", "1F", "RW", "Clock output power-down behavior"},
    {0xEE, "RSVD", "XX", "-", "Reserved"},
    {0xEF, "RSVD", "XX", "-", "Reserved"},
    {0xF0, "MS_SP", "04", "RW", "SCCB master speed"},
    {0xF1, "RSVD", "XX", "-", "Reserved"},
    {0xF2, "RSVD", "XX", "-", "Reserved"},
    {0xF3, "RSVD", "XX", "-", "Reserved"},
    {0xF4, "RSVD", "XX", "-", "Reserved"},
    {0xF5, "RSVD", "XX", "-", "Reserved"},
    {0xF6, "RSVD", "XX", "-", "Reserved"},
    {0xF7, "SS_ID", "60", "RW", "SCCB slave ID"},
    {0xF8, "SS_CTRL", "01", "RW", "SCCB slave control, address auto-increase, enable"},
    {0xF9, "MC_BIST", "40", "RW", "MCU BIST control and status"},
    {0xFA, "MC_AL", "00", "RW", "Program memory pointer address low byte"},
    {0xFB, "MC_AH", "00", "RW", "Program memory pointer address high byte"},
    {0xFC, "MC_D", "80", "RW", "Program memory pointer access address / boundary"},
    {0xFD, "P_CMD", "00", "RW", "SCCB protocol command register"},
    {0xFE, "P_STATUS", "00", "RW", "SCCB protocol status register"},
    {0xFF, "RA_DLMT", "7F", "RW", "Register bank select (0 = DSP/TAB12, 1 = sensor/TAB13)"}
};

static const size_t ov2640_bank0_regs_count =
    sizeof(ov2640_bank0_regs) / sizeof(ov2640_bank0_regs[0]);

// --------- Bank 1 (0xFF = 0x01) = TABLE 13 ----------

static const Ov2640RegInfo ov2640_bank1_regs[] = {
    {0x00, "GAIN", "00", "RW", "AGC gain control LSBs, 1x to 32x"},
    {0x01, "RSVD", "XX", "-", "Reserved"},
    {0x02, "RSVD", "XX", "-", "Reserved"},
    {0x03, "COM1", "0F/0A/06", "RW", "Common control 1 (dummy frames, V window start/end)"},
    {0x04, "REG04", "20", "RW", "Horizontal mirror, vertical flip, VREF bits"},
    {0x05, "RSVD", "XX", "-", "Reserved"},
    {0x06, "RSVD", "XX", "-", "Reserved"},
    {0x07, "RSVD", "XX", "-", "Reserved"},
    {0x08, "REG08", "40", "RW", "Frame exposure one-pin control pre-charge row"},
    {0x09, "COM2", "01", "RW", "Common control 2"},
    {0x0A, "RSVD", "XX", "-", "Reserved"},
    {0x0B, "RSVD", "XX", "-", "Reserved"},
    {0x0C, "COM7", "46", "RW", "Common control 7 (resolution, color bar, RGB/YUV)"},
    {0x0D, "COM8", "C0", "RW", "AGC, AEC, banding filter enable/disable"},
    {0x0E, "COM9", "18", "RW", "AGC gain ceiling"},
    {0x0F, "COM10", "01", "RW", "HSYNC/VREF polarity, PCLK behavior"},
    {0x10, "RSVD", "XX", "-", "Reserved"},
    {0x11, "CLKRC", "01", "RW", "Internal clock prescaler"},
    {0x12, "COM12", "03", "RW", "DC offset auto correction, HREF control"},
    {0x13, "COM13", "8F", "RW", "Gamma, UV saturation, color matrix"},
    {0x14, "RSVD", "XX", "-", "Reserved"},
    {0x15, "RSVD", "XX", "-", "Reserved"},
    {0x16, "RSVD", "XX", "-", "Reserved"},
    {0x17, "HSTART", "11", "RW", "Horizontal window start high bits"},
    {0x18, "HSTOP", "61", "RW", "Horizontal window stop high bits"},
    {0x19, "VSTART", "03", "RW", "Vertical window start high bits"},
    {0x1A, "VSTOP", "7B", "RW", "Vertical window stop high bits"},
    {0x1B, "PSHFT", "00", "RW", "Pixel shift"},
    {0x1C, "MIDH", "7F", "R",  "Manufacturer ID high"},
    {0x1D, "MIDL", "A2", "R",  "Manufacturer ID low"},
    {0x1E, "RSVD", "XX", "-", "Reserved"},
    {0x1F, "RSVD", "XX", "-", "Reserved"},
    {0x20, "AEW", "75", "RW", "AEC stable upper region"},
    {0x21, "AEB", "63", "RW", "AEC stable lower region"},
    {0x22, "VV", "01", "RW", "Fast/slow AEC algorithm tuning"},
    {0x23, "RSVD", "XX", "-", "Reserved"},
    {0x24, "RSVD", "XX", "-", "Reserved"},
    {0x25, "RSVD", "XX", "-", "Reserved"},
    {0x26, "RSVD", "XX", "-", "Reserved"},
    {0x27, "RSVD", "XX", "-", "Reserved"},
    {0x28, "RSVD", "XX", "-", "Reserved"},
    {0x29, "RSVD", "XX", "-", "Reserved"},
    {0x2A, "RSVD", "XX", "-", "Reserved"},
    {0x2B, "RSVD", "XX", "-", "Reserved"},
    {0x2C, "RSVD", "XX", "-", "Reserved"},
    {0x2D, "RSVD", "XX", "-", "Reserved"},
    {0x2E, "RSVD", "XX", "-", "Reserved"},
    {0x2F, "RSVD", "XX", "-", "Reserved"},
    {0x30, "RSVD", "XX", "-", "Reserved"},
    {0x31, "RSVD", "XX", "-", "Reserved"},
    {0x32, "RSVD", "XX", "-", "Reserved"},
    {0x33, "RSVD", "XX", "-", "Reserved"},
    {0x34, "RSVD", "XX", "-", "Reserved"},
    {0x35, "RSVD", "XX", "-", "Reserved"},
    {0x36, "RSVD", "XX", "-", "Reserved"},
    {0x37, "RSVD", "XX", "-", "Reserved"},
    {0x38, "RSVD", "XX", "-", "Reserved"},
    {0x39, "RSVD", "XX", "-", "Reserved"},
    {0x3A, "RSVD", "XX", "-", "Reserved"},
    {0x3B, "RSVD", "XX", "-", "Reserved"},
    {0x3C, "RSVD", "XX", "-", "Reserved"},
    {0x3D, "RSVD", "XX", "-", "Reserved"},
    {0x3E, "RSVD", "XX", "-", "Reserved"},
    {0x3F, "RSVD", "XX", "-", "Reserved"},
    {0x40, "RSVD", "XX", "-", "Reserved"},
    {0x41, "RSVD", "XX", "-", "Reserved"},
    {0x42, "RSVD", "XX", "-", "Reserved"},
    {0x43, "RSVD", "XX", "-", "Reserved"},
    {0x44, "RSVD", "XX", "-", "Reserved"},
    {0x45, "RSVD", "XX", "-", "Reserved"},
    {0x46, "RSVD", "XX", "-", "Reserved"},
    {0x47, "RSVD", "XX", "-", "Reserved"},
    {0x48, "RSVD", "XX", "-", "Reserved"},
    {0x49, "RSVD", "XX", "-", "Reserved"},
    {0x4A, "RSVD", "XX", "-", "Reserved"},
    {0x4B, "RSVD", "XX", "-", "Reserved"},
    {0x4C, "RSVD", "XX", "-", "Reserved"},
    {0x4D, "RSVD", "XX", "-", "Reserved"},
    {0x4E, "RSVD", "XX", "-", "Reserved"},
    {0x4F, "RSVD", "XX", "-", "Reserved"},
    // ... (TAB13 continues for all defined sensor registers)
};

static const size_t ov2640_bank1_regs_count =
    sizeof(ov2640_bank1_regs) / sizeof(ov2640_bank1_regs[0]);

// ======================================================
// Lookup helper
// ======================================================

static const Ov2640RegInfo *find_ov2640_reg(uint8_t bank, uint8_t addr)
{
    const Ov2640RegInfo *table = nullptr;
    size_t count = 0;

    if (bank == 0) {
        table = ov2640_bank0_regs;
        count = ov2640_bank0_regs_count;
    } else if (bank == 1) {
        table = ov2640_bank1_regs;
        count = ov2640_bank1_regs_count;
    } else {
        return nullptr;
    }

    for (size_t i = 0; i < count; ++i) {
        if (table[i].addr == addr) {
            return &table[i];
        }
    }
    return nullptr;
}

// ======================================================
// PMU / GPIO / clocks / pins (unchanged behavior)
// ======================================================

static void dump_pmu_registers_matrix()
{
    Serial.println("\n=== AXP2101 REGISTER MATRIX 0x00-0xFF ===");

    const int cols = 4;
    uint8_t val = 0;

    for (uint16_t r = 0; r < 0x100; r++) {
        val = PMU.readRegister(r);
        Serial.printf("0x%02X:%02X", r, val);
        Serial.print((r % cols == cols - 1) ? "\n" : "  ");
    }
    Serial.println();
}

static void dump_pmu_status()
{
    Serial.println("\n=== AXP2101 STATUS ===");
    Serial.printf("Power Key Pressed: %d\n", PMU.isPekeyShortPressIrq());
    Serial.printf("Input Voltage(mV): %d\n", PMU.getVbusVoltage());
    Serial.printf("Battery Voltage(mV): %d\n", PMU.getBattVoltage());
}

static void dump_pmu_voltages()
{
    Serial.println("\n=== PMU Power Rails ===");

    Serial.printf("ALDO1 (CAM DVDD core): %dmV\n", PMU.getALDO1Voltage());
    Serial.printf("ALDO2 (CAM DVDD io):   %dmV\n", PMU.getALDO2Voltage());
    Serial.printf("ALDO4 (CAM AVDD):      %dmV\n", PMU.getALDO4Voltage());
    Serial.printf("DLDO1: %dmV\n", PMU.getDLDO1Voltage());
    Serial.printf("DLDO2: %dmV\n", PMU.getDLDO2Voltage());
    Serial.printf("BLDO1: %dmV\n", PMU.getBLDO1Voltage());
    Serial.printf("BLDO2 (3V3 IO): %dmV\n", PMU.getBLDO2Voltage());
}

static void dump_gpio()
{
    Serial.println("\n=== ESP32-S3 GPIO LEVELS (non-intrusive) ===");

    for (int pin = 0; pin <= 48; pin++) {
        if (!GPIO_IS_VALID_GPIO(pin)) {
            continue;
        }
        int lvl = gpio_get_level((gpio_num_t)pin);
        Serial.printf("GPIO%-2d level:%d\n", pin, lvl);
    }
}

static void dump_xclk_info()
{
    Serial.println("\n=== LEDC / XCLK INFO ===");
    Serial.printf("Configured XCLK frequency: %u Hz\n",
                  (unsigned)CAM_XCLK_FREQ_HZ);
}

static void dump_chip_clocks()
{
    Serial.println("\n=== ESP32-S3 CLOCK INFO ===");

    esp_chip_info_t info;
    esp_chip_info(&info);

    Serial.printf("Chip cores: %d, revision:%d\n", info.cores, info.revision);
    Serial.printf("CPU freq (Arduino): %u MHz\n", getCpuFrequencyMhz());
    Serial.printf("APB freq (Arduino): %u MHz\n",
                  getApbFrequency() / 1000000);
}

static void dump_camera_pins()
{
    Serial.println("\n=== CAMERA PIN MAP (OV2640 on T-SIM7080G-S3) ===");
    Serial.printf("Y2   (D2)   : GPIO%d\n", Y2_GPIO_NUM);
    Serial.printf("Y3   (D3)   : GPIO%d\n", Y3_GPIO_NUM);
    Serial.printf("Y4   (D4)   : GPIO%d\n", Y4_GPIO_NUM);
    Serial.printf("Y5   (D5)   : GPIO%d\n", Y5_GPIO_NUM);
    Serial.printf("Y6   (D6)   : GPIO%d\n", Y6_GPIO_NUM);
    Serial.printf("Y7   (D7)   : GPIO%d\n", Y7_GPIO_NUM);
    Serial.printf("Y8   (D8)   : GPIO%d\n", Y8_GPIO_NUM);
    Serial.printf("Y9   (D9)   : GPIO%d\n", Y9_GPIO_NUM);

    Serial.printf("XCLK        : GPIO%d\n", XCLK_GPIO_NUM);
    Serial.printf("PCLK        : GPIO%d\n", PCLK_GPIO_NUM);
    Serial.printf("VSYNC       : GPIO%d\n", VSYNC_GPIO_NUM);
    Serial.printf("HREF        : GPIO%d\n", HREF_GPIO_NUM);
    Serial.printf("SIOC (SCL)  : GPIO%d\n", SIOC_GPIO_NUM);
    Serial.printf("SIOD (SDA)  : GPIO%d\n", SIOD_GPIO_NUM);
    Serial.printf("PWDN        : GPIO%d\n", PWDN_GPIO_NUM);
    Serial.printf("RESET       : GPIO%d\n", RESET_GPIO_NUM);
}

static void dump_sensor_struct(sensor_t *s)
{
    Serial.println("\n=== sensor_t STATUS ===");

    Serial.printf("PID:0x%04X VER:0x%04X\n", s->id.PID, s->id.VER);
    Serial.printf("brightness=%d contrast=%d saturation=%d\n",
                  s->status.brightness,
                  s->status.contrast,
                  s->status.saturation);
    Serial.printf("AEC=%d AGC=%d AWB=%d\n",
                  s->status.aec,
                  s->status.agc,
                  s->status.awb);
    Serial.printf("framesize=%d quality=%d\n",
                  s->status.framesize,
                  s->status.quality);
    Serial.printf("special_effect=%d hmirror=%d vflip=%d\n",
                  s->status.special_effect,
                  s->status.hmirror,
                  s->status.vflip);
}

// ======================================================
// OV2640 register dumps with decoding (both banks)
// ======================================================

static void dump_ov2640_bank(sensor_t *s, uint8_t bank)
{
    Serial.printf(
        "\n=== OV2640 REGISTER DUMP (BANK %u, 0xFF = 0x%02X) ===\n",
        bank, bank);

    // Select bank (0 = DSP/TAB12, 1 = sensor/TAB13)
    s->set_reg(s, 0xFF, 0xFF, bank);

    for (uint16_t r = 0; r < 0x100; ++r) {
        int val = s->get_reg(s, r, 0xFF);
        if (val < 0) {
            val = 0xFF;
        }
        const Ov2640RegInfo *info =
            find_ov2640_reg(bank, static_cast<uint8_t>(r));

        if (info) {
            Serial.printf(
                "B%u 0x%02X:0x%02X  %-14s def:%-4s  %s\n",
                bank, r, val & 0xFF,
                info->name, info->defval, info->desc);
        } else {
            Serial.printf("B%u 0x%02X:0x%02X\n",
                          bank, r, val & 0xFF);
        }
    }
}

// ======================================================
// Public API
// ======================================================

void initDiagnostics()
{
    // PMU is initialized in main.cpp
}

void runDiagnostics()
{
    sensor_t *s = esp_camera_sensor_get();
    delay(200);

    dump_pmu_registers_matrix();
    dump_pmu_status();
    dump_pmu_voltages();
    dump_gpio();
    dump_xclk_info();
    dump_chip_clocks();
    dump_camera_pins();

    if (s) {
        dump_sensor_struct(s);
        dump_ov2640_bank(s, 0);   // 0xFF = 0x00 → TABLE 12 (DSP)
        dump_ov2640_bank(s, 1);   // 0xFF = 0x01 → TABLE 13 (sensor)
    }

    Serial.println("\n=== END DIAGNOSTIC ===");
}
