
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <pthread.h>
#include "fs_log.h"
#include "fs_err.h"
#include "fs_util.h"
#include "board_cfg.h"

#include "drivers/i2c/i2c_master.h"

#undef LOG_TAG
#define LOG_TAG "serdes"

#define SERDES_I2C_BUS          MCU_CMD_I2C_BUS
#define DES_I2C_ADDR            0x52
#define SER_DEF_I2C_ADDR        0x80
#define SER_A_I2C_ADDR          0x80

static int serdes_i2c_write_byte(uint8_t slave_addr, uint16_t reg, uint8_t data)
{
    return i2c_master_write(SERDES_I2C_BUS, slave_addr, reg, 2, &data, 1);
}

// static int serdes_i2c_read_byte(uint8_t slave_addr, uint16_t reg, uint8_t *data)
// {
//     return i2c_master_read(SERDES_I2C_BUS, slave_addr, reg, 2, data, 1);
// }

// 初始化 Serdes
int serdes_init(uint8_t mipi_rate_100mbps)
{
    int32_t err = FS_SUCCESS;
    FS_LOGD("try serdes init...");
    err = serdes_i2c_write_byte(DES_I2C_ADDR, 0x0017, 0x14);  // REG_ENABLE=1

    if (err == FS_SUCCESS) {
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0017, 0x14));  // REG_ENABLE=1
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0019, 0x94));  // REG_MNL=1
        usleep(10000);

        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x06C2, 0x10));
        usleep(10000);

        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x14D1, 0x03));
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x15D1, 0x03));
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x16D1, 0x03));
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x17D1, 0x03));
        usleep(10000);
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1445, 0x00));
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1545, 0x00));
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1645, 0x00));
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1745, 0x00));
        usleep(10000);

        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x040b,
                                    0x00));  // BACKTOP : BACKTOP12 | CSI_OUT_EN (CSI_OUT_EN): CSI output disabled

        /* DEV : REG3
            | DIS_REM_CC_A (GMSL Link A I2C Port 0): Enabled
            | DIS_REM_CC_B (GMSL Link B I2C Port 0): Disabled
            | DIS_REM_CC_C (GMSL Link C I2C Port 0): Disabled
            | DIS_REM_CC_D (GMSL Link D I2C Port 0): Disabled
        */
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0003, 0xFE));
        usleep(5000);

        //
        // INSTRUCTIONS FOR DESERIALIZER MAX96712
        //
        // Video Pipes And Routing Configuration
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x00F0,
                                    0x62));  // VIDEO_PIPE_SEL : VIDEO_PIPE_SEL_0 | (Default) VIDEO_PIPE_SEL_0 (Pipe 0
                                                // GMSL2 PHY): A | VIDEO_PIPE_SEL_0 (Pipe 0 Input Pipe): X
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x00F4,
            0x01));  // VIDEO_PIPE_SEL : VIDEO_PIPE_EN | (Default) VIDEO_PIPE_EN (Video Pipe 0): Enabled | VIDEO_PIPE_EN
                    // (Video Pipe 1): Disabled | VIDEO_PIPE_EN (Video Pipe 2): Disabled | VIDEO_PIPE_EN (Video Pipe 3):
                    // Disabled | STREAM_SEL_ALL (Stream Select All): Disabled
        // Pipe to Controller Mapping Configuration
        CHECK_ERR(
            serdes_i2c_write_byte(DES_I2C_ADDR, 0x090B, 0x07));  // MIPI_TX__0 : MIPI_TX11 | MAP_EN_L (MAP_EN_L Pipe 0): 0x7
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x090C,
                                    0x00));  // MIPI_TX__0 : MIPI_TX12 | (Default) MAP_EN_H (MAP_EN_H Pipe 0): 0x0
        CHECK_ERR(
            serdes_i2c_write_byte(DES_I2C_ADDR, 0x090D, 0x2C));  // MIPI_TX__0 : MIPI_TX13 | MAP_SRC_0 (MAP_SRC_0 Pipe 0 DT):
                                                                // 0x2C | (Default) MAP_SRC_0 (MAP_SRC_0 Pipe 0 VC): 0x0
        CHECK_ERR(
            serdes_i2c_write_byte(DES_I2C_ADDR, 0x090E, 0x2C));  // MIPI_TX__0 : MIPI_TX14 | MAP_DST_0 (MAP_DST_0 Pipe 0 DT):
                                                                // 0x2C | (Default) MAP_DST_0 (MAP_DST_0 Pipe 0 VC): 0x0
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x090F,
                                    0x00));  // MIPI_TX__0 : MIPI_TX15 | (Default) MAP_SRC_1 (MAP_SRC_1 Pipe 0 DT): 0x0 |
                                                // (Default) MAP_SRC_1 (MAP_SRC_1 Pipe 0 VC): 0x0
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0910,
                                    0x00));  // MIPI_TX__0 : MIPI_TX16 | (Default) MAP_DST_1 (MAP_DST_1 Pipe 0 DT): 0x0 |
                                                // (Default) MAP_DST_1 (MAP_DST_1 Pipe 0 VC): 0x0
        CHECK_ERR(
            serdes_i2c_write_byte(DES_I2C_ADDR, 0x0911, 0x01));  // MIPI_TX__0 : MIPI_TX17 | MAP_SRC_2 (MAP_SRC_2 Pipe 0 DT):
                                                                // 0x1 | (Default) MAP_SRC_2 (MAP_SRC_2 Pipe 0 VC): 0x0
        CHECK_ERR(
            serdes_i2c_write_byte(DES_I2C_ADDR, 0x0912, 0x01));  // MIPI_TX__0 : MIPI_TX18 | MAP_DST_2 (MAP_DST_2 Pipe 0 DT):
                                                                // 0x1 | (Default) MAP_DST_2 (MAP_DST_2 Pipe 0 VC): 0x0
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x092D,
            0x15));  // MIPI_TX__0 : MIPI_TX45 | MAP_DPHY_DEST_0 (MAP_DPHY_DST_0 Pipe 0): 0x1 | MAP_DPHY_DEST_1
                    // (MAP_DPHY_DST_1 Pipe 0): 0x1 | MAP_DPHY_DEST_2 (MAP_DPHY_DST_2 Pipe 0): 0x1
        // Double Mode Configuration
        // serdes_i2c_write_byte(DES_I2C_ADDR, 0x0933, 0x01);
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x0973,
            0x01));  // MIPI_TX__1 : MIPI_TX51 | ALT_MEM_MAP12 (ALT_MEM_MAP12 CTRL1): Alternate memory map enabled
        // MIPI D-PHY Configuration
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x08A0,
                                    0x84));  // MIPI_PHY : MIPI_PHY0 | (Default) phy_4x2 (Port Configuration): 2 (1x4)
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x094A,
                                    0xC0));  // MIPI_TX__1 : MIPI_TX10 | (Default) CSI2_LANE_CNT (Port A - Lane Count): 4
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x08A3,
            0xE4));  // MIPI_PHY : MIPI_PHY3 | (Default) phy0_lane_map (Lane Map - PHY0 D0): Lane 0 | (Default)
                    // phy0_lane_map (Lane Map - PHY0 D1): Lane 1 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 |
                    // (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x08A5,
            0x00));  // MIPI_PHY : MIPI_PHY5 | (Default) phy0_pol_map (Polarity - PHY0 Lane 0): Normal | (Default)
                    // phy0_pol_map (Polarity - PHY0 Lane 1): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 0):
                    // Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal | (Default) phy1_pol_map (Polarity
                    // - PHY1 Clock Lane): Normal
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x0943,
            0x07));  // MIPI_TX__1 : MIPI_TX3 | DESKEW_INIT (Controller 1 Auto Initial Deskew): Disabled
        CHECK_ERR(serdes_i2c_write_byte(
            DES_I2C_ADDR, 0x0944, 0x01));  // MIPI_TX__1 : MIPI_TX4 | DESKEW_PER (Controller 1 Periodic Deskew): Disabled
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1D00, 0xF4));  //  (config_soft_rst_n - PHY1): 0x0
        // This is to set predefined (coarse) CSI output frequency
        // CSI Phy 1 is 800 Mbps/lane.
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1C00, 0xF4));  // (Default)
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1D00, 0xF4));  // (Default)
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0415, 0x20 | (mipi_rate_100mbps & 0x1F)));  // 1500 Mbps/lane
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0418, 0x20 | (mipi_rate_100mbps & 0x1F)));  // 1500 Mbps/lane
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1C00, 0xF5));  // (Default)
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x1D00, 0xF5));  //  | (Default)  (config_soft_rst_n - PHY1): 0x1

        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x08A2,
                                    0x34));  // MIPI_PHY : MIPI_PHY2 | phy_Stdby_n (phy_Stdby_2): Put PHY2 in standby
                                                // mode | phy_Stdby_n (phy_Stdby_3): Put PHY3 in standby mode
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x040B,
                                    0x02));  // BACKTOP : BACKTOP12 | CSI_OUT_EN (CSI_OUT_EN): CSI output enabled

        // GMSL-A Serializer Address Change from 0x80 to SER_A_I2C_ADDR
        // CHECK_ERR(serdes_i2c_write_byte(SER_DEF_I2C_ADDR, 0x0000, SER_A_I2C_ADDR));
        serdes_i2c_write_byte(SER_DEF_I2C_ADDR, 0x0000, SER_A_I2C_ADDR);

        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0002, 0x03));  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled

        //
        // INSTRUCTIONS FOR GMSL-A SERIALIZER MAX96717
        //
        // MIPI D-PHY Configuration
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0330,
                                    0x40));  // 0x00// MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
        CHECK_ERR(
            serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0383, 0x00));  // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0331,
                                    0x30));  // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0332,
                                    0xE0));  // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2
                                                // | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0333,
                                    0x04));  // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0
                                                // | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0334,
                                    0x00));  // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0):
                                                // Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0335,
                                    0x00));  // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0):
                                                // Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal |
                                                // (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal

        // Controller to Pipe Mapping Configuration
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0338, 0x55));
        CHECK_ERR(
            serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0308, 0x64));  // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port
                                                                // B | (Default) START_PORTB (START_PORTB): Enabled
        CHECK_ERR(serdes_i2c_write_byte(
            SER_A_I2C_ADDR, 0x0311, 0x40));  // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0318,
                                    0x6C));  // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x6C
        // Double Mode Configuration
        CHECK_ERR(serdes_i2c_write_byte(
            SER_A_I2C_ADDR, 0x0313, 0x40));  // FRONTTOP : FRONTTOP_11 | bpp12dblz (bpp12dblz): Send 12-bit pixels as 24-bit
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x031E,
                                    0x38));  // FRONTTOP : FRONTTOP_22 | (Default) soft_bppz (soft_bppz): 0x18 |
                                                // soft_bppz_en (soft_bppz_en): Software override enabled

        // Pipe Configuration
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x005B,
                                    0x02));  // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0

        // Video Transmit Configuration for Serializer(s)
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x0002, 0x43));  // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled

#if 1
        //
        // MFPConfigurationTool
        //
        // Disable default functions on utilized pins.
        CHECK_ERR(serdes_i2c_write_byte(
            SER_A_I2C_ADDR, 0x0005, 0x00));  // Set ERRB_EN to 0x0 (to disable ERRB) | Set LOCK_EN to 0x0 (to disable LOCK)
        // MAX96724 (0x4E), GPIO3 block
        // CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0309, 0x84)); // Set GPIO_OUT_DIS to 0x0 | Set GPIO_TX_EN to 0x0 |
        // Set GPIO_RX_EN to 0x1 | Set GPIO_IN to 0x0 | Set GPIO_OUT to 0x0 CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR,
        // 0x030A, 0xA3)); // Set OUT_TYPE to 0x1 | Set PULL_UPDN_SEL to 0x2 CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR,
        // 0x030B, 0x43)); // Set GPIO_RX_ID to 0x3
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0300,
                                    0x9B));  // Set GPIO_OUT_DIS to 0x1 | Set GPIO_TX_EN to 0x1 | Set GPIO_RX_EN to 0x0 |
                                                // Set GPIO_IN to 0x1 | Set GPIO_OUT to 0x1
        CHECK_ERR(serdes_i2c_write_byte(DES_I2C_ADDR, 0x0301,
                                    0x05));  // Set OUT_TYPE to 0x1 | Set PULL_UPDN_SEL to 0x2 | Set GPIO_TX_ID to 0x5

        // MAX96717 (0x82), GPIO3 block
        // CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x02C7, 0x9B)); // Set GPIO_OUT_DIS to 0x1 | Set GPIO_TX_EN to 0x1
        // | Set GPIO_RX_EN to 0x0 | Set GPIO_IN to 0x1 | Set GPIO_OUT to 0x1 CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR,
        // 0x02C8, 0xA3)); // Set OUT_TYPE to 0x1 | Set PULL_UPDN_SEL to 0x2 | Set GPIO_TX_ID to 0x3
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x02CD,
                                    0x84));  // Set GPIO_OUT_DIS to 0x0 | Set GPIO_TX_EN to 0x0 | Set GPIO_RX_EN to 0x1 |
                                                // Set GPIO_IN to 0x0 | Set GPIO_OUT to 0x0
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x02CE, 0x05));  // Set OUT_TYPE to 0x1 | Set PULL_UPDN_SEL to 0x2
        CHECK_ERR(serdes_i2c_write_byte(SER_A_I2C_ADDR, 0x02CF, 0x45));  // Set GPIO_RX_ID to 0x5
#endif
        FS_LOGD("serdes config successful");
    }
    else {
        FS_LOGD("serdes not enabled, skipped");
        err = FS_ERR_NO_DEVICE;
    }

    return err;
}

int serdes_update_rate(uint8_t mipi_rate_100mbps)
{
    // FIXME: Only the necessary registers for changing the MIPI rate should be configured, rather than reinitializing the entire Serdes
    return serdes_init(mipi_rate_100mbps);
}

void serdes_deinit(void)
{
    return;
}
