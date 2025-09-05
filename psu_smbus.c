#include <base/oserror.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "hal_utils_inner.h"
#include "hal_i2c.h"
#include "hal_hwinfo.h"
#include "psu.h"


static psu_object_t *alloc_psu_smb()
{
    psu_object_t *psu = calloc(sizeof(psu_object_t), 1);
    ASSERT_FR(psu, NULL, "malloc fail!");

    char devname[HAL_NAME_MAX] = { 0 };
    snprintf(devname, sizeof(devname), "/dev/i2c-%d", i2c_get_bus());
    hal_smbus_t *smb = hal_smbus_alloc(devname, 0, 0);
    ASSERT_FG(smb, fail, "smbus init fail!");

    psu->smb = smb;

    return psu;
fail:
    free(psu);
    return NULL;
}

static void free_psu_smb(psu_object_t *psu)
{
    if (!psu || !psu->smb) {
        HAL_DBG("free_psu_smb fail!");
        return;
    }

    hal_smbus_t *smb = psu->smb;
    smb->free(smb);
    free(psu);
}

#define PSU_OULUTONG_REG_ADDR     0x79
#define PSU_OULUTONG_POWER1_SLAVE 0x58
#define PSU_OULUTONG_POWER2_SLAVE 0x59
#define PSU_OULUTONG_POWER_REG    0x03
#define PSU_OULUTONG_POWER1_VALUE 0x46
#define PSU_OULUTONG_POWER2_VALUE 0x6c

static int psu_smb_read_word(psu_object_t *psu, uint32_t idx)
{
    ASSERT_FR(psu && psu->smb && idx < PSU_NUM, -OS_EINVAL, "Invalid argument");
    int slave = psu->reg[idx].slave;
    int addr = psu->reg[idx].addr;
    hal_smbus_t *smb = psu->smb;
    uint16_t status = 0;

    int ret = smb->read_word(smb, slave, addr, &status);
    ASSERT_FR(!ret, -1, "Smbus read power(%d) status fail!", psu->type);
    return status;
}

static int psu_luma_oulutong_status(psu_object_t *psu, uint32_t idx)
{
    int psu_status = 0;
    int status = psu_smb_read_word(psu, idx);
    if (status < 0)
        return status;

    int value = (idx == 0 ? PSU_OULUTONG_POWER1_VALUE : PSU_OULUTONG_POWER2_VALUE);
    /* 如果是0x2008，供应商的解释是电源发生过一些输入不稳定，或其他内部错误。先清理状态后再次判断 */
    if (0x2008 == (status & 0xffff)) {
        psu->smb->write_r(psu->smb, psu->reg[idx].slave, PSU_OULUTONG_POWER_REG, &value, 1);
        psu_status = HAL_PSU_STAT_ON;
    } else if (0x0 == (status & 0xff)) {
        psu_status = HAL_PSU_STAT_ON;
    } else {
        psu_status = HAL_PSU_STAT_OFF;
    }
    return psu_status;
}

static psu_object_t *alloc_psu_luma_oulutong()
{
    psu_object_t *psu = alloc_psu_smb();
    if (!psu)
        return psu;

    psu->reg[0].slave = PSU_OULUTONG_POWER1_SLAVE;
    psu->reg[0].addr = PSU_OULUTONG_REG_ADDR;

    psu->reg[1].slave = PSU_OULUTONG_POWER2_SLAVE;
    psu->reg[1].addr = PSU_OULUTONG_REG_ADDR;

    psu->type = HAL_PSU_OULUTONG;
    psu->free = free_psu_smb;
    psu->status = psu_luma_oulutong_status;
    return psu;
}

#define PSU_OULUTONG_POUT_REG 0x96
#define PSU_OULUTONG_PIN_REG  0x97

static int psu_xeme_oulutong_status(psu_object_t *psu, uint32_t idx)
{
    int status = psu_smb_read_word(psu, idx);
    if (status < 0)
        return status;

    if (HAL_BIT(6) & status)
        return HAL_PSU_STAT_OFF;
    else
        return HAL_PSU_STAT_ON;
}

static double psu_oulutong_lineal_value(psu_object_t *psu, int slave, int reg)
{
    ASSERT_FR(psu && psu->smb, -OS_EINVAL, "Invalid argument");

    hal_smbus_t *smb = psu->smb;
    uint16_t d16;
    int ret = smb->read_word(smb, slave, PSU_OULUTONG_REG_ADDR, &d16);
    ASSERT_FR(!ret, -1, "Smbus read power(oulutong 150w) power value fail!");

    return psu_lineal_value(d16);
}

static double psu_oulutong_pin(psu_object_t *psu, uint32_t idx)
{
    int slave = (idx == 0 ? PSU_OULUTONG_POWER1_SLAVE : PSU_OULUTONG_POWER2_SLAVE);
    return psu_oulutong_lineal_value(psu, slave, PSU_OULUTONG_PIN_REG);
}

static double psu_oulutong_pout(psu_object_t *psu, uint32_t idx)
{
    int slave = (idx == 0 ? PSU_OULUTONG_POWER1_SLAVE : PSU_OULUTONG_POWER2_SLAVE);
    return psu_oulutong_lineal_value(psu, slave, PSU_OULUTONG_POUT_REG);
}

static psu_object_t *alloc_psu_xeme_oulutong()
{
    psu_object_t *psu = alloc_psu_smb();
    if (!psu)
        return psu;

    psu->reg[0].slave = PSU_OULUTONG_POWER1_SLAVE;
    psu->reg[0].addr = PSU_OULUTONG_REG_ADDR;

    psu->reg[1].slave = PSU_OULUTONG_POWER2_SLAVE;
    psu->reg[1].addr = PSU_OULUTONG_REG_ADDR;

    psu->type = HAL_PSU_OULUTONG;
    psu->free = free_psu_smb;
    psu->status = psu_xeme_oulutong_status;
    psu->pin = psu_oulutong_pin;
    psu->pout = psu_oulutong_pout;

    return psu;
}

#define PSU_TAIDA_SLAVE 0x25
#define PSU_TAIDA_REG   0xe0
#define PSU_TAIDA_POUT  0x96

static int psu_taida_status(psu_object_t *psu, uint32_t idx)
{
    int status = psu_smb_read_word(psu, idx);
    if (status < 0)
        return status;

    uint8_t flag1 = status & (idx == 0 ? HAL_BIT(5) : HAL_BIT(4));
    uint8_t flag2 = status & (idx == 0 ? HAL_BIT(2) : HAL_BIT(1));

    // N/A 状态
    if (!flag2)
        return HAL_PSU_STAT_NA;
    // off 状态
    if (flag1)
        return HAL_PSU_STAT_OFF;

    return HAL_PSU_STAT_ON;
}

static double psu_taida_pout(psu_object_t *psu, uint32_t idx)
{
    ASSERT_FR(psu && psu->smb && idx == 0, -OS_EINVAL, "Invalid argument");
    hal_smbus_t *smb = psu->smb;
    uint16_t data = 0;
    int ret = smb->read_word(smb, PSU_TAIDA_SLAVE, PSU_TAIDA_POUT, &data);
    ASSERT_FR(!ret, 0, "Smbus read power(taida) status fail!");

    return psu_lineal_value(data);
}

static psu_object_t *alloc_psu_taida()
{
    psu_object_t *psu = alloc_psu_smb();
    if (!psu)
        return psu;

    psu->reg[0].slave = PSU_TAIDA_SLAVE;
    psu->reg[0].addr = PSU_TAIDA_REG;

    psu->reg[1].slave = PSU_TAIDA_SLAVE;
    psu->reg[1].addr = PSU_TAIDA_REG;

    psu->type = HAL_PSU_TAIDA;
    psu->free = free_psu_smb;
    psu->status = psu_taida_status;
    psu->pout = psu_taida_pout;
    return psu;
}

static psu_match_t psu_match_table[] = {
    {
        .family = { "sxf", "tina" },
        .match = { alloc_psu_taida, alloc_psu_luma_oulutong, NULL },
    },
    {
        .family = { "sxf", "tina1" },
        .match = { alloc_psu_taida, alloc_psu_luma_oulutong, NULL },
    },
    {
        .family = { "sxf", "mona" },
        .match = { alloc_psu_taida, alloc_psu_luma_oulutong, NULL },
    },
    {
        .family = { "sxf", "xeme" },
        .match = { alloc_psu_taida, alloc_psu_xeme_oulutong, NULL },
    },
    {
        .family = { "sxf", "dota" },
        .match = { alloc_psu_taida, alloc_psu_xeme_oulutong, NULL },
    },
    {
        .family = { "sxf", "dota1" },
        .match = { alloc_psu_taida, alloc_psu_xeme_oulutong, NULL },
    },
};

void psu_smbus_register(void)
{
    psu_register(psu_match_table, HAL_ARRSZ(psu_match_table));
}