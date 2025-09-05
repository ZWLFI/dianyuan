#include <unistd.h>
#include <sys/io.h>
#include <fcntl.h>
#include "hal_utils_inner.h"
#include "psu.h"

static void free_kuka_priv(psu_object_t *psu)
{
    if (!psu) {
        HAL_DBG("free_kuka_priv fail!");
        return;
    }
    FREE(psu);
}

// io权限等级
#define PSU_IO_PRIVILEGE_3 3
#define PSU_IO_PRIVILEGE_0 0

static int psu_kuka_status(psu_object_t *psu, uint32_t idx)
{
    ASSERT_FR(psu && idx < PSU_NUM, -OS_EINVAL, "Invalid argument");

    // 欧陆通60w冗余电源，无法检测哪个电源在位，所以默认显示第一个电源在位
    if (idx == 0)
        return HAL_PSU_STAT_ON;

    // 提高io权限等级
    int ret = iopl(PSU_IO_PRIVILEGE_3);
    ASSERT_FR(ret >= 0, -1, "Set the I/O privilege level 3 fail!");

    uint8_t data = 0;
	
    // 需要解锁两次
	outb(0x87, 0x4E);
	outb(0x87, 0x4E);
	
    outb(0x07, 0x4E);  //logic 寄存器 0x07
	outb(0x09, 0x4F);  //logic 9

    outb(0x30, 0x4E);  // logic 9 CR30  sio的GP56 active
    data = inb(0x4F);
	outb((data | HAL_BIT(3)), 0x4F);
	
	outb(0xEB, 0x4E);   //Multi-function[0xEB].bit6 output type
    data = inb(0x4F);
	outb((data | HAL_BIT(6)), 0x4F);

    outb(0xF5, 0x4E);   //GP56 data register
	data = inb(0x4F);

    outb(0xAA, 0x4E);//加锁

    // 降低io权限等级
    ret = iopl(PSU_IO_PRIVILEGE_0);
    if (ret < 0)
        HAL_DBG("Set the I/O privilege level 0 fail!");

    // 两个电源都在位
    if (data & HAL_BIT(6)) {
        return HAL_PSU_STAT_ON;
    }

    return HAL_PSU_STAT_OFF;
}

static psu_object_t *alloc_psu_kuka_60w()
{
    psu_object_t *psu = calloc(sizeof(psu_object_t), 1);
    ASSERT_FR(psu, NULL, "malloc fail!");
    psu->type = HAL_PSU_OULUTONG;
    psu->free = free_kuka_priv;
    psu->status = psu_kuka_status;
    return psu;
}

static psu_match_t psu_match_table[] = {
    {
        .family = { "sxf", "kuka", "820_1_1" },
        .product_model = "sta-100-b2200",
        .match = { alloc_psu_kuka_60w, NULL },
    },
    {
        .family = { "sxf", "kuka", "820_1_1" },
        .product_model = "sta-100-b2300",
        .match = { alloc_psu_kuka_60w, NULL },
    },
};

void psu_ioport_register(void)
{
    psu_register(psu_match_table, HAL_ARRSZ(psu_match_table));
}