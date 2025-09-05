#include <base/oserror.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include "hal_utils_inner.h"
#include "psu.h"

#define PSU_KUKA_PATH     "/sys/devices/pci0000:00/0000:00:1f.1"
#define PSU_KUKA_RESOURCE PSU_KUKA_PATH "/resource"

#define PSU_GPIO_POWER1      0xC504A8
#define PSU_GPIO_POWER2      0xC504B0
#define PSU_GPIO_POWER_WRITE 0x45000100

#define PSU_MAP_SIZE 4096UL
#define PSU_MAP_MASK (PSU_MAP_SIZE - 1)

typedef struct {
    int fd;
    void *map_base;
    uint64_t start_addr;
} kuka_priv_t;

static int base_addr_iter(const char *line, size_t size, void *priv)
{
    uint64_t *addr = (uint64_t *)priv;
    char *endptr = NULL;

    *addr = strtoull(line, &endptr, 16);
    if (endptr == line)
        return -1;

    return 1;
}

static int alloc_kuka_priv(psu_object_t *psu)
{
    ASSERT_FR(hal_path_exist(PSU_KUKA_RESOURCE), -EINVAL, "pci not exist!");

    uint64_t base_addr = 0;
    int ret = hal_eachline(base_addr_iter, &base_addr, PSU_KUKA_RESOURCE);
    ASSERT_FR(ret >= 0, -EINVAL, "read base addr fail");

    psu->start_addr = base_addr;

    psu->fd = open("/dev/mem", O_RDWR | O_SYNC);
    ASSERT_FR(psu->fd >= 0, -EINVAL, "open /dev/mem fail");

    psu->map_base = mmap(0, PSU_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                         psu->fd, (base_addr + PSU_GPIO_POWER1) & ~PSU_MAP_MASK);
    ASSERT_FG(psu->map_base != (void *)-1, mmap_fail, "mmap fail");

    void *virt_addr = (void *)((char *)psu->map_base + ((psu->start_addr + PSU_GPIO_POWER2) & PSU_MAP_MASK));
    *((unsigned int *)virt_addr) = PSU_GPIO_POWER_WRITE;

    return 0;
mmap_fail:
    close(psu->fd);
    return -EINVAL;
}

static void free_kuka_priv(psu_object_t *psu)
{
    if (!psu) {
        HAL_DBG("free_kuka_priv fail!");
        return;
    }
    munmap(psu->map_base, PSU_MAP_SIZE);
    close(psu->fd);
    FREE(psu);
}

static int psu_kuka_status(psu_object_t *psu, uint32_t idx)
{
    ASSERT_FR(psu && idx < PSU_NUM, -OS_EINVAL, "Invalid argument");

    uint32_t data1 = 0, data2 = 0;

    void *virt_addr = (void *)((char *)psu->map_base + ((psu->start_addr + PSU_GPIO_POWER1) & PSU_MAP_MASK));
    data1 = *((uint32_t *)virt_addr);

    virt_addr = (void *)((char *)psu->map_base + ((psu->start_addr + PSU_GPIO_POWER2) & PSU_MAP_MASK));
    data2 = *((uint32_t *)virt_addr);

    if (idx) {
        // 第一个bit为1时电源2没上电
        if (data2 & HAL_BIT(1))
            return HAL_PSU_STAT_OFF;
        else
            return HAL_PSU_STAT_ON;
    }

    // 两个电源都上电时，第一位都为0

    if (!(data1 & HAL_BIT(1)) && !(data1 & HAL_BIT(1)))
        return HAL_PSU_STAT_ON;

    if ((data1 & HAL_BIT(1)) && (data2 & HAL_BIT(1)))
        return HAL_PSU_STAT_ON;

    return HAL_PSU_STAT_OFF;
}

static psu_object_t *alloc_psu_kuka()
{
    psu_object_t *psu = calloc(sizeof(psu_object_t), 1);
    ASSERT_FR(psu, NULL, "malloc fail!");

    int ret = alloc_kuka_priv(psu);
    ASSERT_FG(!ret, fail, "alloc kuka priv fail!");
    psu->type = HAL_PSU_TAIDA;
    psu->free = free_kuka_priv;
    psu->status = psu_kuka_status;
    return psu;
fail:
    free(psu);
    return NULL;
}

static psu_match_t psu_match_table[] = {
    {
        .family = { "sxf", "kuka", "820_1_1" },
        .match = { alloc_psu_kuka, NULL },
    },
};

void psu_mmap_register(void)
{
    psu_register(psu_match_table, HAL_ARRSZ(psu_match_table));
}