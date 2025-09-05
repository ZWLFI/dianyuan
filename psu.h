#ifndef __SXF_PSU_H__
#define __SXF_PSU_H__

#include "hal_utils.h"
#include "hal_sensor.h"

#define PSU_NUM 2        // 支持电源个数
#define PSU_SET_STAT(_buf, _id, _stat)   \
    do {                                 \
        uint8_t *__buf = _buf;           \
        uint8_t __stat = _stat;          \
        int __idx = psu_stat_index(_id); \
        if (__idx >= 0)                  \
            __buf[__idx] = __stat;       \
    } while (0)
#define PSU_GET_STAT(_buf, _id) ({   \
    uint8_t *__buf = _buf;           \
    int __idx = psu_stat_index(_id); \
    (__idx >= 0) ? __buf[__idx] : 0; \
})

static inline int psu_stat_index(int id)
{
    switch (id) {
    case HAL_SEN_PSU_PIN1:
    case HAL_SEN_PSU_POUT1:
    case HAL_SEN_PSU_STATUS1:
        return 0;

    case HAL_SEN_PSU_PIN2:
    case HAL_SEN_PSU_POUT2:
    case HAL_SEN_PSU_STATUS2:
        return 1;

    default:
        return -1;
    }
}

static inline double psu_lineal_value(uint32_t value)
{
    int ly = value & 0x7FF;         // lineal_y
    int ln = value & 0xF800;        // lineal_n

    if (value & 0x400)
        ly -= 1 << 11;

    if (value & 0x8000)
        ln = (ln >> 11) - (1 << 5);

    double result = ly;
    int abs = ln > 0 ? ln : -ln;

    while (abs--)
        result *= (ln >= 0) ? 2.0 : 0.5;

    return result;
}

typedef enum {
    HAL_PSU_TAIDA,          // 台达
    HAL_PSU_OULUTONG,       // 欧陆通
    HAL_PSU_QUANHAN,        // 全汉
    HAL_PSU_UNKNOW,
} hal_psu_type_e;

typedef struct psu_reg_t {
    uint8_t slave;
    uint8_t addr;
} psu_reg_t;

typedef struct psu_object_t {
    hal_psu_type_e type;        // 电源类型
    void (*free)(struct psu_object_t *psu);
    int (*status)(struct psu_object_t *psu, uint32_t idx);         // 电源状态
    double (*pin)(struct psu_object_t *psu, uint32_t idx);         // 输入功率
    double (*pout)(struct psu_object_t *psu, uint32_t idx);        // 输出功率
    union {
        struct {
            hal_smbus_t *smb;
            psu_reg_t reg[PSU_NUM];
        };
        // kuka 电源的私有变量
        struct {
            int fd;
            void *map_base;
            uint64_t start_addr;
        };
    };

} psu_object_t;

#define PSU_ALLOC_FUN_MAX    5        // 最多有几个alloc函数
typedef psu_object_t *(*psu_alloc_t)();
typedef struct {
    hal_family_t family;
    const char *product_model;
    psu_alloc_t match[PSU_ALLOC_FUN_MAX];
} psu_match_t;

/**
 * @description: 申请 psu_object_t 对象
 * @return {psu_object_t*} psu句柄
 */
psu_object_t *psu_alloc(void);
/**
 * @description: 释放句柄
 * @param {psu_object_t*} psu : psu的句柄
 */
void psu_free(psu_object_t *psu);
/**
 * @description: 获取电源状态
 * @param {psu_object_t*} psu : psu的句柄
 * @param {uint32_t} idx: 第几个电源，从0开始
 * @return {double} 成功: HAL_PSU_STAT_ON/HAL_PSU_STAT_OFF/HAL_PSU_STAT_NA, 失败: -errno
 */
int psu_status(psu_object_t *psu, uint32_t idx);
/**
 * @description: 获取输入功率
 * @param {psu_object_t*} psu : psu的句柄
 * @param {uint32_t} idx: 第几个电源，从0开始
 * @return {double} 输入功率
 */
double psu_power_input(psu_object_t *psu, uint32_t idx);
/**
 * @description: 获取输出功率
 * @param {psu_object_t*} psu : psu的句柄
 * @param {uint32_t} idx: 第几个电源，从0开始
 * @return {double} 输出功率
 */
double psu_power_output(psu_object_t *psu, uint32_t idx);
/**
 * @description: 获取电源类型
 * @param {psu_object_t*} psu : psu的句柄
 * @return {double} 电源类型，例如台达，欧陆通，全汉等
 */
hal_psu_type_e psu_type(psu_object_t *psu);
/**
 * @description: 注册电源匹配规则
 * @param {psu_match_t**} match: 匹配规则
 * @param {size} size: 规则长度
 */
void psu_register(psu_match_t *match, int size);
void psu_mmap_register(void);
void psu_smbus_register(void);
#if defined __x86_64__ || defined __i386__
void psu_ioport_register(void);
#endif

#endif