#include <base/oserror.h>
#include <stdlib.h>
#include <stdio.h>
#include "hal_utils_inner.h"
#include "hal_hwinfo.h"
#include "psu.h"

#define family_ok(_f1, _f2)                                                \
    ({                                                                     \
        int _ok = 0;                                                       \
        const char *str = hal_get_hwinfo(_f2) ?: "";                       \
        if (_f1 == NULL || _f1[0] == '\0' || !strcmp(_f1, HAL_FAMILY_ALL)) \
            _ok = 1;                                                       \
        else if (!strcmp(_f1, str))                                        \
            _ok = 1;                                                       \
        _ok;                                                               \
    })

static bool family_match(hal_family_t *family)
{
    if (!family_ok(family->vendor, HAL_DATA_HW_VENDOR))
        return false;
    if (!family_ok(family->platform, HAL_DATA_HW_PLATFORM))
        return false;
    if (!family_ok(family->model, HAL_DATA_HW_MODEL))
        return false;
    return true;
}

static bool product_model_match(const char *model)
{
    const char *str = hal_get_hwinfo(HAL_DATA_HW_PRODUCT_MODEL);
    return strcmp(str, model) == 0;
}

#define PSU_MATCH_TABLE_MAX 32
static psu_match_t *psu_match_table[PSU_MATCH_TABLE_MAX];
static int psu_match_table_size;

void psu_register(psu_match_t *match, int size)
{
    for (int i = 0; i < size; i++) {
        if (psu_match_table_size >= PSU_MATCH_TABLE_MAX) {
            HAL_DBG("match table out of range");
            return;
        }
        psu_match_table[psu_match_table_size++] = &match[i];
    }
}

static psu_object_t *psu_try_alloc(psu_alloc_t *alloc)
{
    psu_object_t *psu = NULL;
    // 遍历alloc函数
    while (*alloc) {
        psu = (*alloc)();
        if (psu) {
            // 测试是否能获取到电源状态
            if (psu_status(psu, 0) >= 0)
                return psu;
            psu->free(psu);
        }
        ++alloc;
    }
    return NULL;
}

static psu_match_t *psu_get_match_next(psu_match_t *match, int match_model)
{
    int idx = 0;

    /* 1. 先找到当前index */
    if (match) {
        for (int i = 0; i < psu_match_table_size; ++i) {
            psu_match_t *t = psu_match_table[i];
            if (t == match) {
                idx = i + 1;
                break;
            }
        }
    }

    /* 2. 根据上次的idx, 找到下一个匹配 */
    for (int i = idx; i < psu_match_table_size; ++i) {
        psu_match_t *t = psu_match_table[i];

        if (match_model) {
            const char *model = t->product_model;
            if (!model || !product_model_match(model))
                continue;
        }

        if (family_match(&t->family))  
            return t;
    }

    return NULL;
}

psu_object_t *psu_alloc(void)
{
    psu_object_t *psu = NULL;
    psu_match_t *t = NULL;
    int is_product_model = false;

    // 1.先匹配带product_model
    do {
        t = psu_get_match_next(t, 1);
        if (!t)
            break;
        is_product_model = true;
        HAL_DBG("psu alloc by product_model");
        psu = psu_try_alloc(t->match);
        if (psu)
            return psu;
    } while(t);

    // 如果已经匹配到product_model就直接返回不继续匹配
    if (is_product_model)
        return NULL;

    // 2. 再匹配不带product_model
    t = NULL;
    do {
        t = psu_get_match_next(t, 0);
        if (!t)
            break;
        psu = psu_try_alloc(t->match);
        if (psu)
            return psu;
    } while(t);

    return NULL;
}

void psu_free(psu_object_t *psu)
{
    if (psu)
        psu->free(psu);
}

int psu_status(psu_object_t *psu, uint32_t idx)
{
    return psu->status ? psu->status(psu, idx) : -OS_EINVAL;
}

double psu_power_input(psu_object_t *psu, uint32_t idx)
{
    return psu->pin ? psu->pin(psu, idx) : 0;
}

double psu_power_output(psu_object_t *psu, uint32_t idx)
{
    return psu->pout ? psu->pout(psu, idx) : 0;
}

hal_psu_type_e psu_type(psu_object_t *psu)
{
    return psu ? psu->type : HAL_PSU_UNKNOW;
}

typedef void(*register_fun_t)(void);

__attribute__((constructor)) 
static void psu_register_construct(void) 
{
    register_fun_t reg[] = {
        psu_mmap_register,
        psu_smbus_register,
#if defined __x86_64__ || defined __i386__
        psu_ioport_register,
#endif
    };

    for (int i = 0; i < HAL_ARRSZ(reg); ++i)
        reg[i]();
}