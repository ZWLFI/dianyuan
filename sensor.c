#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <base/oserror.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "hal.h"
#include "hal_utils_inner.h"
#include "hal_sensor.h"
#include "psu.h"
#include "hal_i2c.h"
#include "yudi.h"
#include "hal_protocol.h"

#define SENSOR_SLAVE        0x40    // 默认的sensor设备地址
#define EXT_REG_ADDR        0x00    // 切换到扩展寄存器的地址
#define EXT_REG_DATA        0x01    // 切换到扩展寄存器要写入值

// 电源相关
#define CRPS_SLAVE          0x70    // 切换到CRPS通路的设备地址
#define CRPS_REG_ADDR       0x00    // 切换到CRPS通路的设备寄存器地址
#define CRPS_REG_DATA       0x20    // 切换到CRPS通路的设备寄存器要写入值
#define PSU1_ADDR           0x58    // PSU1的设备地址
#define PSU2_ADDR           0x59    // PSU2的设备地址
#define PSU_REG_ADDR        0x9a    // PSU model的寄存器地址
#define PSU_MAX_MODEL_LEN   14      // 读取PSU型号的最大长度
#define PSU_UNKNOWN_MODEL   "Unknown"

typedef struct {
    hal_sensor_id_e id;            // 传感器的名称id
    hal_sensor_type_e type;        // 传感器的类型
    uint8_t offset_h;              // 读取寄存器高字节偏移量
    uint8_t offset_l;              // 读取寄存器低字节偏移量
    double factor;                 // 读取后的系数
    uint8_t slave;                 // 传感器设备地址
    double min;                    // 传感器的最小值
    double max;                    // 传感器的最大值
} sensor_object_t;

typedef struct {
    psu_object_t *psu;
    char psu1_model[PSU_MAX_MODEL_LEN];
    char psu2_model[PSU_MAX_MODEL_LEN];
    uint8_t status[PSU_NUM];
    hal_smbus_t *smb;
    // 连接Switch fan 和 Netcard fan的总线
    hal_smbus_t *smb_fan;
    size_t obj_num;
    sensor_object_t objs[];
} sensor_drv_t;

typedef struct psu_model_info_t {
    const char *name;
    uint32_t psu_in_max_watts;
    uint32_t psu_out_max_watts;
} psu_model_info_t;

static psu_model_info_t psu_model_info[] = {
    { "CRPS350S", 720, 350 },
    { "DPS-300AB-102", 960, 300 },
    { "U1D-D0550-B", 1440, 550 },
};

static inline int sensor_write_byte(hal_smbus_t *smb, sensor_object_t *obj, uint8_t offset, uint8_t data)
{
    int ret;
    if (obj->slave != SENSOR_SLAVE) { // CPLD
        ret = smb->write_r(smb, obj->slave, offset, &data, 1);
        ASSERT_FR(ret == 1, -1, "CPLD write failed! slave: 0x%x, offset: 0x%x", obj->slave, offset);
    } else { // MCU
        hal_proto_t *hp = hal_proto_alloc(smb, SENSOR_SLAVE, MCU_PROTO_V1);
        ASSERT_FR(hp, -1, "MCU proto alloc for write failed.");
        ret = hal_proto_write(hp, offset, &data, 1);
        hal_proto_free(hp);
        ASSERT_FR(ret == 0, -1, "MCU write failed! offset: 0x%x", offset);
    }
    return 0;
}

static inline int sensor_read_byte(hal_smbus_t *smb, sensor_object_t *obj, uint8_t offset, uint8_t *val)
{
    int ret;
    if (obj->slave != SENSOR_SLAVE) { // CPLD
        ret = smb->read_r(smb, obj->slave, offset, val, 1);
        ASSERT_FR(ret == 1, -1, "CPLD read failed! slave: 0x%x, offset: 0x%x", obj->slave, offset);
    } else { // MCU
        hal_proto_t *hp = hal_proto_alloc(smb, SENSOR_SLAVE, MCU_PROTO_V1);
        ASSERT_FR(hp, -1, "MCU proto alloc for read failed.");

        ret = hal_proto_read(hp, offset, val, 1);
        hal_proto_free(hp);
        ASSERT_FR(ret == 0, -1, "MCU read failed! offset: 0x%x", offset);
    }
    return 0;
}


static inline int switch_to_extend_register(hal_smbus_t *smb, sensor_object_t *obj)
{
    // 如果是SWITCH_FAN或者NETCARD_FAN, 不需要切扩展寄存器
    if(obj->id == HAL_SEN_FAN_SWITCH || obj->id == HAL_SEN_FAN_NETCARD)
        return 0;
    
    return sensor_write_byte(smb, obj, EXT_REG_ADDR, EXT_REG_DATA);
}

static int sensor_get_val(hal_smbus_t *smb, sensor_object_t *obj, uint8_t offset, uint8_t *val)
{
    ASSERT_FR(smb && obj && val, -1, "Invalid argument");
    
    int ret = switch_to_extend_register(smb, obj);
    ASSERT_FR(ret == 0, -1, "switch to extend register failed!");

    ret = sensor_read_byte(smb, obj, offset, val);
    ASSERT_FR(ret == 0, -1, "sensor get val failed!");

    return 0;
}

static double sensor_get_temp(sensor_drv_t *drv, sensor_object_t *obj)
{
    hal_smbus_t *smb = drv->smb;
    uint8_t temp = 0;
    int ret = sensor_get_val(smb, obj, obj->offset_l, &temp);
    ASSERT_FR(ret == 0, 0, "smbus read temp failed!");
    
    return temp * obj->factor;
}

static double sensor_get_vol_fan(sensor_drv_t *drv, sensor_object_t *obj)
{
    hal_smbus_t *smb = drv->smb;
    // 如果是SWITCH_FANAN或者NETCARD_FAN, 需要使用另一条总线
    if(obj->id == HAL_SEN_FAN_SWITCH || obj->id == HAL_SEN_FAN_NETCARD)
        smb = drv->smb_fan;

    uint8_t val_l = 0, val_h = 0;
    int ret = sensor_get_val(smb, obj, obj->offset_l, &val_l);
    ASSERT_FR(ret == 0, 0, "smbus read vol low addr failed!");

    ret = sensor_get_val(smb, obj, obj->offset_h, &val_h);
    ASSERT_FR(ret == 0, 0, "smbus read vol high addr failed!");

    return (val_h*16*16 + val_l) * obj->factor;
}

static inline int switch_to_crps(hal_smbus_t *smb)
{
#ifdef xtest
    return 1;
#endif
    uint8_t data = CRPS_REG_DATA;
    return smb->write_r(smb, CRPS_SLAVE, CRPS_REG_ADDR, &data, 1);
}

static int sensor_get_psu_model(sensor_drv_t *drv, hal_smbus_t *smb, uint32_t psu_addr)
{
    // 1. 先切换CRPS通路
    int ret = switch_to_crps(smb);
    ASSERT_FR(ret == 1, -1, "switch to CRPS failed!");

    // 2. 向对应的PSU的地址设置寄存器地址， 然后读取14个字节
    char *psu_model = psu_addr == PSU1_ADDR ? drv->psu1_model : drv->psu2_model;
    ret = smb->rblock(smb, psu_addr, PSU_REG_ADDR, (uint8_t *)psu_model, PSU_MAX_MODEL_LEN);
    // 获取失败默认设置为PSU_UNKNOWN_MODEL， 后面做进一步处理
    if (ret != PSU_MAX_MODEL_LEN) {
        HAL_DBG("get PSU model failed, set to CRPS350S");
        strcpy(psu_model, PSU_UNKNOWN_MODEL);
        return 0;
    }

    // 3. 将读取到的14个字节转换成字符串
    for (int i = 0; i < PSU_MAX_MODEL_LEN; i++) {
        if (psu_model[i] == '#') {
            psu_model[i] = '\0';
            break;
        }
    }

    return 0;
}

static int sensor_get_psu_max_watts(char *model, int out)
{
#ifdef xtest
    return 1;
#endif
    for (unsigned int i = 0; i < HAL_ARRSZ(psu_model_info); i++) {
        if (strstr(model, psu_model_info[i].name)) {
            if (out)
                return psu_model_info[i].psu_out_max_watts;
            else
                return psu_model_info[i].psu_in_max_watts;
        }
    }

    return -1;
}

static int sensor_get_psu_in_max_watts(char *model)
{
    return sensor_get_psu_max_watts(model, 0);
}

static int sensor_get_psu_out_max_watts(char *model)
{
    return sensor_get_psu_max_watts(model, 1);
}


static int sensor_get_psu_status(psu_object_t *psu, sensor_object_t *obj)
{
    hal_smbus_t *smb = psu->smb;
    uint16_t val = 0;
    // 1、先切换到CRPS通路
    int ret = switch_to_crps(smb);
    ASSERT_FR(ret == 1, -1, "switch to CRPS failed!");

    // 2、获取状态
    ret = smb->read_word(smb, obj->slave, obj->offset_l, &val);
    if (ret != 0) {
        HAL_DBG("Smbus read power status fail, setting psu offline");
        return HAL_PSU_STAT_OFF;
    }

    // 3、通过POWER_GOOD# bit位确认是否在位，0表示在位，1表示不在位
    return (HAL_BIT(11) & val) ? HAL_PSU_STAT_OFF : HAL_PSU_STAT_ON;
}

static int sensor_get_psu_watts(psu_object_t *psu, sensor_object_t *obj)
{
    hal_smbus_t *smb = psu->smb;
    uint16_t val = 0;
    // 1、先切换到CRPS通路
    int ret = switch_to_crps(smb);
    ASSERT_FR(ret == 1, -1, "switch to CRPS failed!");

    // 2、获取功率
    ret = smb->read_word(smb, obj->slave, obj->offset_l, &val);
    ASSERT_FR(!ret, -1, "Smbus read power watts fail!");

    return val;
}

static void sensor_info(sensor_drv_t *drv, size_t num, hal_data_t *data)
{
    sensor_object_t *obj = drv->objs + num;
    double value = 0.0;

    switch (obj->type) {
    case HAL_SEN_TEMP:
        value = sensor_get_temp(drv, obj);
        break;
    case HAL_SEN_FAN:
    case HAL_SEN_VOL:
        value = sensor_get_vol_fan(drv, obj);
        break;
    case HAL_SEN_DISCRETE: {
        // 获取电源状态
        int pst = sensor_get_psu_status(drv->psu, obj);
        ASSERT_FG(pst != -1, out, "Smbus read psu staus fail!");
        PSU_SET_STAT(drv->status, obj->id, (uint8_t)pst);
        hal_sensor_data(data, obj->id, obj->type, 0, 0, 0, hal_psu_stat(pst));
        return;
    }

    case HAL_SEN_WATTS:
        if (PSU_GET_STAT(drv->status, obj->id) == HAL_PSU_STAT_ON) {
            int ret = sensor_get_psu_watts(drv->psu, obj);
            ASSERT_FG(ret != -1, out, "Smbus read psu watts fail!");
            uint16_t d16 = (uint16_t)ret & 0xffff;
            value = psu_lineal_value(d16);
        }
        break;
    default:
        HAL_ERR("error object type");
    }

    hal_sensor_data(data, obj->id, obj->type, obj->min, obj->max, value, NULL);
out:
    return;
}

static int sensor_iter(hal_device_sensor_t *dev, hal_iter_sensor_t cb, void *priv)
{
    ASSERT_FR(dev && cb, -OS_EINVAL, "Invalid argument");
    hal_data_t *data = hal_data_alloc();
    sensor_drv_t *drv = dev->priv;

    int ret = 0;
    for (size_t i = 0; i < drv->obj_num; ++i) {
        sensor_info(drv, i, data);
        ret = cb(data, priv);
        ASSERT_FG(ret == 0, out, "");
    }

out:
    data->free(data);
    return ret;
}

static void sensor_close(struct hal_device_t *dev)
{
    sensor_drv_t *drv = dev->priv;

    if (drv->smb)
        drv->smb->free(drv->smb);

    if (drv->smb_fan)
        drv->smb_fan->free(drv->smb_fan);
    
    if (drv->psu) {
        if (drv->psu->smb)
            drv->psu->smb->free(drv->psu->smb);
        free(drv->psu);
    }
}

static sensor_drv_t sensor_drv = {
    .obj_num = 18,
    .objs    = {
        {.type = HAL_SEN_TEMP,  .id = HAL_SEN_TEMP_CPU0,    .factor = 1.0,   .slave = 0x40, .offset_l = 0x24,  .min = 0,  .max = 85 },
        {.type = HAL_SEN_TEMP,  .id = HAL_SEN_TEMP_SYS0,    .factor = 1.0,   .slave = 0x40, .offset_l = 0x22,  .min = 0,  .max = 60 },
        // 设备背面风扇
        {.type = HAL_SEN_FAN,   .id = HAL_SEN_FAN_SYS1,     .factor = 1.0,   .slave = 0x59, .offset_l = 0x24,  .offset_h = 0x25,   .min = 0,       .max = 4900 * 1.3 },
        {.type = HAL_SEN_FAN,   .id = HAL_SEN_FAN_SYS2,     .factor = 1.0,   .slave = 0x59, .offset_l = 0x26,  .offset_h = 0x27,   .min = 0,       .max = 4900 * 1.3 },
        // 交换芯片风扇
        {.type = HAL_SEN_FAN,   .id = HAL_SEN_FAN_SWITCH,   .factor = 1.0,   .slave = 0x40, .offset_l = 0x31,  .offset_h = 0x30,   .min = 0,       .max = 4800 * 1.3 },
        // E810 芯片风扇
        {.type = HAL_SEN_FAN,   .id = HAL_SEN_FAN_NETCARD,  .factor = 1.0,   .slave = 0x40, .offset_l = 0x33,  .offset_h = 0x32,   .min = 0,       .max = 5000 * 1.3 },
        {.type = HAL_SEN_FAN,   .id = HAL_SEN_FAN_CPU1,     .factor = 1.0,   .slave = 0x59, .offset_l = 0x28,  .offset_h = 0x29,   .min = 0,       .max = 6600 * 1.3 },
        {.type = HAL_SEN_VOL,   .id = HAL_SEN_VOL_CPU,      .factor = 0.001, .slave = 0x40, .offset_l = 0x11,  .offset_h = 0x10,   .min = 0.865,   .max = 0.955  },
        {.type = HAL_SEN_VOL,   .id = HAL_SEN_VOL_DDR,      .factor = 0.001, .slave = 0x40, .offset_l = 0x13,  .offset_h = 0x12,   .min = 1.14,     .max = 1.26 },
        {.type = HAL_SEN_VOL,   .id = HAL_SEN_VOL_3_3V,     .factor = 0.001, .slave = 0x40, .offset_l = 0x15,  .offset_h = 0x14,   .min = 3,       .max = 3.6  },
        {.type = HAL_SEN_VOL,   .id = HAL_SEN_VOL_5V,       .factor = 0.001, .slave = 0x40, .offset_l = 0x19,  .offset_h = 0x18,   .min = 4.5,     .max = 5.5 },
        {.type = HAL_SEN_VOL,   .id = HAL_SEN_VOL_12V,      .factor = 0.001, .slave = 0x40, .offset_l = 0x21,  .offset_h = 0x20,   .min = 10.8,    .max = 13.2 },

        {.type = HAL_SEN_DISCRETE, .id = HAL_SEN_PSU_STATUS1, .factor = 1.0,   .slave = 0x58, .offset_l = 0x79},
        {.type = HAL_SEN_DISCRETE, .id = HAL_SEN_PSU_STATUS2, .factor = 1.0,   .slave = 0x59, .offset_l = 0x79},
        {.type = HAL_SEN_WATTS,    .id = HAL_SEN_PSU_POUT1,   .factor = 1.0,   .slave = 0x58, .offset_l = 0x96, .min = 0, .max = 350},
        {.type = HAL_SEN_WATTS,    .id = HAL_SEN_PSU_POUT2,   .factor = 1.0,   .slave = 0x59, .offset_l = 0x96, .min = 0, .max = 350},
        {.type = HAL_SEN_WATTS,    .id = HAL_SEN_PSU_PIN1,    .factor = 1.0,   .slave = 0x58, .offset_l = 0x97, .min = 0, .max = 1440},
        {.type = HAL_SEN_WATTS,    .id = HAL_SEN_PSU_PIN2,    .factor = 1.0,   .slave = 0x59, .offset_l = 0x97, .min = 0, .max = 1440},
    },
};

static hal_sensor_method_t sensor_method = { .iter = sensor_iter };
static hal_device_sensor_t sensor_dev = {
    HACL_DEVICE(HAL_TAG_DEV_SENSOR, NULL, NULL, sensor_close),
    .method = &sensor_method,
};


static int sensor_psu_init(sensor_drv_t *drv)
{
    psu_object_t *psu = calloc(sizeof(psu_object_t), 1);
    ASSERT_FR(psu, -1, "malloc fail!");

    char devname[HAL_NAME_MAX] = { 0 };
    snprintf(devname, sizeof(devname), "/dev/i2c-%d", hal_find_i2c_bus(YUDI_PSU_BUS));
    const char *i2c_devname = hal_getenv(HAL_ENV_SMBUS_DEV) ?: devname;

    hal_smbus_t *smb = hal_smbus_alloc(i2c_devname, 0, 0);
    ASSERT_FG(smb, err, "smbus init fail!");

    psu->smb = smb;
    drv->psu = psu;

  
    // 将未获取到的电源型号设置为另一电源型号
    if (strncmp(drv->psu1_model, drv->psu2_model, strlen(drv->psu1_model)) != 0) {
        if (strncmp(drv->psu1_model, PSU_UNKNOWN_MODEL, strlen(drv->psu1_model)) == 0)
            strcpy(drv->psu1_model, drv->psu2_model);
        else if (strncmp(drv->psu2_model, PSU_UNKNOWN_MODEL, strlen(drv->psu2_model)) == 0)
            strcpy(drv->psu2_model, drv->psu1_model);
    }

    // 根据PSU型号修改功率范围信息
    for (unsigned int i = 0; i < sensor_drv.obj_num; i++) {
        switch (sensor_drv.objs[i].id)
        {
        case HAL_SEN_PSU_POUT1:
            ret = sensor_get_psu_out_max_watts(drv->psu1_model);
            ASSERT_FG(ret > 0, err, "get psu max watts fail!");
            drv->objs[i].max = ret;
            break;
        case HAL_SEN_PSU_POUT2:
            ret = sensor_get_psu_out_max_watts(drv->psu2_model);
            ASSERT_FG(ret > 0, err, "get psu max watts fail!");
            drv->objs[i].max = ret;
            break;
        case HAL_SEN_PSU_PIN1:
            ret = sensor_get_psu_in_max_watts(drv->psu1_model);
            ASSERT_FG(ret > 0, err, "get psu max watts fail!");
            drv->objs[i].max = ret;
            break;
        case HAL_SEN_PSU_PIN2:
            ret = sensor_get_psu_in_max_watts(drv->psu2_model);
            ASSERT_FG(ret > 0, err, "get psu max watts fail!");
            drv->objs[i].max = ret;
            break;
        default:
            break;
        }
    }

    return 0;
err:
    free(psu);
    return -1;
}

HAL_API hal_device_t *sensor_open(hal_module_t __attribute__((unused)) *hm, 
                                  hal_family_t __attribute__((unused)) *family)
{
    HAL_BUG_ON_OPEN(sensor_open);

    sensor_drv_t *drv = &sensor_drv;
    hal_device_sensor_t *dev = &sensor_dev;

    // 初始化获取sensor信息的smbus
    char devname[HAL_NAME_MAX] = { 0 };
    snprintf(devname, sizeof(devname), "/dev/i2c-%d", hal_find_i2c_bus(YUDI_SENSOR_BUS));

    const char *i2c_devname = hal_getenv(HAL_ENV_SMBUS_DEV) ?: devname;
    drv->smb = hal_smbus_alloc(i2c_devname, SENSOR_SLAVE, 0);
    ASSERT_FR(drv->smb, NULL, "sensor smbus init fail!");

    snprintf(devname, sizeof(devname), "/dev/i2c-%d", hal_find_i2c_bus(YUDI_BUS));
    drv->smb_fan = hal_smbus_alloc(i2c_devname, SENSOR_SLAVE, 0);
    ASSERT_FR(drv->smb_fan, NULL, "sensor smbus init fail!");

    // 初始化获取psu的smbus
    int ret = sensor_psu_init(drv);
    ASSERT_FG(ret == 0, err, "psu smbus init fail!");

    dev->priv = drv;
    return (hal_device_t *)dev;
err:
    drv->smb->free(drv->smb);
    drv->smb_fan->free(drv->smb_fan);
    return NULL;
}

HAL_DECL_MODULE(sensor) = {
    HAL_MODULE("sensor", HAL_TAG_MOD_DEVICE, sensor_open, HAL_FLG_MUTEX),
    .family_cnt = 1,
    .family = { { "sxf", "yudi", ""} },
};



