/*-----------------------------------------------------------------------
     创建者		: Morris chiou
     传感器		: 电子罗盘
     文件名		: SENSOR_AK09911C.c
     功能		: AK09911C 地磁传感器驱动
     创建日期	: 2017/11/27
---------------------------------------------------------------------- */

/* 注意：RETN 引脚必须连接到 Vcc，避免复位 */
#include "SENSOR_AK09911C.h"
#include "myiic.h"
#include <delay.h>
#include <math.h>
#include <stdio.h>

AK09911C5_ASA_DATA
AK09911C_ASA_XYZ_DATA; /* ASA X,Y,Z 轴灵敏度补偿数据，初始化时读取并存储 */

/*------------------------------- 获取第 1、2、3、4 象限的原始数据
 * -------------------------------*/
#define AK09911C_USE_RANDOM_COLLECT_DATA                                                                        \
  (0) /*如果为"1"->使用随机方式采集原始数据；如果为"0"->使用固定角度采集原始数据 \
       */

static uint8_t first_quadrant_cnt = 0, second_quadrant_cnt = 0,
               third_quadrant_cnt = 0, fourth_quadrant_cnt = 0;

#if (AK09911C_USE_RANDOM_COLLECT_DATA == 1)

#define AK09911C_data_size (50) /* 50 个 (x,y) 点 */
static int16_t first_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第一象限采集 50 个 (x,y) 点数据 */
static int16_t second_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第二象限采集 50 个 (x,y) 点数据 */
static int16_t third_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第三象限采集 50 个 (x,y) 点数据 */
static int16_t fourth_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第四象限采集 50 个 (x,y) 点数据 */

#elif (AK09911C_USE_RANDOM_COLLECT_DATA ==                                     \
       0) /* 使用固定角度采集，以获得更好的圆形轨迹 */

/* 单位：2.5 度 */
#define AK09911C_data_size (36) /* 36 个 (x,y) 点 */

#define AK0911C_min_get_point_num (15) /* 每个象限至少需要采集的数据点数量 */

static int16_t first_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第一象限采集 36 个 (x,y) 点数据 */
static int16_t second_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第二象限采集 36 个 (x,y) 点数据 */
static int16_t third_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第三象限采集 36 个 (x,y) 点数据 */
static int16_t fourth_quadrant_data[2][AK09911C_data_size] = {
    0}; /* 在第四象限采集 36 个 (x,y) 点数据 */

/* 90 度 / 2.5 度 = 36 等分，因此需要 5 个 uint8_t 变量来记录 */
static uint8_t first_fix_angle_flag[5] = {0};
static uint8_t second_fix_angle_flag[5] = {0};
static uint8_t third_fix_angle_flag[5] = {0};
static uint8_t fourth_fix_angle_flag[5] = {0};

#endif
/*------------------------------- 获取第 1、2、3、4 象限的原始数据
 * -------------------------------*/

/********************************************** 系统功能
 * **************************************************/
/*--------------------------------------------------------------------------------------------------*/
/**
 * @brief 读取 AK09911C 芯片 ID 和信息
 * @param id_info 存储读取到的 ID 信息数组（4 字节）
 * @return 0:成功; -1:读取失败
 * @note 读取寄存器 0x00~0x03
 */
int8_t AK09911C_GET_ID_INFO(uint8_t *id_info) {
  uint8_t read_data[4] = {0};
  int8_t status = 0;

  IIC_Stop_Hang();

  status = IIC_Read_N_Bytes(AK09911C_SLAVE_ADDRESS, AK09911C_REG_WAI1, 4,
                            &read_data[0]);
  if (status != 0) {
    return -1; /* 读取失败*/
  }

  id_info[0] = read_data[0]; /*AK09911C_REG_WAI1 ; 值 = 0x48*/
  id_info[1] = read_data[1]; /*AK09911C_REG_WAI2 ; 值 = 0x05*/
  id_info[2] = read_data[2]; /*AK09911C_REG_INFO1*/
  id_info[3] = read_data[3]; /*AK09911C_REG_INFO2*/

  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/* initial AK09911C */
/**
 * @brief 初始化 AK09911C 传感器
 *
 * @return 0: 初始化成功; -1: 初始化失败
 */
int8_t AK09911C_SET_INITIAL(void) {
  int8_t status = 0;

  IIC_Stop_Hang();

  /*步骤 1：复位 AK09911C*/
  status = AK09911C_SET_RESET();
  if (status != 0) {
    return -1; /* 复位失败*/
  }

  delay_ms(50);
  IIC_Stop_Hang();

  /*步骤 2：设置系统模式为掉电模式*/
  status = AK09911C_SET_SYSTEM_MODE(AK09911C_POWER_DOWN_MODE);
  if (status != 0) {
    return -1; /* 设置系统模式失败*/
  }
  delay_ms(2);

  IIC_Stop_Hang();

  /*步骤 3：获取 XYZ 轴灵敏度补偿值（熔丝 ROM）*/
  status = AK09911C_GET_ASA_DATA(&AK09911C_ASA_XYZ_DATA);
  if (status != 0) {
    return -1; /*获取 ASA 数据失败*/
  }

  return 0; /*初始化 AK09911C 成功*/
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*reset the AK09911C*/
int8_t AK09911C_SET_RESET(void) {
  int8_t status = 0;

  IIC_Stop_Hang();
  IIC_Stop_Hang();

  status = IIC_Write_1_Byte(AK09911C_SLAVE_ADDRESS, AK09911C_REG_CNTL3,
                            AK09911C_SW_RESET);
  if (status != 0) {
    return -1; /* 写入失败*/
  }

  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/**
 * @brief 设置掉电模式
 *
 * @return 成功返回 0，失败返回 -1
 * @note 掉电模式下，传感器停止测量并进入低功耗状态
 */
int8_t AK09911C_SET_POWER_DOWN_MODE(void) {

  int8_t status = 0;
  uint8_t write_data = AK09911C_POWER_DOWN_MODE & 0x1F; /*mask : 0x1F*/
  IIC_Stop_Hang();
  IIC_Stop_Hang();

  status =
      IIC_Write_1_Byte(AK09911C_SLAVE_ADDRESS, AK09911C_REG_CNTL2, write_data);
  if (status != 0) {
    return -1; /* 设置模式失败*/
  }
  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*  AK09911C */
/* mode : use enum AK09911C_MODE_SELECT*/
int8_t AK09911C_SET_SYSTEM_MODE(uint8_t mode) {
  int8_t status = 0;
  uint8_t write_data = mode & 0x1F; /*mask : 0x1F*/
  // printf(" SET_SYSTEM_MODE = 0x%x\r\n",write_data);

  IIC_Stop_Hang();
  IIC_Stop_Hang();
  IIC_Stop_Hang();

  status =
      IIC_Write_1_Byte(AK09911C_SLAVE_ADDRESS, AK09911C_REG_CNTL2, write_data);
  if (status != 0) {
    return -1; /* 设置模式失败*/
  }

  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*
        GET AK09911C mode
        AK09911C_POWER_DOWN_MODE=(0x0<<0),		// Power-down mode
        AK09911C_SINGLE_MODE=(0x01<<0),			// Single measurement
   mode AK09911C_CONTI_MODE_1=(0x01<<1),			// Continuous
   measurement mode 1 AK09911C_CONTI_MODE_2=(0x01<<2),			//
   Continuous measurement mode 2 AK09911C_CONTI_MODE_3=(0x03<<2),
   // Continuous measurement mode 3 AK09911C_CONTI_MODE_4=(0x01<<3),
   // Continuous measurement mode 4 AK09911C_SELF_TEST=(0x01<<4),
   // Self-test mode AK09911C_FUSE_ROM=(0x1F)
   //Fuse ROM access mode
*/
int8_t AK09911C_GET_MODE(uint8_t *mode) {
  uint8_t read_data = 0;
  int8_t status = 0;
  IIC_Stop_Hang();
  IIC_Stop_Hang();

  status = IIC_Read_1_Byte(AK09911C_SLAVE_ADDRESS, AK09911C_REG_CNTL2,
                           &read_data); /* 读取模式*/
  if (status != 0) {
    return -1; /* 读取失败*/
  }

  printf("AK09911C_GET_MODE = 0x%x\r\n", read_data);

  if (read_data == AK09911C_POWER_DOWN_MODE) /* 掉电模式*/
  {
    *mode = AK09911C_POWER_DOWN_MODE;
    return 0;
  } else if (read_data == AK09911C_SINGLE_MODE) /* 单次测量模式*/
  {
    *mode = AK09911C_SINGLE_MODE;
    return 0;
  } else if (read_data == AK09911C_CONTI_MODE_1) /* 连续测量模式 1*/
  {
    *mode = AK09911C_CONTI_MODE_1;
    return 0;
  } else if (read_data == AK09911C_CONTI_MODE_2) /* 连续测量模式 2*/
  {
    *mode = AK09911C_CONTI_MODE_2;
    return 0;
  } else if (read_data == AK09911C_CONTI_MODE_3) /* 连续测量模式 3*/
  {
    *mode = AK09911C_CONTI_MODE_3;
    return 0;
  } else if (read_data == AK09911C_CONTI_MODE_4) /* 连续测量模式 4*/
  {
    *mode = AK09911C_CONTI_MODE_4;
    return 0;
  } else if (read_data == AK09911C_SELF_TEST) /* 自测试模式*/
  {
    *mode = AK09911C_SELF_TEST;
    return 0;
  } else if (read_data == AK09911C_FUSE_ROM) /* 熔丝 ROM 访问模式*/
  {
    *mode = AK09911C_FUSE_ROM;
    return 0;
  } else {
    /* 无操作*/
  }

  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/* AK09911C */
/* read the Data status  [reg 0x10] */
int8_t AK09911C_GET_DATA_READY_STATUS(uint8_t *dataready_status) {

  int8_t status = 0;
  uint8_t read_data = 0;
  IIC_Stop_Hang();

  status =
      IIC_Read_1_Byte(AK09911C_SLAVE_ADDRESS, AK09911C_REG_ST1, &read_data);
  if (status != 0) {
    return -1; /* 读取状态 1 失败*/
  }

  *dataready_status = read_data;
  return 0; /* 读取状态 1 成功*/
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/* get AK09911C the overflow status [bit3]*/
/* *** when any of measurement data is read, be sure to read ST2 register at the
 * end.*/
int8_t AK09911C_GET_OVER_FLOW_STATUS(uint8_t *overflow_status) {

  int8_t status = 0;
  uint8_t read_data = 0;
  IIC_Stop_Hang();
  IIC_Stop_Hang();

  status =
      IIC_Read_1_Byte(AK09911C_SLAVE_ADDRESS, AK09911C_REG_ST2, &read_data);
  if (status != 0) {
    return -1; /* 读取状态 2 失败*/
  }

  *overflow_status = read_data;
  return 0; /* 读取状态 2 成功*/
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*get  AK09911C  XYZ axis Magnetic Data*/
int8_t AK09911C_GET_XYZ_DATA(AK09911C_AXIS_DATA *raw_data) {

  uint8_t read_data[6] = {0};
  int8_t status = 0;
  uint8_t data_ready = 0, over_flow = 0;

  IIC_Stop_Hang();

  status = AK09911C_SET_SYSTEM_MODE(AK09911C_SINGLE_MODE);
  if (status != 0) {
    return -1; /* 设置系统模式失败*/
  }

  IIC_Stop_Hang();

  delay_ms(50);

  status = AK09911C_GET_DATA_READY_STATUS(&data_ready);
  if (status != 0) {
    return -1; /* 读取失败*/
  }

  IIC_Stop_Hang();
  // printf("data_ready = 0x%x\r\n",data_ready);

  /* 检查状态：无跳过数据 [bit1]=0 且 数据就绪 [bit0]=1 */
  if ((data_ready & 0x03) != 0x03)
    return -3; /* 数据未就绪或跳过数据*/

  IIC_Stop_Hang();

  status = IIC_Read_N_Bytes(AK09911C_SLAVE_ADDRESS, AK09911C_REG_HXL, 6,
                            &read_data[0]);
  if (status != 0) {
    return -1; /* 读取失败*/
  }

  raw_data->X_AXIS = (uint16_t)(read_data[1] << 8) +
                     (uint16_t)read_data[0]; /* 测量磁数据 - X 轴数据 */
  raw_data->Y_AXIS = (uint16_t)(read_data[3] << 8) +
                     (uint16_t)read_data[2]; /* 测量磁数据 - Y 轴数据 */
  raw_data->Z_AXIS = (uint16_t)(read_data[5] << 8) +
                     (uint16_t)read_data[4]; /* 测量磁数据 - Z 轴数据 */

  IIC_Stop_Hang();

  /** 读取任何测量数据后，必须最后读取 ST2 寄存器*/
  status = AK09911C_GET_OVER_FLOW_STATUS(&over_flow);
  if (status != 0) {
    return -1; /* 读取失败*/
  }

  IIC_Stop_Hang();

  /*检查溢出 */
  if (over_flow & 0x08) {
    return -4; /*数据溢出!! */
  }

  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/* get the AK09911C  XYZ-axis sensitivity adjustment value*/
/**
 * @brief 获取 AK09911C XYZ 轴灵敏度补偿值（熔丝 ROM）
 * @param asa_data 存储 ASA 数据的结构体指针
 * @return 0:成功; -1:失败
 */
int8_t AK09911C_GET_ASA_DATA(AK09911C5_ASA_DATA *asa_data) {
  uint8_t read_data[3] = {0};
  int8_t status = 0;

  IIC_Stop_Hang();

  status = AK09911C_SET_SYSTEM_MODE(
      AK09911C_FUSE_ROM); /*设置 FUSE_ROM 模式以读取 ASA 数据*/
  if (status != 0) {
    return -1; /*设置系统模式失败*/
  }

  status = IIC_Read_N_Bytes(AK09911C_SLAVE_ADDRESS, AK09911C_REG_ASAX, 3,
                            &read_data[0]);
  if (status != 0) {
    return -1; /*读取失败*/
  }

  asa_data->ASA_X = read_data[0]; /*X 轴灵敏度补偿值*/
  asa_data->ASA_Y = read_data[1]; /*Y 轴灵敏度补偿值*/
  asa_data->ASA_Z = read_data[2]; /*Z 轴灵敏度补偿值*/

  printf("AK09911C ASA_X = 0x%x\r\n", AK09911C_ASA_XYZ_DATA.ASA_X);
  printf("AK09911C ASA_Y = 0x%x\r\n", AK09911C_ASA_XYZ_DATA.ASA_Y);
  printf("AK09911C ASA_Z = 0x%x\r\n", AK09911C_ASA_XYZ_DATA.ASA_Z);

  status = AK09911C_SET_SYSTEM_MODE(
      AK09911C_POWER_DOWN_MODE); /*读取 ROM 数据后进入掉电模式*/
  if (status != 0) {
    return -1; /*设置系统模式失败*/
  }

  return 0;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*
        calculate the XYZ data of AK09911C  XYZ axis & XYZ ASA data
        Sensitivity adjustment data for each axis is stored to fuse ROM on
   shipment.
        ** See AK09911C Spec page.30

        Hadj -> adjusted measurement data.
        H 	 -> the measurement data read out from the measurement data
   register. ASA  -> REG_ASAX,Y,Z data.

        Hadj = H *(      (ASA + 128 ) / 128 )

        H is in the range of -8190 to 8190.  The magnetometer has a range of
        +4912uT & -4912uT .  To go from the raw value to uT is:

        HuT = H * 4912/8190, or  H*6/10.

        Since 1uT = 100 gauss, our final scale factor becomes:

        Hadj = H * ((ASA + 128) / 128) * 6/10 * 100

        Hadj = H * ((ASA + 128) * 60 / 128
*/
/**
 * @brief 计算 AK09911C XYZ 轴的校准数据（包含 ASA 补偿）
 * @param data 待校准的原始数据指针（输入输出参数）
 * @note 灵敏度补偿数据存储在各轴的熔丝 ROM 中
 *       参考 AK09911C 规格书第 30 页
 *
 *       计算公式：Hadj = H * ((ASA + 128) / 128)
 *       Hadj -> 校准后的测量数据
 *       H     -> 从测量数据寄存器读取的原始数据
 *       ASA   -> REG_ASAX/Y/Z 寄存器的值
 *
 *       H 的范围：-8190 到 8190
 *       磁力计量程：+4912uT ~ -4912uT
 *       原始值转 uT: HuT = H * 4912/8190 = H * 6/10
 *
 *       最终比例因子：Hadj = H * ((ASA + 128) * 60 / 128)
 */
void AK09911C_GET_CAL(AK09911C_AXIS_DATA *data) {
  int16_t temp = 0;
  float temp2 = 0;

  data->X_AXIS = (data->X_AXIS * 6 / 10);
  data->Y_AXIS = (data->Y_AXIS * 6 / 10);
  data->Z_AXIS = (data->Z_AXIS * 6 / 10);

  /*X 轴*/
  temp2 =
      (float)data->X_AXIS * (((float)AK09911C_ASA_XYZ_DATA.ASA_X / 128) + 1);
  data->X_AXIS = (int16_t)temp2;

  /*Y 轴*/
  temp2 =
      (float)data->Y_AXIS * (((float)AK09911C_ASA_XYZ_DATA.ASA_Y / 128) + 1);
  data->Y_AXIS = (int16_t)temp2;

  /*Z 轴*/
  temp2 =
      (float)data->Z_AXIS * (((float)AK09911C_ASA_XYZ_DATA.ASA_Z / 128) + 1);
  data->Z_AXIS = (int16_t)temp2;
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*
        step 1. Set Power-down mode. (MODE[4:0]=��00000��)
        step 2. Set Self-test mode. (MODE[4:0]=��10000��)
        step 3. Check Data Ready or not by polling DRDY bit of ST1 register ;
   When Data Ready, proceed to the next step. step 4. Read measurement data (HXL
   to HZH)
*/
/**
 * @brief 执行 AK09911C 自测试
 * @param data 存储自测试数据的结构体指针
 * @return 0:成功; -1:写入失败; -3:数据溢出
 * @note 自测试步骤：
 *   1. 设置掉电模式 (MODE[4:0]="00000")
 *   2. 设置自测试模式 (MODE[4:0]="10000")
 *   3. 轮询 ST1 寄存器的 DRDY 位检查数据就绪
 *   4. 读取测量数据 (HXL 到 HZH)
 */
int8_t AK09911C_SET_SELF_TEST(AK09911C_AXIS_DATA *data) {
  int8_t status = 0, status1 = 0;
  uint8_t count = 0, read_data_ready_status = 0, read_data_overflow_status = 0;
  uint8_t raw_data[6] = {0};

  IIC_Stop_Hang();
  IIC_Stop_Hang();

  /*步骤 1：设置掉电模式 (MODE[4:0]="00000")*/
  status = AK09911C_SET_POWER_DOWN_MODE();
  if (status != 0) {
    return -1; /* 写入失败*/
  }
  delay_ms(1);

  IIC_Stop_Hang();

  /*步骤 2：设置自测试模式 (MODE[4:0]="10000")*/
  status = AK09911C_SET_SYSTEM_MODE(AK09911C_SELF_TEST);
  if (status != 0) {
    return -1; /* 写入失败*/
  }

  /*步骤 3：轮询 ST1 寄存器的 DRDY 位检查数据就绪*/
  for (count = 0; count < 8; count++) /*重试 8 次 */
  {
    AK09011C_DELAY(0xFF0);

    status = AK09911C_GET_DATA_READY_STATUS(
        &read_data_ready_status); /*检查 DRDY, DOR 位*/

    if (status == 0) {
      /*数据就绪且无错误*/
      if ((read_data_ready_status & 0x03) == 0x03)
        break; /* 无跳过数据 [bit1]=0 且 数据就绪 [bit0]=1*/

    } else if (status == -1) {
      /*读取状态失败*/
      status1 = -1;
    }
  }

  /*步骤 4：读取测量数据 (HXL 到 HZH)*/

  IIC_Stop_Hang();
  IIC_Stop_Hang();
  /*读取 xyz 轴原始数据*/
  status = IIC_Read_N_Bytes(AK09911C_SLAVE_ADDRESS, AK09911C_REG_HXL, 6,
                            &raw_data[0]);
  if (status != 0) {
    return -1; /*读取失败*/
  }

  IIC_Stop_Hang();

  /*读取任何测量数据后，必须读取 ST2 寄存器*/
  status = AK09911C_GET_OVER_FLOW_STATUS(&read_data_overflow_status);
  if (status != 0) {
    return -1; /* 读取失败*/
  }

  /*检查溢出*/
  if (read_data_overflow_status & 0x08) {
    return -3; /* 数据溢出!!*/
  }

  /*** 设置为掉电模式*/
  IIC_Stop_Hang();
  IIC_Stop_Hang();

  status = AK09911C_SET_POWER_DOWN_MODE();
  if (status != 0) {
    return -1; /* 写入失败*/
  }

  data->X_AXIS = (raw_data[1] << 8) + raw_data[0];
  data->Y_AXIS = (raw_data[3] << 8) + raw_data[2];
  data->Z_AXIS = (raw_data[5] << 8) + raw_data[4];

  AK09911C_GET_CAL(data);

  return 0; /* 读取成功*/
}
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/**
 * @brief AK09911C 软件延时函数
 * @param count 延时计数值
 */
void AK09011C_DELAY(uint32_t count) {
  uint32_t delay_cnt = 0;
  uint8_t delay = 0;

  for (delay_cnt = 0; delay_cnt <= count; delay_cnt++) {
    delay++;
    delay--;
  }
}
/*--------------------------------------------------------------------------------------------------*/

/********************************************** 系统功能
 * **************************************************/

/********************************************** 高级应用功能
 * **************************************************/
/*------------------------------------------------------------------------------------------------------*/
/**
 * @brief 采集校准数据点（需要旋转电子罗盘 360 度或 720 度）
 * @param data 当前磁力计数据
 * @param quadrant_ready 象限数据就绪标志指针
 * @return 1:所有象限数据采集完成; -1:数据未就绪
 *
 * @note 坐标系说明：
 *                    (+y)
 *                      |
 *                      |
 *        第二象限      |     第一象限
 *                      |
 *                      |
 *       (-x) ----------+---------- (+x)
 *                      |
 *                      |
 *        第三象限      |     第四象限
 *                      |
 *                      |
 *                     (-y)
 *
 * 最终结果保存在：
 *   first_quadrant_data[2][AK09911C_data_size]
 *   second_quadrant_data[2][AK09911C_data_size]
 *   third_quadrant_data[2][AK09911C_data_size]
 *   fourth_quadrant_data[2][AK09911C_data_size]
 */
uint8_t
AK09011C_GET_CALIBRATE_DATA(AK09911C_AXIS_DATA data,
                            AK09911C_4_QUADRANT_DATA_POINT *quadrant_ready) {
  uint16_t cnt = 0, check_level = 0, check_array = 0, check_bit = 0;
  float check_raw_data_theta = 0;
  float down_data = 0.0, up_data = 0.0;
  uint8_t tiny_cnt = 0;

  // https://blog.csdn.net/mu399/article/details/81951786
  // https://zh.wikipedia.org/wiki/%E4%B8%89%E8%A7%92%E5%87%BD%E6%95%B0

  /* 计算角度 theta */
  check_raw_data_theta = atan2(data.Y_AXIS, data.X_AXIS) * 180 / 3.14159;
  if (check_raw_data_theta > 360) {
    check_raw_data_theta -= 360;
  }

  if (check_raw_data_theta < 0) {
    check_raw_data_theta += 360;
  }

  printf("theta = %f C\r\n", check_raw_data_theta);

#if (AK09911C_USE_RANDOM_COLLECT_DATA == 1) /* 使用随机方式采集数据点 */

  if (check_raw_data_theta >= 0 && check_raw_data_theta < 90) /*第一象限数据*/
  {
    if (first_quadrant_cnt < AK09911C_data_size) {
      first_quadrant_data[0][first_quadrant_cnt] = data.X_AXIS;
      first_quadrant_data[1][first_quadrant_cnt] = data.Y_AXIS;

      quadrant_ready->first_quadant_data_pointok = 0;
      first_quadrant_cnt++;
    } else {
      quadrant_ready->first_quadant_data_pointok = 1;
    }
  } else if (check_raw_data_theta >= 90 &&
             check_raw_data_theta < 180) /*第二象限数据*/
  {
    if (second_quadrant_cnt < AK09911C_data_size) {
      second_quadrant_data[0][second_quadrant_cnt] = data.X_AXIS;
      second_quadrant_data[1][second_quadrant_cnt] = data.Y_AXIS;

      quadrant_ready->second_quadant_data_pointok = 0;
      second_quadrant_cnt++;
    } else {
      quadrant_ready->second_quadant_data_pointok = 1;
    }
  } else if (check_raw_data_theta >= 180 &&
             check_raw_data_theta < 270) /*第三象限数据*/
  {
    if (third_quadrant_cnt < AK09911C_data_size) {
      third_quadrant_data[0][third_quadrant_cnt] = data.X_AXIS;
      third_quadrant_data[1][third_quadrant_cnt] = data.Y_AXIS;

      quadrant_ready->third_quadant_data_pointok = 0;
      third_quadrant_cnt++;
    } else {
      quadrant_ready->third_quadant_data_pointok = 1;
    }
  } else if (check_raw_data_theta >= 270 &&
             check_raw_data_theta < 360) /*第四象限数据*/
  {
    if (fourth_quadrant_cnt < AK09911C_data_size) {
      fourth_quadrant_data[0][fourth_quadrant_cnt] = data.X_AXIS;
      fourth_quadrant_data[1][fourth_quadrant_cnt] = data.Y_AXIS;

      quadrant_ready->fourth_quadant_data_pointok = 0;
      fourth_quadrant_cnt++;
    } else {
      quadrant_ready->fourth_quadant_data_pointok = 1;
    }
  }

  printf("1_cnt:%d ; 2_cnt:%d ,3_cnt:%d ,4_cnt:%d\r\n", first_quadrant_cnt,
         second_quadrant_cnt, third_quadrant_cnt, fourth_quadrant_cnt);

#else

  if (check_raw_data_theta >= 0 && check_raw_data_theta < 90) /*第一象限数据*/
  {

    printf("1-quadrant %d\r\n", first_quadrant_cnt);
    if (first_quadrant_cnt < AK09911C_data_size) {
      /* 90 / 2.5 = 36 -> 36 个点 */
      for (cnt = 0; cnt < AK09911C_data_size; cnt++) /* 0~35，共 36 个 */
      {
        /* 计算下限和上限 */
        /* 例如 cnt = 0 ; down_data = 0 , up_data = 2.5 */
        /* 例如 cnt = 1 ; down_data = 2.5 , up_data = 5 */
        down_data = (float)cnt * 2.5; /* 2.5 -> 2.5 度 */
        up_data = (float)(cnt + 1) * 2.5;
        // printf("%f ~ %f \r\n",down_data,up_data);
        if (check_raw_data_theta >= down_data &&
            check_raw_data_theta < up_data) {
          check_level = (uint16_t)(down_data / 2.5);
          /* 获取需要置 1 的位 */ /* 例如：down_data =85 ; 85/2.5= 34 位 */
          check_array = check_level / 8; /* 获取数组索引 */
          check_bit = check_level % 8;   /* 获取数组中的位*/

          printf("check_array %d , check_bit %d\r\n", check_array, check_bit);
          // printf("pre check 0x%x\r\n",first_fix_angle_flag[check_array] &
          // (1<<check_bit) );
          /* 检查角度标志 !=0 */ /* 如果 check == 1，表示该角度的原始数据 (X,Y)
                                    已存在 */
          if ((first_fix_angle_flag[check_array] & (1 << check_bit)) ==
              0) /* 该角度没有原始数据记录，需要保存 */
          {
            /* 设置角度标志为"1" */
            first_fix_angle_flag[check_array] =
                first_fix_angle_flag[check_array] | (1 << check_bit);

            for (tiny_cnt = 0; tiny_cnt < 5; tiny_cnt++) {
              printf("first_fix_angle_flag[%d] 0x%x\r\n", tiny_cnt,
                     first_fix_angle_flag[tiny_cnt]);
            }

            /*保存 X,Y 点数据 */
            first_quadrant_data[0][first_quadrant_cnt] = data.X_AXIS;
            first_quadrant_data[1][first_quadrant_cnt] = data.Y_AXIS;

            quadrant_ready->first_quadant_data_pointok = 0;
            first_quadrant_cnt++;

            break; /* 退出 for 循环 */

          } else /* 该角度已有原始数据 */
          {
            /* 不做任何操作，检查下一个点 */
            printf("do nothing\r\n");
          }
        }
      }
    } else {
      quadrant_ready->first_quadant_data_pointok = 1;
    }
  } else if (check_raw_data_theta >= 90 &&
             check_raw_data_theta < 180) /*第二象限数据*/
  {
    printf("2-quadrant %d\r\n", second_quadrant_cnt);
    if (second_quadrant_cnt < AK09911C_data_size) {

      /* 90 / 2.5 = 36 -> 36 个点 */
      for (cnt = 0; cnt < AK09911C_data_size; cnt++) /* 0~35，共 36 个 */
      {
        /* 计算下限和上限 */
        /* 例如 cnt = 0 ; down_data = 0 , up_data = 2.5 */
        /* 例如 cnt = 1 ; down_data = 2.5 , up_data = 5 */
        down_data = (float)cnt * 2.5 + 90.0;
        /* 2.5 -> 2.5 度 */ /*起始 -> 90 度 */
        up_data = (float)(cnt + 1) * 2.5 + 90.0;

        if (check_raw_data_theta >= down_data &&
            check_raw_data_theta < up_data) {
          check_level = (uint16_t)(down_data / 2.5);
          /* 获取需要置 1 的位 */ /* 例如：down_data =85 ; 85/2.5= 34 位 */
          check_array = check_level / 8; /* 获取数组索引 */
          check_bit = check_level % 8;

          printf("check_array %d , check_bit %d\r\n", check_array, check_bit);

          /* 检查角度标志 !=0 */ /* 如果 check == 1，表示该角度的原始数据 (X,Y)
                                    已存在 */
          if ((second_fix_angle_flag[check_array] & (1 << check_bit)) ==
              0) /* 该角度没有原始数据记录，需要保存 */
          {
            /* 设置角度标志为"1" */
            second_fix_angle_flag[check_array] =
                second_fix_angle_flag[check_array] | (1 << check_bit);

            for (tiny_cnt = 0; tiny_cnt < 5; tiny_cnt++) {
              printf("second_fix_angle_flag[%d] 0x%x\r\n", tiny_cnt,
                     second_fix_angle_flag[tiny_cnt]);
            }

            /*保存 X,Y 点数据 */
            second_quadrant_data[0][second_quadrant_cnt] = data.X_AXIS;
            second_quadrant_data[1][second_quadrant_cnt] = data.Y_AXIS;

            quadrant_ready->second_quadant_data_pointok = 0;
            second_quadrant_cnt++;

            break; /* 退出 for 循环 */

          } else /* 该角度已有原始数据 */
          {
            /* 不做任何操作，检查下一个点 */
          }
        }
      }
    } else {
      quadrant_ready->second_quadant_data_pointok = 1;
    }
  } else if (check_raw_data_theta >= 180 &&
             check_raw_data_theta < 270) /*第三象限数据*/
  {
    printf("3-quadrant %d\r\n", third_quadrant_cnt);
    if (third_quadrant_cnt < AK09911C_data_size) {
      /* 90 / 2.5 = 36 -> 36 个点 */
      for (cnt = 0; cnt < AK09911C_data_size; cnt++) /* 0~35，共 36 个 */
      {
        /* 计算下限和上限 */
        /* 例如 cnt = 0 ; down_data = 0 , up_data = 2.5 */
        /* 例如 cnt = 1 ; down_data = 2.5 , up_data = 5 */
        down_data = (float)cnt * 2.5 + 180.0;
        /* 2.5 -> 2.5 度 */ /*起始 -> 180 度 */
        up_data = (float)(cnt + 1) * 2.5 + 180.0;

        if (check_raw_data_theta >= down_data &&
            check_raw_data_theta < up_data) {
          check_level = (uint16_t)(down_data / 2.5);
          /* 获取需要置 1 的位 */ /* 例如：down_data =85 ; 85/2.5= 34 位 */
          check_array = check_level / 8; /* 获取数组索引 */
          check_bit = check_level % 8;

          printf("check_array %d , check_bit %d\r\n", check_array, check_bit);

          /* 检查角度标志 !=0 */ /* 如果 check == 1，表示该角度的原始数据 (X,Y)
                                    已存在 */
          if ((third_fix_angle_flag[check_array] & (1 << check_bit)) ==
              0) /* 该角度没有原始数据记录，需要保存 */
          {
            /* 设置角度标志为"1" */
            third_fix_angle_flag[check_array] =
                third_fix_angle_flag[check_array] | (1 << check_bit);

            for (tiny_cnt = 0; tiny_cnt < 5; tiny_cnt++) {
              printf("third_fix_angle_flag[%d] 0x%x\r\n", tiny_cnt,
                     third_fix_angle_flag[tiny_cnt]);
            }

            /*保存 X,Y 点数据 */
            third_quadrant_data[0][third_quadrant_cnt] = data.X_AXIS;
            third_quadrant_data[1][third_quadrant_cnt] = data.Y_AXIS;

            quadrant_ready->third_quadant_data_pointok = 0;
            third_quadrant_cnt++;

            break; /* 退出 for 循环 */
          } else   /* 该角度已有原始数据 */
          {
            /* 不做任何操作，检查下一个点 */
          }
        }
      }
    } else {
      quadrant_ready->third_quadant_data_pointok = 1;
    }
  } else if (check_raw_data_theta >= 270 &&
             check_raw_data_theta < 360) /*第四象限数据*/
  {
    printf("4-quadrant %d\r\n", fourth_quadrant_cnt);
    if (fourth_quadrant_cnt < AK09911C_data_size) {
      /* 90 / 2.5 = 36 -> 36 个点 */
      for (cnt = 0; cnt < AK09911C_data_size; cnt++) /* 0~35，共 36 个 */
      {
        /* 计算下限和上限 */
        /* 例如 cnt = 0 ; down_data = 0 , up_data = 2.5 */
        /* 例如 cnt = 1 ; down_data = 2.5 , up_data = 5 */
        down_data = (float)cnt * 2.5 + 270.0;
        /* 2.5 -> 2.5 度 */ /*起始 -> 270 度 */
        up_data = (float)(cnt + 1) * 2.5 + 270.0;

        if (check_raw_data_theta >= down_data &&
            check_raw_data_theta < up_data) {
          check_level = (uint16_t)(down_data / 2.5);
          /* 获取需要置 1 的位 */ /* 例如：down_data =85 ; 85/2.5= 34 位 */
          check_array = check_level / 8; /* 获取数组索引 */
          check_bit = check_level % 8;

          printf("check_array %d , check_bit %d\r\n", check_array, check_bit);

          /* 检查角度标志 !=0 */ /* 如果 check == 1，表示该角度的原始数据 (X,Y)
                                    已存在 */
          if ((fourth_fix_angle_flag[check_array] & (1 << check_bit)) ==
              0) /* 该角度没有原始数据记录，需要保存 */
          {
            /* 设置角度标志为"1" */
            fourth_fix_angle_flag[check_array] =
                fourth_fix_angle_flag[check_array] | (1 << check_bit);

            for (tiny_cnt = 0; tiny_cnt < 5; tiny_cnt++) {
              printf("fourth_fix_angle_flag[%d] 0x%x\r\n", tiny_cnt,
                     fourth_fix_angle_flag[tiny_cnt]);
            }

            /*保存 X,Y 点数据 */
            fourth_quadrant_data[0][fourth_quadrant_cnt] = data.X_AXIS;
            fourth_quadrant_data[1][fourth_quadrant_cnt] = data.Y_AXIS;

            quadrant_ready->fourth_quadant_data_pointok = 0;
            fourth_quadrant_cnt++;

            break; /* 退出 for 循环 */
          } else   /* 该角度已有原始数据 */
          {
            /* 不做任何操作，检查下一个点 */
          }
        }
      }
    } else {
      quadrant_ready->fourth_quadant_data_pointok = 1;
    }
  }
#endif

  if (first_quadrant_cnt >= AK0911C_min_get_point_num &&
      second_quadrant_cnt >= AK0911C_min_get_point_num &&
      third_quadrant_cnt >= AK0911C_min_get_point_num &&
      fourth_quadrant_cnt >= AK0911C_min_get_point_num)
  //	if(first_quadrant_cnt==AK09911C_data_size &&
  // second_quadrant_cnt==AK09911C_data_size &&
  // third_quadrant_cnt==AK09911C_data_size &&
  // fourth_quadrant_cnt==AK09911C_data_size)
  {
/* 通过 UART 显示数据 */
#if 1
    printf("===================================\r\n");
    for (cnt = 0; cnt < first_quadrant_cnt; cnt++) {
      printf("%d %d\r\n", first_quadrant_data[0][cnt],
             first_quadrant_data[1][cnt]);
    }
    for (cnt = 0; cnt < second_quadrant_cnt; cnt++) {
      printf("%d %d\r\n", second_quadrant_data[0][cnt],
             second_quadrant_data[1][cnt]);
    }
    for (cnt = 0; cnt < third_quadrant_cnt; cnt++) {
      printf("%d %d\r\n", third_quadrant_data[0][cnt],
             third_quadrant_data[1][cnt]);
    }
    for (cnt = 0; cnt < fourth_quadrant_cnt; cnt++) {
      printf("%d %d\r\n", fourth_quadrant_data[0][cnt],
             fourth_quadrant_data[1][cnt]);
    }

    printf("===================================\r\n");
#endif

    return 1; /*最终数据采集完成! */
  }

  return -1; /*原始数据未就绪 */
}
/*------------------------------------------------------------------------------------------------------*/
/**
 * @brief 获取电子罗盘校准因子，消除硬铁和软铁干扰
 * @param cal_result 存储校准结果的结构体指针
 * @return 1:找到校准因子
 *
 * @note 使用采集到的四个象限数据计算校准因子：
 *   first_quadrant_data[2][AK09911C_data_size] ->
 *     first_quadrant_data[0][] 作为 X 轴数据
 *     first_quadrant_data[1][] 作为 Y 轴数据
 *   second_quadrant_data[2][AK09911C_data_size]
 *   third_quadrant_data[2][AK09911C_data_size]
 *   fourth_quadrant_data[2][AK09911C_data_size]
 *
 * 计算 Xmax, Xmin, Ymax, Ymin，找出 Xoffset, Yoffset：
 *   xoffset = (xmax + xmin) / 2;
 *   yoffset = (ymax + ymin) / 2;
 *   xgain = 1;
 *   ygain = (final_x_max - final_x_min) / (final_y_max - final_y_min);
 */
uint8_t AK09911C_GET_CALIBRATE_FACTOR(AK09911C_CALIBRATE *cal_result) {
  int16_t x_max[4] = {0}, x_min[4] = {0}, y_max[4] = {0},
          y_min[4] = {0}; /*例如 x_max[0] -> 第一象限 X 最大值；x_max[1] ->
                             第二象限 X 最大值，以此类推 */
  int16_t final_x_max = 0, final_x_min = 0, final_y_max = 0, final_y_min = 0;
  uint8_t cnt = 0;

  float x_offset = 0.0, y_offset = 0.0;
  float x_gain = 0.0, y_gain = 0.0;

  /* 查找各象限的 x_max 和 y_min */
  for (cnt = 0; cnt < AK09911C_data_size; cnt++) {
    /* 查找 X 轴最大值部分 */
    if (first_quadrant_data[0][cnt] > x_max[0]) {
      x_max[0] = first_quadrant_data[0][cnt];
    }

    if (second_quadrant_data[0][cnt] > x_max[1]) {
      x_max[1] = second_quadrant_data[0][cnt];
    }

    if (third_quadrant_data[0][cnt] > x_max[2]) {
      x_max[2] = third_quadrant_data[0][cnt];
    }

    if (fourth_quadrant_data[0][cnt] > x_max[3]) {
      x_max[3] = fourth_quadrant_data[0][cnt];
    }

    /* 查找 X 轴最小值部分 */
    if (first_quadrant_data[0][cnt] <= x_min[0]) {
      x_min[0] = first_quadrant_data[0][cnt];
    }

    if (second_quadrant_data[0][cnt] <= x_min[1]) {
      x_min[1] = second_quadrant_data[0][cnt];
    }

    if (third_quadrant_data[0][cnt] <= x_min[2]) {
      x_min[2] = third_quadrant_data[0][cnt];
    }

    if (fourth_quadrant_data[0][cnt] <= x_min[3]) {
      x_min[3] = fourth_quadrant_data[0][cnt];
    }

    /* 查找 Y 轴最大值部分 */
    if (first_quadrant_data[1][cnt] > y_max[0]) {
      y_max[0] = first_quadrant_data[1][cnt];
    }

    if (second_quadrant_data[1][cnt] > y_max[1]) {
      y_max[1] = second_quadrant_data[1][cnt];
    }

    if (third_quadrant_data[1][cnt] > y_max[2]) {
      y_max[2] = third_quadrant_data[1][cnt];
    }

    if (fourth_quadrant_data[1][cnt] > y_max[3]) {
      y_max[3] = fourth_quadrant_data[1][cnt];
    }

    /* 查找 Y 轴最小值部分 */
    if (first_quadrant_data[1][cnt] <= y_min[0]) {
      y_min[0] = first_quadrant_data[1][cnt];
    }

    if (second_quadrant_data[1][cnt] <= y_min[1]) {
      y_min[1] = second_quadrant_data[1][cnt];
    }

    if (third_quadrant_data[1][cnt] <= y_min[2]) {
      y_min[2] = third_quadrant_data[1][cnt];
    }

    if (fourth_quadrant_data[1][cnt] <= y_min[3]) {
      y_min[3] = fourth_quadrant_data[1][cnt];
    }
  }

  /* 查找最终的 x_max 和 y_min */
  for (cnt = 0; cnt < 4; cnt++) {
    /*最终查找 X 轴最大最小值部分 */
    if (x_max[cnt] > final_x_max) {
      final_x_max = x_max[cnt];
    }

    if (x_min[cnt] < final_x_min) {
      final_x_min = x_min[cnt];
    }

    /*最终查找 Y 轴最大最小值部分 */
    if (y_max[cnt] > final_y_max) {
      final_y_max = y_max[cnt];
    }

    if (y_min[cnt] < final_y_min) {
      final_y_min = y_min[cnt];
    }
  }

  /* 计算 X 轴和 Y 轴的偏移量 */
  x_offset = ((float)final_x_max + (float)final_x_min) / 2;
  y_offset = ((float)final_y_max + (float)final_y_min) / 2;

  /* 设置 x_gain 为 1，y_gain = (final_x_max - final_x_min) / (final_y_max -
   * final_y_min) */
  x_gain = 1.0;
  y_gain = ((float)final_x_max - (float)final_x_min) /
           ((float)final_y_max - (float)final_y_min);

  /* 获取最终的校准因子 */
  cal_result->X_OFFSET = x_offset;
  cal_result->Y_OFFSET = y_offset;
  cal_result->X_FACTOR_BASE = x_gain;
  cal_result->Y_FACTOR_BASE = y_gain;

  printf("final  X_offset %f ; Y_offset %f\r\n", cal_result->X_OFFSET,
         cal_result->Y_OFFSET);
  printf("final  X_factor %f ; Y_factor %f\r\n", cal_result->X_FACTOR_BASE,
         cal_result->Y_FACTOR_BASE);

  return 1; /* 找到校准因子 */
}
/*------------------------------------------------------------------------------------------------------*/
/**
 * @brief 简单计算 AK09911C 方位角（无补偿）
 * @param data 磁力计数据
 * @return 方位角（角度制）
 *
 * @note 坐标系说明：
 *                        y
 *                        ^
 *                        |     /
 *                        |    /
 *                        |   /
 *                        |  /
 *                        | /     degree
 *                        |/
 *                        ------------> x
 *
 *         指向南方
 * @note 此 API 不包含俯仰角和横滚角补偿
 */
float AK09011C_GET_AZIMUTH_WITHOUT_COMPENSATION(AK09911C_AXIS_DATA data) {
  float x_data, y_data, result_data, cal;

  /* 计算方位角 */
  result_data =
      atan2((float)data.Y_AXIS, (float)data.X_AXIS) * 180.0 / 3.14159 + 180.0;
  return result_data;
}
/*------------------------------------------------------------------------------------------------------*/
/**
 * @brief 简单计算 AK09911C 方位角（带补偿）
 * @param data 磁力计数据
 * @param pitch 俯仰角（来自其他三轴加速度计，如
 * KXSD9、LIS3DH、ADXL345、MMA8452、LSM303DLHC 等）
 * @param roll 横滚角
 * @return 方位角（角度制）
 *
 * @note 坐标系说明：
 *                        y
 *                        ^
 *                        |     /
 *                        |    /
 *                        |   /
 *                        |  /
 *                        | /     degree
 *                        |/
 *                        ------------> x
 *
 *         指向南方
 * @note 此 API 包含俯仰角和横滚角补偿
 */
float AK09011C_GET_AZIMUTH_WITH_COMPENSATION(AK09911C_AXIS_DATA data,
                                             float pitch, float roll) {

  return 0.0;
}
/*------------------------------------------------------------------------------------------------------*/
/********************************************** 高级应用功能
 * **************************************************/
