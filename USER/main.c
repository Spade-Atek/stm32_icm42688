
#include "example-raw-data-registers.h"
/* InvenSense utils */
//#include "../Invn/EmbUtils/Message.h"
#include "../Invn/EmbUtils/ErrorHelper.h"
//#include "../Invn/EmbUtils/RingBuffer.h"
#include "delay.h"
#include "sys.h"
#include "usart.h" 
#include "icm42688.h"
#include "myiic.h"
#include "IMU.h"
#include "eeprom.h"
#include "spi.h"

#define	SPI_IMU_CS PAout(2)  //选中IMU	
/*
 * Set power mode flag
 * Set this flag to run example in low-noise mode.
 * Reset this flag to run example in low-power mode.
 * Note : low-noise mode is not available with sensor data frequencies less than 12.5Hz.
 */
#define IS_LOW_NOISE_MODE 1

/* 
 * Set this to 0 if you want to test timestamping mechanism without CLKIN 32k capability.
 * Please set a hardware bridge between PA17 (from MCU) and CLKIN pins (to ICM).
 * Warning: This option is not available for all ICM426XX. Please check the datasheet.
 */
#define USE_CLK_IN 0


// 模式选择需要在example-raw-data-registers.h里选择宏定义ICM_USE_HARD_SPI/ICM_USE_I2C

// VCC--------5V或者3.3V都可以
//SPI 模式接线
// PA2------------------------CS
// PB13------------------------SCLK
// PB14------------------------MISO
// PB15------------------------MOSI

//IIC 模式接线
// PB6------------------------SCL
// PB7------------------------SDA
// AD0默认上拉可以不接

int math_pl=0;
#define ACCEL_LSB_PER_G 1000.0f
#define G_TO_MS2 9.80665f

void IMU_ROS(void)
{
    float q[4];       // 四元数
    float imuData[6]; // acc[0..2], gyro[3..5]
    float acc[3], gyro[3];

    // 获取四元数
    IMU_getQ(q);

    // 获取原始传感器数据
    IMU_getValues(imuData);

    // acc 已经在 bsp_IcmGetRawData 里是 mg/dps -> 转成 m/s2
		acc[0] = imuData[0] / ACCEL_LSB_PER_G * G_TO_MS2;
		acc[1] = imuData[1] / ACCEL_LSB_PER_G * G_TO_MS2;
		acc[2] = imuData[2] / ACCEL_LSB_PER_G * G_TO_MS2;
	
    // gyro 已经是 dps -> 转成 rad/s
    gyro[0] = imuData[3] * M_PI / 180.0f;
    gyro[1] = imuData[4] * M_PI / 180.0f;
    gyro[2] = imuData[5] * M_PI / 180.0f;

    // 输出 ROS JSON / rosserial 格式
		printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
					 q[1], q[2], q[3], q[0],   // 四元数 x,y,z,w
					 gyro[0], gyro[1], gyro[2], // 角速度 rad/s
					 acc[0], acc[1], acc[2]     // 线加速度 m/s2
		);
}

/* Flag set from icm426xx device irq handler */
static volatile int irq_from_device;

static void SetupMCUHardware(struct inv_icm426xx_serif *icm_serif);

static void check_rc(int rc, const char *msg_context)
{
	if (rc < 0) {
		printf("%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while (1)
			;
	}
}

int main(void)
{
	int                       rc = 0;
	struct inv_icm426xx_serif icm426xx_serif;
	SystemInit();
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为115200
	printf("OK\r\n");
	IIC_Init();
	SPI2_Init();
	//SPI2_SetSpeed(SPI_BaudRatePrescaler_16);
	//delay_ms(100);
	
	//load_config();
	//delay_ms(50);
	//IMU_init();
	/* Initialize MCU hardware */
	SetupMCUHardware(&icm426xx_serif);

	/* Initialize Icm426xx */
	rc = SetupInvDevice(&icm426xx_serif);
	check_rc(rc, "error while setting up INV device");
	IMU_init();
	
	/* Configure Icm426xx */
	/* /!\ In this example, the data output frequency will be the faster  between Accel and Gyro odr */
	rc = ConfigureInvDevice((uint8_t)IS_LOW_NOISE_MODE, ICM426XX_ACCEL_CONFIG0_FS_SEL_8g,
	                        ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps, ICM426XX_ACCEL_CONFIG0_ODR_200_HZ,
	                        ICM426XX_GYRO_CONFIG0_ODR_200_HZ, (uint8_t)USE_CLK_IN);

	check_rc(rc, "error while configuring INV device");
	delay_ms(100);

	do {
//			rc = GetDataFromInvDevice();
//			check_rc(rc, "error while processing FIFO");
		IMU_ROS();
	delay_ms(10);
	} while (1);	
//	while(1)
//	{	
//		IMU_getYawPitchRoll(ypr);
//		IMU_TT_getgyro(motion6);
//		
//		math_pl++;
//		delay_ms(10);
//	}
}


static uint8_t io_write_reg(uint8_t reg, uint8_t value)
{
#if defined(ICM_USE_HARD_SPI)
    SPI_IMU_CS=0;
    /* 写入要读的寄存器地址 */
    /* 写入要读的寄存器地址 */
    SPI2_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    SPI2_ReadWriteByte(value);
    SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IICwriteBytes(ICM42688_ADDRESS, reg, 1, &value);
#endif
    return 0;
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *rbuffer,
                        uint32_t rlen)
{
	uint32_t len = rlen;
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    SPI_IMU_CS=0;
    /* 写入要读的寄存器地址 */
    SPI2_ReadWriteByte(reg);
    /* 读取寄存器数据 */
    while(len)
	{
		*rbuffer = SPI2_ReadWriteByte(0x00);
		len--;
		rbuffer++;
	}
    SPI_IMU_CS=1;
#elif defined(ICM_USE_I2C)
	IICreadBytes(ICM42688_ADDRESS, reg, len, buf);
#endif

		return 0;
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *wbuffer,
                         uint32_t wlen)
{
	int rc;

	for (uint32_t i = 0; i < wlen; i++) 
	{
		rc = io_write_reg(reg + i, wbuffer[i]);
		if (rc)
			return rc;
	}
	return 0;
}


/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIO so that MCU can receive interrupts from ICM426xx
 *   - a microsecond timer requested by Icm426xx driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a serial link to communicate from MCU to Icm426xx
 */
static void SetupMCUHardware(struct inv_icm426xx_serif *icm_serif)
{

	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context    = 0; /* no need */
	icm_serif->read_reg   = inv_io_hal_read_reg;
	icm_serif->write_reg  = inv_io_hal_write_reg;
	icm_serif->max_read   = 1024 * 3; /* maximum number of bytes allowed per serial read */
	icm_serif->max_write  = 1024 * 3; /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = 0;
	//inv_io_hal_init(icm_serif);
}
void inv_icm426xx_sleep_us(uint32_t us)
{
	delay_us(us);
}

uint64_t inv_icm426xx_get_time_us(void)
{
	return 0;
}


