#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

//LOG_MODULE_REGISTER(sensor, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(ctrl, LOG_LEVEL_DBG);

#define SLEEP_INTERVAL 50
#define ADC_NODE DT_NODELABEL(adc)
static const struct device *adc_dev_0 = DEVICE_DT_GET(ADC_NODE);
static const struct device *adc_dev_1 = DEVICE_DT_GET(ADC_NODE);
static const struct device *adc_dev_2 = DEVICE_DT_GET(ADC_NODE);
static const struct device *adc_dev_3 = DEVICE_DT_GET(ADC_NODE);
static const struct device *adc_dev_4 = DEVICE_DT_GET(ADC_NODE);
static const struct device *adc_dev_5 = DEVICE_DT_GET(ADC_NODE);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)


static const struct gpio_dt_spec ledR = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec ledG = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec ledB = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

// nRF52840 AnalogInput0 - pin
// AIN0 p0.02
// AIN1 p0.03
// AIN2 p0.04
// AIN3 p0.05
// AIN4 p0.028
// AIN5 p0.029
// AIN6 p0.030
// AIN7 p0.031
#define ADC_RESOLUTION 10                                // the reading value range is 0 to 1023
#define ADC_CHANNEL_0    0                               
#define ADC_CHANNEL_1    1
#define ADC_CHANNEL_2    2
#define ADC_CHANNEL_3    3
#define ADC_CHANNEL_4    4
#define ADC_CHANNEL_5    5
#define ADC_PORT_0       SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN0
#define ADC_PORT_1       SAADC_CH_PSELP_PSELP_AnalogInput1 // AIN1
#define ADC_PORT_2       SAADC_CH_PSELP_PSELP_AnalogInput2 // AIN2
#define ADC_PORT_3       SAADC_CH_PSELP_PSELP_AnalogInput3 // AIN3
#define ADC_PORT_4       SAADC_CH_PSELP_PSELP_AnalogInput4 // AIN4
#define ADC_PORT_5       SAADC_CH_PSELP_PSELP_AnalogInput5 // AIN5
#define ADC_REFERENCE  	ADC_REF_INTERNAL                  // 0.6v
#define ADC_GAIN       	ADC_GAIN_1_5                      // ADC_REFERENCE*5

struct adc_channel_cfg ch0_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_0,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_0
#endif
};

struct adc_channel_cfg ch1_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_1,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_1
#endif
};

struct adc_channel_cfg ch2_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_2,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_2
#endif
};

struct adc_channel_cfg ch3_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_3,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_3
#endif
};

struct adc_channel_cfg ch4_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_5,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_4
#endif
};

struct adc_channel_cfg ch5_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL_5,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_5
#endif
};

int16_t buffer_0[1];
struct adc_sequence seq_0 = {
	.channels    = BIT(ADC_CHANNEL_0),
	.buffer      = buffer_0,
	.buffer_size = sizeof(buffer_0),
	.resolution  = ADC_RESOLUTION
};

int16_t buffer_1[1];
struct adc_sequence seq_1 = {
	.channels    = BIT(ADC_CHANNEL_1),
	.buffer      = buffer_1,
	.buffer_size = sizeof(buffer_1),
	.resolution  = ADC_RESOLUTION
};

int16_t buffer_2[1];
struct adc_sequence seq_2 = {
	.channels    = BIT(ADC_CHANNEL_2),
	.buffer      = buffer_2,
	.buffer_size = sizeof(buffer_2),
	.resolution  = ADC_RESOLUTION
};

int16_t buffer_3[1];
struct adc_sequence seq_3 = {
	.channels    = BIT(ADC_CHANNEL_3),
	.buffer      = buffer_3,
	.buffer_size = sizeof(buffer_3),
	.resolution  = ADC_RESOLUTION
};

int16_t buffer_4[1];
struct adc_sequence seq_4 = {
	.channels    = BIT(ADC_CHANNEL_4),
	.buffer      = buffer_4,
	.buffer_size = sizeof(buffer_4),
	.resolution  = ADC_RESOLUTION
};

int16_t buffer_5[1];
struct adc_sequence seq_5 = {
	.channels    = BIT(ADC_CHANNEL_5),
	.buffer      = buffer_5,
	.buffer_size = sizeof(buffer_5),
	.resolution  = ADC_RESOLUTION
};

void setupRGB(){
        gpio_pin_configure_dt(&ledR, GPIO_OUTPUT_ACTIVE);
        gpio_pin_configure_dt(&ledG, GPIO_OUTPUT_ACTIVE);
        gpio_pin_configure_dt(&ledB, GPIO_OUTPUT_ACTIVE);
}
void rgb(bool r, bool g, bool b){
        gpio_pin_set_dt(&ledR, r);
        gpio_pin_set_dt(&ledG, g);
        gpio_pin_set_dt(&ledB, b);
}

void main(void)
{
	int err = 0;
        printk("Starting\n\r");
	if(!device_is_ready(adc_dev_0) || !device_is_ready(adc_dev_1) || 
	!device_is_ready(adc_dev_2) || !device_is_ready(adc_dev_3) ||
	!device_is_ready(adc_dev_4) || !device_is_ready(adc_dev_5)) {
		printk("ADC device is not ready");
		return;
	}

	err = adc_channel_setup(adc_dev_0, &ch0_cfg);
	if(err != 0)
		printk("adc0 device setup read erro %d", err);
	err = adc_channel_setup(adc_dev_1, &ch1_cfg);
	if(err != 0)
		printk("adc1 device setup read erro %d", err);
	err = adc_channel_setup(adc_dev_2, &ch2_cfg);
	if(err != 0)
		printk("adc2 device setup read erro %d", err);
	err = adc_channel_setup(adc_dev_3, &ch3_cfg);
	if(err != 0)
		printk("adc3 device setup read erro %d", err);
	err = adc_channel_setup(adc_dev_4, &ch4_cfg);
	if(err != 0)
		printk("adc4 device setup read erro %d", err);
	err = adc_channel_setup(adc_dev_5, &ch5_cfg);
	if(err != 0)
		printk("adc5 device setup read erro %d", err);
	setupRGB();
	rgb(1,0,0);
	k_msleep(500);
	rgb(0,1,0);
	k_msleep(500);
	rgb(0,0,1);
	k_msleep(500);
	rgb(0,0,0);
	while (1)
	{
	    err = adc_read(adc_dev_0, &seq_0);
	    err = adc_read(adc_dev_1, &seq_1);
	    err = adc_read(adc_dev_2, &seq_2);
	    err = adc_read(adc_dev_3, &seq_3);
	    err = adc_read(adc_dev_4, &seq_4);
	    err = adc_read(adc_dev_5, &seq_5);
		int32_t adv_vref = adc_ref_internal(adc_dev_0);
		int32_t mv_0 = buffer_0[0];
		int32_t mv_1 = buffer_1[0];
		int32_t mv_2 = buffer_2[0];
		int32_t mv_3 = buffer_3[0];
		int32_t mv_4 = buffer_4[0];
		int32_t mv_5 = buffer_5[0];
		adc_raw_to_millivolts(adv_vref, ADC_GAIN, ADC_RESOLUTION, &mv_0);
		adc_raw_to_millivolts(adv_vref, ADC_GAIN, ADC_RESOLUTION, &mv_1);
		adc_raw_to_millivolts(adv_vref, ADC_GAIN, ADC_RESOLUTION, &mv_2);
		adc_raw_to_millivolts(adv_vref, ADC_GAIN, ADC_RESOLUTION, &mv_3);
		adc_raw_to_millivolts(adv_vref, ADC_GAIN, ADC_RESOLUTION, &mv_4);
		adc_raw_to_millivolts(adv_vref, ADC_GAIN, ADC_RESOLUTION, &mv_5);
		printk("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n\r", mv_0/3000.,mv_1/3000.,mv_2/3000.,mv_3/3000.,mv_4/3000.,mv_5/3000.);
		k_msleep(SLEEP_INTERVAL);
	}
        return 0;
}