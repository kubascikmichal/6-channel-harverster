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
#define ADC_CHANNEL    0
#define ADC_PORT_0       SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN0
#define ADC_PORT_1       SAADC_CH_PSELP_PSELP_AnalogInput1 // AIN1
#define ADC_PORT_2       SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN2
#define ADC_PORT_3       SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN3
#define ADC_PORT_4       SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN4
#define ADC_PORT_5       SAADC_CH_PSELP_PSELP_AnalogInput0 // AIN5
#define ADC_REFERENCE  ADC_REF_INTERNAL                  // 0.6v
#define ADC_GAIN       ADC_GAIN_1_5                      // ADC_REFERENCE*5

struct adc_channel_cfg ch0_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_0
#endif
};

struct adc_channel_cfg ch1_cfg = 
{
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 1,
#ifdef CONFIG_ADC_NRFX_SAADC
	.input_positive = ADC_PORT_1
#endif
};

int16_t buffer_0[1];
struct adc_sequence seq_0 = {
	.channels    = BIT(ADC_CHANNEL),
	.buffer      = buffer_0,
	.buffer_size = sizeof(buffer_0),
	.resolution  = ADC_RESOLUTION
};

int16_t buffer_1[1];
struct adc_sequence seq_1 = {
	.channels    = BIT(ADC_CHANNEL),
	.buffer      = buffer_1,
	.buffer_size = sizeof(buffer_1),
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
	if(!device_is_ready(adc_dev_0) || !device_is_ready(adc_dev_1)) {
		printk("ADC device is not ready");
		return;
	}

	err = adc_channel_setup(adc_dev_0, &ch0_cfg);
	err = adc_channel_setup(adc_dev_1, &ch1_cfg);
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
		if(0 != err) {
			printk("adc device read erro %dr, terminating\n\r", err);
			return;
		}
		int32_t mv_0 = buffer_0[0];
		int32_t mv_1 = buffer_1[0];
		int32_t adv_vref_0 = adc_ref_internal(adc_dev_0);
		int32_t adv_vref_1 = adc_ref_internal(adc_dev_1);
		adc_raw_to_millivolts(adv_vref_0, ADC_GAIN, ADC_RESOLUTION, &mv_0);
		adc_raw_to_millivolts(adv_vref_1, ADC_GAIN, ADC_RESOLUTION, &mv_1);
		printk("[%.2f; %.2f]\n\r", mv_0/3000.,mv_1/3000.);
		k_msleep(SLEEP_INTERVAL);
	}
        return 0;
}