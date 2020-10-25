#![no_std]
#![no_main]

use panic_halt as _;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitive_style;
use embedded_graphics::primitives::Rectangle;
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::pac;
use gd32vf103xx_hal::prelude::*;
use longan_nano::{lcd, lcd_pins};
use riscv_rt::entry;

fn enable_adc(adc: &pac::ADC0, delay: &mut McycleDelay) {
    let rcu = unsafe { &*pac::RCU::ptr() };

    /*rcu_periph_clock_enable(RCU_ADC0) */
    rcu.apb2en.modify(|_, w| w.adc0en().set_bit());

    /*rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8); */
    rcu.cfg0
        .modify(|_, w| unsafe { w.adcpsc_1_0().bits(0b11).adcpsc_2().clear_bit() });

    /* adc_deinit(ADC0) */
    rcu.apb2rst.modify(|_, w| w.adc0rst().set_bit());
    rcu.apb2rst.modify(|_, w| w.adc0rst().clear_bit());

    /* adc_mode_config(ADC_MODE_FREE) */
    adc.ctl0.write(|w| unsafe { w.syncm().bits(0) });

    /* adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE); */
    adc.ctl0.modify(|_, w| w.sm().set_bit());

    /* adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT); */
    adc.ctl1.modify(|_, w| w.dal().clear_bit());

    /* adc_tempsensor_vrefint_enable(); */
    adc.ctl1.modify(|_, w| w.tsvren().set_bit());

    /* adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2); */
    adc.isq.modify(|_, w| unsafe { w.il().bits(2 - 1) });

    /* adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5); */
    adc.isq.modify(|_, w| unsafe { w.isq2().bits(16) });
    adc.sampt0.modify(|_, w| unsafe { w.spt16().bits(7) });

    /* adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5); */
    adc.isq.modify(|_, w| unsafe { w.isq3().bits(17) });
    adc.sampt0.modify(|_, w| unsafe { w.spt17().bits(7) });
    /* adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_NONE); */
    adc.ctl1.modify(|_, w| unsafe { w.etsic().bits(7) });

    /* adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE); */
    adc.ctl1.modify(|_, w| w.eteic().set_bit());

    /* adc_enable(ADC0); */
    adc.ctl1.modify(|r, w| {
        if r.adcon().bit_is_clear() {
            w.adcon().set_bit()
        } else {
            w
        }
    });

    delay.delay_ms(1);

    /* adc_calibration_enable(ADC0); */
    adc.ctl1.modify(|_, w| w.rstclb().set_bit());
    while adc.ctl1.read().rstclb().bit_is_set() {}

    adc.ctl1.modify(|_, w| w.clb().set_bit());
    while adc.ctl1.read().clb().bit_is_set() {}
}

fn get_temp(adc: &pac::ADC0, delay: &mut McycleDelay) -> f32 {
    /* adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL); */
    adc.ctl1.modify(|_, w| w.swicst().set_bit());

    delay.delay_ms(2000);

    let idata0 = adc.idata0.read().idatan().bits() as f32;
    let temp = (1.42 - idata0 * 3.3 / 4096.0) * 1000.0 / 4.3 + 25.0;
    temp
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();
    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);

    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (lcd.size().width as i32, lcd.size().height as i32);

    let mut delay = McycleDelay::new(&rcu.clocks);

    enable_adc(&dp.ADC0, &mut delay);

    let temp_initial = get_temp(&dp.ADC0, &mut delay);

    loop {
        let temp = get_temp(&dp.ADC0, &mut delay);

        // Position the inital into the middle of the screen
        let position = ((temp - temp_initial) * (width as f32 / 10.0)) as i32 + width / 2;
        // Clear screen
        Rectangle::new(Point::new(0, 0), Point::new(width - 1, height - 1))
            .into_styled(primitive_style!(fill_color = Rgb565::BLACK))
            .draw(&mut lcd)
            .unwrap();

        Rectangle::new(Point::new(0, 0), Point::new(position, height - 1))
            .into_styled(primitive_style!(fill_color = Rgb565::RED))
            .draw(&mut lcd)
            .unwrap();
    }
}
