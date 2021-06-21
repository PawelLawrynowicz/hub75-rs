#![no_std]
#![feature(const_generics)]

use core::usize;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::OutputPin;
// Inspired by
// - https://github.com/polyfloyd/ledcat/blob/master/src/device/hub75.rs
// - https://github.com/mmou/led-marquee/blob/8c88531a6938edff6db829ca21c15304515874ea/src/hub.rs
// - https://github.com/adafruit/RGB-matrix-Panel/blob/master/RGBmatrixPanel.cpp
// - https://www.mikrocontroller.net/topic/452187 (sorry, german only)

/// # Theory of Operation
/// This display is essentially split in half, with the top 16 rows being
/// controlled by one set of shift registers (r1, g1, b1) and the botton 16
/// rows by another set (r2, g2, b2). So, the best way to update it is to
/// show one of the botton and top rows in tandem. The row (between 0-15) is then
/// selected by the A, B, C, D pins, which are just, as one might expect, the bits 0 to 3.
/// Pin F is used by the 64x64 display to get 5 bit row addressing (1/32 row scan rate)
///
/// The display doesn't really do brightness, so we have to do it ourselves, by
/// rendering the same frame multiple times, with some pixels being turned of if
/// they are darker (pwm)

#[cfg(feature = "size-64x64")]
const NUM_ROWS: usize = 32;
#[cfg(not(feature = "size-64x64"))]
const NUM_ROWS: usize = 16;

const GAMMA8: [u8; 256] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
    5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14,
    14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27,
    27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46,
    47, 48, 49, 50, 50, 51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68, 69, 70, 72,
    73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104,
    105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137,
    138, 140, 142, 144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
    177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213, 215, 218, 220,
    223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255,
];
#[derive(PartialEq, Eq)]
pub struct Pins {
    pub r1: u16,
    pub g1: u16,
    pub b1: u16,
    pub r2: u16,
    pub g2: u16,
    pub b2: u16,
    pub a: u16,
    pub b: u16,
    pub c: u16,
    pub clock: u16,
    pub latch: u16,
    pub oe: u16,
}
pub struct Hub75<const PIN_POS: Pins, const ROW_LENGTH: usize> {
    //r1, g1, b1, r2, g2, b2, column, row
    #[cfg(not(feature = "stripe-multiplexing"))]
    data: [[(u8, u8, u8, u8, u8, u8); ROW_LENGTH]; NUM_ROWS],

    #[cfg(feature = "stripe-multiplexing")]
    data: [[(u8, u8, u8, u8, u8, u8); ROW_LENGTH]; NUM_ROWS / 2],

    output_port: *mut u16,

    brightness_step: u8,
    brightness_count: u8,
    brightness_bits: u8,
}

impl<const PIN_POS: Pins, const ROW_LENGTH: usize> Hub75<PIN_POS, ROW_LENGTH> {
    const PINS: Pins = Pins {
        r1: 1 << PIN_POS.r1,
        g1: 1 << PIN_POS.g1,
        b1: 1 << PIN_POS.b1,
        r2: 1 << PIN_POS.r2,
        g2: 1 << PIN_POS.g2,
        b2: 1 << PIN_POS.b2,
        a: 1 << PIN_POS.a,
        b: 1 << PIN_POS.b,
        c: 1 << PIN_POS.c,
        clock: 1 << PIN_POS.clock,
        latch: 1 << PIN_POS.latch,
        oe: 1 << PIN_POS.oe,
    };

    /// TODO: Write better documentation
    /// color_pins are numbers of pins r1, g1, b1, r2, g2, b2, A, B, C, clock, latch, OE
    pub fn new(brightness_bits: u8, output_port: &mut u16) -> Self {
        assert!(brightness_bits < 9 && brightness_bits > 0);

        #[cfg(not(feature = "stripe-multiplexing"))]
        let data = [[(0, 0, 0, 0, 0, 0); ROW_LENGTH]; NUM_ROWS];
        #[cfg(feature = "stripe-multiplexing")]
        let data = [[(0, 0, 0, 0, 0, 0); ROW_LENGTH]; NUM_ROWS / 2];

        let brightness_step = 1 << (8 - brightness_bits);
        let brightness_count = ((1 << brightness_bits as u16) - 1) as u8;

        Self {
            data,
            brightness_step,
            brightness_count,
            brightness_bits,
            output_port,
        }
    }

    /// Output the buffer to the display
    ///
    /// Takes some time and should be called quite often, otherwise the output
    /// will flicker
    pub fn output<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY) {
        // PWM cycle
        for mut brightness in 0..self.brightness_count {
            brightness = (brightness + 1).saturating_mul(self.brightness_step);
            self.output_single(delay, brightness);
        }
    }

    pub fn output_single<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY, brightness: u8) {
        for (count, row) in self.data.iter().enumerate() {
            let mut address = 0;
            let mut output_buffer = Self::PINS.latch + address;

            for element in row.iter() {
                output_buffer = Self::PINS.latch + address;
                //Assuming data pins are connected to consecutive pins of a single port starting ftom P0
                //in this order: r1,g1,b1,r2,g2,b2

                if element.0 >= brightness {
                    output_buffer += Self::PINS.r1;
                }
                if element.1 >= brightness {
                    output_buffer += Self::PINS.g1;
                }
                if element.2 >= brightness {
                    output_buffer += Self::PINS.b1;
                }
                if element.3 >= brightness {
                    output_buffer += Self::PINS.r2;
                }
                if element.4 >= brightness {
                    output_buffer += Self::PINS.g2;
                }
                if element.5 >= brightness {
                    output_buffer += Self::PINS.b2;
                }

                //clock will be set to high when we push out values
                output_buffer += Self::PINS.clock;

                unsafe {
                    *self.output_port = output_buffer;
                    //set clock low
                    output_buffer -= Self::PINS.clock;
                    *self.output_port = output_buffer;
                }
            }
            output_buffer += Self::PINS.oe;
            output_buffer -= Self::PINS.latch;
            unsafe {
                *self.output_port = output_buffer;
            }
            output_buffer += Self::PINS.latch;
            delay.delay_us(1);
            unsafe {
                *self.output_port = output_buffer;
            }

            /*self.pins.oe().set_high()?;
            // Prevents ghosting, no idea why
            delay.delay_us(1);
            self.pins.lat().set_low()?;
            delay.delay_us(1);
            self.pins.lat().set_high()?;
            // Select row*/

            address = 0;

            if count & 1 != 0 {
                address += Self::PINS.a;
            }
            if count & 2 != 0 {
                address += Self::PINS.b;
            }
            if count & 4 != 0 {
                address += Self::PINS.c;
            }

            output_buffer += address;

            unsafe {
                *self.output_port = output_buffer;
            }

            /*delay.delay_us(1);
            self.pins.oe().set_low()?;*/

            output_buffer -= Self::PINS.oe;

            unsafe {
                *self.output_port = output_buffer;
            }
        }
    }

    pub fn output_bcm<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY, delay_base_us: u8) {
        let shift = 8 - self.brightness_bits;

        // PWM cycle
        for bit in 0..self.brightness_bits {
            self.output_single_bcm(delay, bit + shift);
            delay.delay_us(delay_base_us * (1 << bit))
        }
    }

    pub fn output_single_bcm<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY, bit: u8) {
        let mask = 1 << bit;
        //derived empirically, without it the last row will be dimmer than others
        let delay_after_last_row = (5 * ROW_LENGTH / 64) as u8;

        //hacky, but it's the most efficient way. We need to make sure oe is HIGH when pushing color bits, but only during first iteration.
        //By assigning it here we don't have to check a condition every iteration of inner loop;
        let mut address = Self::PINS.oe;
        let mut output_buffer = 0;

        for (count, row) in self.data.iter().enumerate() {
            for element in row.iter() {
                output_buffer = address;

                //Assuming data pins are connected to consecutive pins of a single port starting ftom P0
                //in this order: r1,g1,b1,r2,g2,b2
                if element.0 & mask != 0 {
                    output_buffer += Self::PINS.r1;
                }
                if element.1 & mask != 0 {
                    output_buffer += Self::PINS.g1;
                }
                if element.2 & mask != 0 {
                    output_buffer += Self::PINS.b1;
                }
                if element.3 & mask != 0 {
                    output_buffer += Self::PINS.r2;
                }
                if element.4 & mask != 0 {
                    output_buffer += Self::PINS.g2;
                }
                if element.5 & mask != 0 {
                    output_buffer += Self::PINS.b2;
                }

                output_buffer += Self::PINS.clock;

                unsafe {
                    *self.output_port = output_buffer;
                    output_buffer -= Self::PINS.clock;
                    *self.output_port = output_buffer;
                }
            }

            output_buffer |= Self::PINS.oe;
            output_buffer &= !Self::PINS.latch;

            unsafe {
                *self.output_port = output_buffer;
            }

            output_buffer |= Self::PINS.latch;

            address = 0;

            if count & 1 != 0 {
                address += Self::PINS.a;
            }
            if count & 2 != 0 {
                address += Self::PINS.b;
            }
            if count & 4 != 0 {
                address += Self::PINS.c;
            }

            output_buffer &= !(Self::PINS.a + Self::PINS.b + Self::PINS.c);
            output_buffer += address;

            unsafe {
                *self.output_port = output_buffer;
            }

            output_buffer &= !Self::PINS.oe;

            delay.delay_us(1);

            unsafe {
                *self.output_port = output_buffer;
            }
        }

        //prevents last row from being brighter
        delay.delay_us(delay_after_last_row);

        output_buffer |= Self::PINS.oe;
        unsafe{
            *self.output_port = output_buffer;
        }
    }

    /// Clear the output
    ///
    /// It's a bit faster than using the embedded_graphics interface
    /// to do the same
    pub fn clear_display(&mut self) {
        for row in self.data.iter_mut() {
            for e in row.iter_mut() {
                e.0 = 0;
                e.1 = 0;
                e.2 = 0;
                e.3 = 0;
                e.4 = 0;
                e.5 = 0;
            }
        }
    }
}

use embedded_graphics::{
    drawable::Pixel,
    pixelcolor::{Rgb888, RgbColor},
    prelude::Size,
    DrawTarget,
};

impl<const PIN_POS: Pins, const ROW_LENGTH: usize> DrawTarget<Rgb888> for Hub75<PIN_POS, ROW_LENGTH> {
    type Error = core::convert::Infallible;

    #[cfg(not(feature = "stripe-multiplexing"))]
    fn draw_pixel(&mut self, item: Pixel<Rgb888>) -> Result<(), Self::Error> {
        let Pixel(coord, color) = item;

        let column = coord[0];
        let row = coord[1];

        if column < 0 || column >= ROW_LENGTH as i32|| row < 0 || row >= (NUM_ROWS * 2) as i32{
            return Ok(());
        }

        let mut pixel_tuple = &mut self.data[row as usize % NUM_ROWS][column as usize];

        if row > 15 {
            pixel_tuple.3 = GAMMA8[color.r() as usize];
            pixel_tuple.4 = GAMMA8[color.g() as usize];
            pixel_tuple.5 = GAMMA8[color.b() as usize];
        } else {
            pixel_tuple.0 = GAMMA8[color.r() as usize];
            pixel_tuple.1 = GAMMA8[color.g() as usize];
            pixel_tuple.2 = GAMMA8[color.b() as usize];
        }

        Ok(())
    }

    #[cfg(feature = "stripe-multiplexing")]
    fn draw_pixel(&mut self, item: Pixel<Rgb888>) -> Result<(), Self::Error> {
        let Pixel(coord, color) = item;

        let mut x = coord[0] as usize;
        let mut y = coord[1] as usize;

        if (x < 0 || x >= ROW_LENGTH / 2 || y < 0 || y >= NUM_ROWS * 2){
            return Ok(());
        }

        let is_top_stripe = (y % NUM_ROWS) < NUM_ROWS / 2;

        let screen_offset = x / 32;

        x = x + (screen_offset * 32);

        if is_top_stripe {
            x = x + 32;
        }

        let column = x;
        let row = y % (NUM_ROWS / 2);

        let mut pixel_tuple = &mut self.data[row as usize][column as usize];

        if y > 15 {
            pixel_tuple.3 = GAMMA8[color.r() as usize];
            pixel_tuple.4 = GAMMA8[color.g() as usize];
            pixel_tuple.5 = GAMMA8[color.b() as usize];
        } else {
            pixel_tuple.0 = GAMMA8[color.r() as usize];
            pixel_tuple.1 = GAMMA8[color.g() as usize];
            pixel_tuple.2 = GAMMA8[color.b() as usize];
        }

        Ok(())
    }

    fn draw_iter<T>(&mut self, item: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = Pixel<Rgb888>>,
    {
        let pixels = item.into_iter();

        for pixel in pixels {
            self.draw_pixel(pixel).unwrap();
        }

        Ok(())
    }

    fn clear(&mut self, color: Rgb888) -> Result<(), Self::Error> {
        #[cfg(not(feature = "stripe-multiplexing"))]
        let rows = NUM_ROWS;
        #[cfg(feature = "stripe-multiplexing")]
        let rows = NUM_ROWS / 2;

        for row in 0..rows {
            for column in 0..ROW_LENGTH {
                let pixel_tuple = &mut self.data[row][column];
                pixel_tuple.0 = GAMMA8[color.r() as usize];
                pixel_tuple.1 = GAMMA8[color.g() as usize];
                pixel_tuple.2 = GAMMA8[color.b() as usize];
                pixel_tuple.3 = GAMMA8[color.r() as usize];
                pixel_tuple.4 = GAMMA8[color.g() as usize];
                pixel_tuple.5 = GAMMA8[color.b() as usize];
            }
        }

        Ok(())
    }

    fn size(&self) -> Size {
        Size {
            #[cfg(not(feature = "stripe-multiplexing"))]
            width: ROW_LENGTH as u32,
            #[cfg(feature = "stripe-multiplexing")]
            width: (ROW_LENGTH as u32) / 2,
            height: (NUM_ROWS * 2) as u32,
        }
    }
}
