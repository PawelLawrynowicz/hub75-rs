#![no_std]

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

pub struct Hub75<PINS, const ROW_LENGTH: usize> {
    //r1, g1, b1, r2, g2, b2, column, row
    #[cfg(not(feature = "stripe-multiplexing"))]
    data: [[(u8, u8, u8, u8, u8, u8); ROW_LENGTH]; NUM_ROWS],

    #[cfg(feature = "stripe-multiplexing")]
    data: [[(u8, u8, u8, u8, u8, u8); ROW_LENGTH]; NUM_ROWS / 2],

    output_port: *mut u8,

    brightness_step: u8,
    brightness_count: u8,
    pins: PINS,
}

/// A trait, so that it's easier to reason about the pins
/// Implemented for a tuple `(r1, g1, b1, r2, g2, b2, a, b, c, d, clk, lat, oe)`
/// with every element implementing `OutputPin`
/// f pin is needed for 64x64 matrix support
pub trait Outputs {
    type Error;
    type A: OutputPin<Error = Self::Error>;
    type B: OutputPin<Error = Self::Error>;
    type C: OutputPin<Error = Self::Error>;
    #[cfg(not(feature = "stripe-multiplexing"))]
    type D: OutputPin<Error = Self::Error>;
    #[cfg(feature = "size-64x64")]
    type F: OutputPin<Error = Self::Error>;
    type CLK: OutputPin<Error = Self::Error>;
    type LAT: OutputPin<Error = Self::Error>;
    type OE: OutputPin<Error = Self::Error>;
    fn a(&mut self) -> &mut Self::A;
    fn b(&mut self) -> &mut Self::B;
    fn c(&mut self) -> &mut Self::C;
    #[cfg(not(feature = "stripe-multiplexing"))]
    fn d(&mut self) -> &mut Self::D;
    #[cfg(feature = "size-64x64")]
    fn f(&mut self) -> &mut Self::F;
    fn clk(&mut self) -> &mut Self::CLK;
    fn lat(&mut self) -> &mut Self::LAT;
    fn oe(&mut self) -> &mut Self::OE;
}

#[cfg(feature = "size-64x64")]
#[cfg(not(feature = "stripe-multiplexing"))]
impl<
        E,
        A: OutputPin<Error = E>,
        B: OutputPin<Error = E>,
        C: OutputPin<Error = E>,
        D: OutputPin<Error = E>,
        F: OutputPin<Error = E>,
        CLK: OutputPin<Error = E>,
        LAT: OutputPin<Error = E>,
        OE: OutputPin<Error = E>,
    > Outputs for (A, B, C, D, F, CLK, LAT, OE)
{
    type Error = E;
    type A = A;
    type B = B;
    type C = C;
    type D = D;
    type F = F;
    type CLK = CLK;
    type LAT = LAT;
    type OE = OE;
    fn a(&mut self) -> &mut A {
        &mut self.0
    }
    fn b(&mut self) -> &mut B {
        &mut self.1
    }
    fn c(&mut self) -> &mut C {
        &mut self.2
    }
    fn d(&mut self) -> &mut D {
        &mut self.3
    }
    fn f(&mut self) -> &mut F {
        &mut self.4
    }
    fn clk(&mut self) -> &mut CLK {
        &mut self.5
    }
    fn lat(&mut self) -> &mut LAT {
        &mut self.6
    }
    fn oe(&mut self) -> &mut OE {
        &mut self.7
    }
}

#[cfg(not(feature = "size-64x64"))]
#[cfg(not(feature = "stripe-multiplexing"))]
impl<
        E,
        A: OutputPin<Error = E>,
        B: OutputPin<Error = E>,
        C: OutputPin<Error = E>,
        D: OutputPin<Error = E>,
        CLK: OutputPin<Error = E>,
        LAT: OutputPin<Error = E>,
        OE: OutputPin<Error = E>,
    > Outputs for (A, B, C, D, CLK, LAT, OE)
{
    type Error = E;
    type A = A;
    type B = B;
    type C = C;
    type D = D;
    type CLK = CLK;
    type LAT = LAT;
    type OE = OE;
    fn a(&mut self) -> &mut A {
        &mut self.0
    }
    fn b(&mut self) -> &mut B {
        &mut self.1
    }
    fn c(&mut self) -> &mut C {
        &mut self.2
    }
    fn d(&mut self) -> &mut D {
        &mut self.3
    }
    fn clk(&mut self) -> &mut CLK {
        &mut self.4
    }
    fn lat(&mut self) -> &mut LAT {
        &mut self.5
    }
    fn oe(&mut self) -> &mut OE {
        &mut self.6
    }
}

#[cfg(feature = "stripe-multiplexing")]
impl<
        E,
        A: OutputPin<Error = E>,
        B: OutputPin<Error = E>,
        C: OutputPin<Error = E>,
        CLK: OutputPin<Error = E>,
        LAT: OutputPin<Error = E>,
        OE: OutputPin<Error = E>,
    > Outputs for (A, B, C, CLK, LAT, OE)
{
    type Error = E;
    type A = A;
    type B = B;
    type C = C;
    type CLK = CLK;
    type LAT = LAT;
    type OE = OE;
    fn a(&mut self) -> &mut A {
        &mut self.0
    }
    fn b(&mut self) -> &mut B {
        &mut self.1
    }
    fn c(&mut self) -> &mut C {
        &mut self.2
    }
    fn clk(&mut self) -> &mut CLK {
        &mut self.3
    }
    fn lat(&mut self) -> &mut LAT {
        &mut self.4
    }
    fn oe(&mut self) -> &mut OE {
        &mut self.5
    }
}

impl<PINS: Outputs, const ROW_LENGTH: usize> Hub75<PINS, ROW_LENGTH> {
    /// Create a new hub instance
    /// `brightness_bits` provides the number of brightness_bits for each color (1-8).
    /// More bits allow for much more colors, especially in combination with the gamma correction,
    /// but each extra bit doubles the time `output` will take. This might lead to noticable flicker.
    ///
    /// 3-4 bits are usually a good choice.
    /// IMPORTANT: When using stripe multiplexing set row width to double of the actual width of the screen.
    pub fn new(pins: PINS, brightness_bits: u8, output_port: &mut u8) -> Self {
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
            output_port,
            pins
        }
    }

    /// Output the buffer to the display
    ///
    /// Takes some time and should be called quite often, otherwise the output
    /// will flicker
    pub fn output<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY) -> Result<(), PINS::Error> {
        // Enable the output
        // The previous last row will continue to display
        self.pins.oe().set_low()?;
        // PWM cycle
        for mut brightness in 0..self.brightness_count {
            brightness = (brightness + 1).saturating_mul(self.brightness_step);
            self.output_single(delay, brightness)?;
        }
        // Disable the output
        // Prevents one row from being much brighter than the others
        self.pins.oe().set_high()?;
        Ok(())
    }

    pub fn output_single<DELAY: DelayUs<u8>>(&mut self, delay: &mut DELAY, brightness: u8) -> Result<(), PINS::Error>{
        for (count, row) in self.data.iter().enumerate() {
            for element in row.iter() {
                //Assuming data pins are connected to consecutive pins of a single port starting ftom P0
                //in this order: r1,g1,b1,r2,g2,b2
                unsafe {
                    *self.output_port = 0;

                    if element.0 >= brightness {
                        *self.output_port += 1;
                    }
                    if element.1 >= brightness {
                        *self.output_port += 2;
                    }
                    if element.2 >= brightness {
                        *self.output_port += 4;
                    }
                    if element.3 >= brightness {
                        *self.output_port += 8;
                    }
                    if element.4 >= brightness {
                        *self.output_port += 16;
                    }
                    if element.5 >= brightness {
                        *self.output_port += 32;
                    }
                }
                self.pins.clk().set_high()?;
                self.pins.clk().set_low()?;
            }
            self.pins.oe().set_high()?;
            // Prevents ghosting, no idea why
            delay.delay_us(2);
            self.pins.lat().set_low()?;
            delay.delay_us(2);
            self.pins.lat().set_high()?;
            // Select row
            if count & 1 != 0 {
                self.pins.a().set_high()?;
            } else {
                self.pins.a().set_low()?;
            }
            if count & 2 != 0 {
                self.pins.b().set_high()?;
            } else {
                self.pins.b().set_low()?;
            }
            if count & 4 != 0 {
                self.pins.c().set_high()?;
            } else {
                self.pins.c().set_low()?;
            }
            #[cfg(not(feature = "stripe-multiplexing"))]
            if count & 8 != 0 {
                self.pins.d().set_high()?;
            } else {
                self.pins.d().set_low()?;
            }
            #[cfg(feature = "size-64x64")]
            if count & 16 != 0 {
                self.pins.f().set_high()?;
            } else {
                self.pins.f().set_low()?;
            }
            delay.delay_us(2);
            self.pins.oe().set_low()?;
        }

        Ok(())
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

impl<PINS: Outputs, const ROW_LENGTH: usize> DrawTarget<Rgb888> for Hub75<PINS, ROW_LENGTH> {
    type Error = core::convert::Infallible;

    #[cfg(not(feature = "stripe-multiplexing"))]
    fn draw_pixel(&mut self, item: Pixel<Rgb888>) -> Result<(), Self::Error> {
        let Pixel(coord, color) = item;

        let column = coord[0];
        let row = coord[1];

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
