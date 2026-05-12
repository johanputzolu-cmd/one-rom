// Copyright (c) 2026 Piers Finlayson <piers@piers.rocks>
//
// MIT licence

use alloc::{vec, vec::Vec};
use embassy_rp::gpio::Flex;
use embassy_rp::peripherals::*;

pub fn get_board() -> impl Board {
    #[cfg(feature = "fire-40-a")]
    return Fire40A::new();
    #[cfg(feature = "fire-32-a")]
    return Fire32A::new();
    #[cfg(feature = "fire-28-a")]
    return Fire28A::new();
    #[cfg(feature = "fire-24-e")]
    return Fire24E::new();
}

#[cfg(feature = "eprom-16bit")]
pub trait Board {
    fn new() -> Self
    where
        Self: Sized;
    // Ordered A0-Amax
    fn addr_pins(&self) -> [Flex<'static>; 19];
    // Ordered D0-D15
    fn data_pins(&self) -> [Flex<'static>; 16];
    // /OE and /CE in that order
    fn cs_pins(&self) -> Vec<Flex<'static>>;
    // ROM type dependent
    fn special_pins(&self) -> Vec<Flex<'static>>;
    // Board status LED
    fn led_pins(&self) -> [Flex<'static>; 1];
    // Board image sel pins
    #[allow(dead_code)]
    fn sel_pins(&self) -> Vec<Flex<'static>>;
}
#[cfg(feature = "eprom-8bit")]
pub trait Board {
    fn new() -> Self
    where
        Self: Sized;
    // Ordered A0-Amax
    #[cfg(feature = "pin-32")]
    fn addr_pins(&self) -> [Flex<'static>; 19];
    #[cfg(feature = "pin-28")]
    fn addr_pins(&self) -> [Flex<'static>; 16];
    #[cfg(feature = "pin-24")]
    fn addr_pins(&self) -> [Flex<'static>; 12];
    // Ordered D0-D7
    fn data_pins(&self) -> [Flex<'static>; 8];
    // /OE and /CE in that order, then CSn if unique pins
    fn cs_pins(&self) -> Vec<Flex<'static>>;
    // ROM type dependent
    #[allow(dead_code)]
    fn special_pins(&self) -> Vec<Flex<'static>>;
    // Board status LED
    fn led_pins(&self) -> [Flex<'static>; 1];
    // Board image sel pins
    #[allow(dead_code)]
    fn sel_pins(&self) -> Vec<Flex<'static>>;
}

#[cfg(feature = "fire-40-a")]
pub struct Fire40A {}

#[cfg(feature = "fire-40-a")]
impl Board for Fire40A {
    fn new() -> Self {
        Self {}
    }

    fn addr_pins(&self) -> [Flex<'static>; 19] {
        [
            Flex::new(unsafe { PIN_37::steal() }),
            Flex::new(unsafe { PIN_36::steal() }),
            Flex::new(unsafe { PIN_35::steal() }),
            Flex::new(unsafe { PIN_34::steal() }),
            Flex::new(unsafe { PIN_33::steal() }),
            Flex::new(unsafe { PIN_32::steal() }),
            Flex::new(unsafe { PIN_31::steal() }),
            Flex::new(unsafe { PIN_30::steal() }),
            Flex::new(unsafe { PIN_29::steal() }),
            Flex::new(unsafe { PIN_27::steal() }),
            Flex::new(unsafe { PIN_26::steal() }),
            Flex::new(unsafe { PIN_25::steal() }),
            Flex::new(unsafe { PIN_24::steal() }),
            Flex::new(unsafe { PIN_23::steal() }),
            Flex::new(unsafe { PIN_22::steal() }),
            Flex::new(unsafe { PIN_21::steal() }),
            Flex::new(unsafe { PIN_20::steal() }),
            Flex::new(unsafe { PIN_19::steal() }),
            Flex::new(unsafe { PIN_28::steal() }),
        ]
    }

    fn cs_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_16::steal() }),
            Flex::new(unsafe { PIN_17::steal() }),
        ]
    }

    fn special_pins(&self) -> Vec<Flex<'static>> {
        vec![Flex::new(unsafe { PIN_18::steal() })]
    }

    fn data_pins(&self) -> [Flex<'static>; 16] {
        [
            Flex::new(unsafe { PIN_0::steal() }),
            Flex::new(unsafe { PIN_1::steal() }),
            Flex::new(unsafe { PIN_2::steal() }),
            Flex::new(unsafe { PIN_3::steal() }),
            Flex::new(unsafe { PIN_4::steal() }),
            Flex::new(unsafe { PIN_5::steal() }),
            Flex::new(unsafe { PIN_6::steal() }),
            Flex::new(unsafe { PIN_7::steal() }),
            Flex::new(unsafe { PIN_8::steal() }),
            Flex::new(unsafe { PIN_9::steal() }),
            Flex::new(unsafe { PIN_10::steal() }),
            Flex::new(unsafe { PIN_11::steal() }),
            Flex::new(unsafe { PIN_12::steal() }),
            Flex::new(unsafe { PIN_13::steal() }),
            Flex::new(unsafe { PIN_14::steal() }),
            Flex::new(unsafe { PIN_15::steal() }),
        ]
    }

    fn led_pins(&self) -> [Flex<'static>; 1] {
        [Flex::new(unsafe { PIN_42::steal() })]
    }

    fn sel_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_40::steal() }),
            Flex::new(unsafe { PIN_41::steal() }),
            Flex::new(unsafe { PIN_39::steal() }),
            Flex::new(unsafe { PIN_39::steal() }),
        ]
    }
}

#[cfg(feature = "fire-32-a")]
pub struct Fire32A {}

#[cfg(feature = "fire-32-a")]
impl Board for Fire32A {
    fn new() -> Self {
        Self {}
    }

    fn addr_pins(&self) -> [Flex<'static>; 19] {
        [
            Flex::new(unsafe { PIN_34::steal() }),
            Flex::new(unsafe { PIN_33::steal() }),
            Flex::new(unsafe { PIN_32::steal() }),
            Flex::new(unsafe { PIN_31::steal() }),
            Flex::new(unsafe { PIN_30::steal() }),
            Flex::new(unsafe { PIN_29::steal() }),
            Flex::new(unsafe { PIN_28::steal() }),
            Flex::new(unsafe { PIN_27::steal() }),
            Flex::new(unsafe { PIN_20::steal() }),
            Flex::new(unsafe { PIN_19::steal() }),
            Flex::new(unsafe { PIN_17::steal() }),
            Flex::new(unsafe { PIN_18::steal() }),
            Flex::new(unsafe { PIN_26::steal() }),
            Flex::new(unsafe { PIN_21::steal() }),
            Flex::new(unsafe { PIN_22::steal() }),
            Flex::new(unsafe { PIN_25::steal() }),
            Flex::new(unsafe { PIN_16::steal() }),
            Flex::new(unsafe { PIN_23::steal() }),
            Flex::new(unsafe { PIN_24::steal() }),
        ]
    }

    fn cs_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_14::steal() }),
            Flex::new(unsafe { PIN_15::steal() }),
        ]
    }

    fn special_pins(&self) -> Vec<Flex<'static>> {
        vec![
            // Second /OE instance, used as A16 for 27C301
            Flex::new(unsafe { PIN_35::steal() }),
            // A19 used for 27C080 (stacked One ROMs)
            Flex::new(unsafe { PIN_13::steal() }),
        ]
    }

    fn data_pins(&self) -> [Flex<'static>; 8] {
        [
            Flex::new(unsafe { PIN_0::steal() }),
            Flex::new(unsafe { PIN_1::steal() }),
            Flex::new(unsafe { PIN_2::steal() }),
            Flex::new(unsafe { PIN_3::steal() }),
            Flex::new(unsafe { PIN_4::steal() }),
            Flex::new(unsafe { PIN_5::steal() }),
            Flex::new(unsafe { PIN_6::steal() }),
            Flex::new(unsafe { PIN_7::steal() }),
        ]
    }

    fn led_pins(&self) -> [Flex<'static>; 1] {
        [Flex::new(unsafe { PIN_45::steal() })]
    }

    #[allow(dead_code)]
    fn sel_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_38::steal() }),
            Flex::new(unsafe { PIN_39::steal() }),
            Flex::new(unsafe { PIN_36::steal() }),
            Flex::new(unsafe { PIN_37::steal() }),
        ]
    }
}

#[cfg(feature = "fire-28-a")]
pub struct Fire28A {}

#[cfg(feature = "fire-28-a")]
impl Board for Fire28A {
    fn new() -> Self {
        Self {}
    }

    fn addr_pins(&self) -> [Flex<'static>; 16] {
        [
            Flex::new(unsafe { PIN_25::steal() }),
            Flex::new(unsafe { PIN_24::steal() }),
            Flex::new(unsafe { PIN_23::steal() }),
            Flex::new(unsafe { PIN_22::steal() }),
            Flex::new(unsafe { PIN_21::steal() }),
            Flex::new(unsafe { PIN_19::steal() }),
            Flex::new(unsafe { PIN_20::steal() }),
            Flex::new(unsafe { PIN_18::steal() }),
            Flex::new(unsafe { PIN_14::steal() }),
            Flex::new(unsafe { PIN_12::steal() }),
            Flex::new(unsafe { PIN_11::steal() }),
            Flex::new(unsafe { PIN_13::steal() }),
            Flex::new(unsafe { PIN_17::steal() }),
            Flex::new(unsafe { PIN_15::steal() }),
            Flex::new(unsafe { PIN_10::steal() }),
            Flex::new(unsafe { PIN_16::steal() }),
        ]
    }

    fn cs_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_8::steal() }),
            Flex::new(unsafe { PIN_9::steal() }),
        ]
    }

    #[allow(dead_code)]
    fn special_pins(&self) -> Vec<Flex<'static>> {
        vec![]
    }

    fn data_pins(&self) -> [Flex<'static>; 8] {
        [
            Flex::new(unsafe { PIN_7::steal() }),
            Flex::new(unsafe { PIN_6::steal() }),
            Flex::new(unsafe { PIN_5::steal() }),
            Flex::new(unsafe { PIN_0::steal() }),
            Flex::new(unsafe { PIN_1::steal() }),
            Flex::new(unsafe { PIN_2::steal() }),
            Flex::new(unsafe { PIN_3::steal() }),
            Flex::new(unsafe { PIN_4::steal() }),
        ]
    }

    fn led_pins(&self) -> [Flex<'static>; 1] {
        [Flex::new(unsafe { PIN_29::steal() })]
    }

    #[allow(dead_code)]
    fn sel_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_26::steal() }),
            Flex::new(unsafe { PIN_27::steal() }),
        ]
    }
}

#[cfg(feature = "fire-24-e")]
pub struct Fire24E {}

#[cfg(feature = "fire-24-e")]
impl Board for Fire24E {
    fn new() -> Self {
        Self {}
    }

    fn addr_pins(&self) -> [Flex<'static>; 12] {
        [
            Flex::new(unsafe { PIN_23::steal() }),
            Flex::new(unsafe { PIN_22::steal() }),
            Flex::new(unsafe { PIN_21::steal() }),
            Flex::new(unsafe { PIN_20::steal() }),
            Flex::new(unsafe { PIN_19::steal() }),
            Flex::new(unsafe { PIN_18::steal() }),
            Flex::new(unsafe { PIN_17::steal() }),
            Flex::new(unsafe { PIN_16::steal() }),
            Flex::new(unsafe { PIN_15::steal() }),
            Flex::new(unsafe { PIN_14::steal() }),
            Flex::new(unsafe { PIN_13::steal() }),
            //Flex::new(unsafe { PIN_11::steal() }),
            Flex::new(unsafe { PIN_12::steal() }),
        ]
    }

    fn cs_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_10::steal() }),
            Flex::new(unsafe { PIN_11::steal() }),
        ]
    }

    #[allow(dead_code)]
    fn special_pins(&self) -> Vec<Flex<'static>> {
        vec![
            // X1
            Flex::new(unsafe { PIN_9::steal() }),
            // X2
            Flex::new(unsafe { PIN_8::steal() }),
        ]
    }

    fn data_pins(&self) -> [Flex<'static>; 8] {
        [
            Flex::new(unsafe { PIN_7::steal() }),
            Flex::new(unsafe { PIN_6::steal() }),
            Flex::new(unsafe { PIN_5::steal() }),
            Flex::new(unsafe { PIN_0::steal() }),
            Flex::new(unsafe { PIN_1::steal() }),
            Flex::new(unsafe { PIN_2::steal() }),
            Flex::new(unsafe { PIN_3::steal() }),
            Flex::new(unsafe { PIN_4::steal() }),
        ]
    }

    fn led_pins(&self) -> [Flex<'static>; 1] {
        [Flex::new(unsafe { PIN_29::steal() })]
    }

    #[allow(dead_code)]
    fn sel_pins(&self) -> Vec<Flex<'static>> {
        vec![
            Flex::new(unsafe { PIN_25::steal() }),
            Flex::new(unsafe { PIN_24::steal() }),
            Flex::new(unsafe { PIN_26::steal() }),
            Flex::new(unsafe { PIN_27::steal() }),
        ]
    }
}
