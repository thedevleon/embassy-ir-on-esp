//! embassy hello world
//!
//! This is an example of running the embassy executor with multiple tasks
//! concurrently.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-time-timg0 embassy-executor-thread embassy-generic-timers

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, embassy, gpio::{Gpio6, Gpio7, Output, PushPull, IO}, peripherals::Peripherals, prelude::*, rmt::{asynch::RxChannelAsync, PulseCode, Rmt, RxChannelConfig, RxChannelCreator, TxChannelConfig, TxChannelCreator}, timer::TimerGroup
};

#[embassy_executor::task]
async fn run(mut rx_channel: esp_hal::rmt::Channel<2>) {
    loop {
        // what does this do?
        let mut data = [PulseCode {
            level1: true,
            length1: 1,
            level2: false,
            length2: 1,
        }; 48];

        let result = rx_channel.receive(&mut data).await;
        esp_println::println!("Received: {:?}", data);
    }
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);

    // IO
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let button = io.pins.gpio9.into_pull_up_input();

    // RMT
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    // IR RECV (GPIO6)
    let ir_recv = io.pins.gpio6;
    let rx_config = RxChannelConfig {
        clk_divider: 255,
        idle_threshold: 10000,
        ..RxChannelConfig::default()
    };
    let rx_channel = rmt.channel2.configure(ir_recv, rx_config).unwrap();

    // IR TX (GPIO7)
    let ir_tx_pin = io.pins.gpio7;
    let tx_config = TxChannelConfig {
        clk_divider: 255,
        ..TxChannelConfig::default()
    };
    let tx_channel = rmt.channel0.configure(ir_tx_pin, tx_config).unwrap();
    

    spawner.spawn(run(rx_channel)).ok();

    loop {
        esp_println::println!("Bing!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}