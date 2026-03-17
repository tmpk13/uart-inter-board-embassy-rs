//! Controller board — reads buttons and sends toggle-LED command frames into the chain.
//!
//! Frame format (7 bytes): [SOF, TYPE, CMD, 0x00, hop_hi, hop_lo, EOF]
//!
//!   TYPE 0xCC = command frame
//!   CMD  0x01 = toggle LED
//!
//! hop_count starts at 0; each node increments it before forwarding.
//!
//! Wiring:
//!   PA2 (USART2 TX) ──► first node PA10 (USART1 RX)
//!   PA0 ── button A ── GND   (internal pull-up)
//!   PA1 ── button B ── GND   (internal pull-up)
//!   Common GND

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
});

const FRAME_LEN: usize = 7;
const SOF: u8 = 0xAA;
const EOF: u8 = 0x55;
const TYPE_CMD: u8 = 0xCC;
const CMD_TOGGLE: u8 = 0x01;
const DEBOUNCE: Duration = Duration::from_millis(50);

static OUTBOX: Channel<ThreadModeRawMutex, [u8; FRAME_LEN], 4> = Channel::new();

/// hop_count starts at 0; each node in the chain increments it.
fn make_frame() -> [u8; FRAME_LEN] {
    [SOF, TYPE_CMD, CMD_TOGGLE, 0x00, 0x00, 0x00, EOF]
}

#[embassy_executor::task]
async fn button_a(pin: embassy_stm32::Peri<'static, peripherals::PA0>) {
    let btn = Input::new(pin, Pull::Up);
    let mut count: u16 = 0;
    loop {
        while btn.is_high() {
            Timer::after_millis(10).await;
        }
        Timer::after(DEBOUNCE).await;
        if btn.is_low() {
            info!("Button A pressed #{}", count);
            OUTBOX.send(make_frame()).await;
            count = count.wrapping_add(1);
            while btn.is_low() {
                Timer::after_millis(10).await;
            }
            Timer::after(DEBOUNCE).await;
        }
    }
}

#[embassy_executor::task]
async fn button_b(pin: embassy_stm32::Peri<'static, peripherals::PA1>) {
    let btn = Input::new(pin, Pull::Up);
    let mut count: u16 = 0;
    loop {
        while btn.is_high() {
            Timer::after_millis(10).await;
        }
        Timer::after(DEBOUNCE).await;
        if btn.is_low() {
            info!("Button B pressed #{}", count);
            OUTBOX.send(make_frame()).await;
            count = count.wrapping_add(1);
            while btn.is_low() {
                Timer::after_millis(10).await;
            }
            Timer::after(DEBOUNCE).await;
        }
    }
}

#[embassy_executor::task]
async fn tx_drain(mut tx: UartTx<'static, Async>) {
    loop {
        let frame = OUTBOX.receive().await;
        tx.write(&frame).await.unwrap();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Controller started");

    let tx = UartTx::new(
        p.USART2,
        p.PA2,
        p.DMA1_CH7,
        Irqs,
        Config::default(),
    )
    .unwrap();

    spawner.spawn(button_a(p.PA0).unwrap());
    spawner.spawn(button_b(p.PA1).unwrap());
    spawner.spawn(tx_drain(tx).unwrap());
}
