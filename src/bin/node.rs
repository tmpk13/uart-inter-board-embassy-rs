//! Universal chain node — flash this to every board in the chain.
//!
//! Frame format (7 bytes): [SOF, TYPE, b2, b3, b4, b5, EOF]
//!
//!   TYPE 0x00 = data frame  — b2..b5 = big-endian u32 sequence number
//!   TYPE 0xCC = command frame — b2=CMD, b3=0x00, b4=hop_hi, b5=hop_lo
//!     CMD 0x01 = toggle LED (PC13, active-low)
//!
//! Each node that receives a CMD_TOGGLE frame:
//!   1. Toggles its built-in LED
//!   2. Increments the hop count
//!   3. Forwards the modified frame downstream
//!
//! Wiring (repeat per hop):
//!   board[N] PA2 (USART2 TX) ──► board[N+1] PA10 (USART1 RX)
//!   Common GND across all boards

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::dma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART1        => usart::InterruptHandler<peripherals::USART1>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
});

const FRAME_LEN: usize = 7;
const SOF: u8 = 0xAA;
const EOF: u8 = 0x55;
const TYPE_DATA: u8 = 0x00;
const TYPE_CMD: u8 = 0xCC;
const CMD_TOGGLE: u8 = 0x01;

static OUTBOX: Channel<ThreadModeRawMutex, [u8; FRAME_LEN], 4> = Channel::new();

/// Receives frames from the previous board, acts on commands, and forwards.
#[embassy_executor::task]
async fn rx_forward(mut rx: UartRx<'static, Async>, mut led: Output<'static>) {
    let mut buf = [0u8; FRAME_LEN];
    loop {
        match rx.read(&mut buf).await {
            Ok(()) => {
                if buf[0] != SOF || buf[FRAME_LEN - 1] != EOF {
                    warn!("Bad frame dropped: {:02X}", buf);
                    continue;
                }

                match (buf[1], buf[2]) {
                    (TYPE_CMD, CMD_TOGGLE) => {
                        let hop = u16::from_be_bytes([buf[4], buf[5]]);
                        info!("Toggle LED (hop #{})", hop);
                        led.toggle();
                        let next = hop.wrapping_add(1);
                        buf[4] = (next >> 8) as u8;
                        buf[5] = next as u8;
                        OUTBOX.send(buf).await;
                    }
                    (TYPE_DATA, _) => {
                        let n = u32::from_be_bytes([buf[2], buf[3], buf[4], buf[5]]);
                        info!("Forwarding data #{}", n);
                        OUTBOX.send(buf).await;
                    }
                    _ => warn!("Unknown frame: {:02X}", buf),
                }
            }
            Err(e) => warn!("RX error: {:?}", defmt::Debug2Format(&e)),
        }
    }
}

/// Originates a new numbered data frame every second.
#[embassy_executor::task]
async fn originate() {
    let mut n: u32 = 0;
    loop {
        let frame = [
            SOF,
            TYPE_DATA,
            (n >> 24) as u8,
            (n >> 16) as u8,
            (n >> 8) as u8,
            n as u8,
            EOF,
        ];
        OUTBOX.send(frame).await;
        info!("Originated #{}", n);
        n = n.wrapping_add(1);
        Timer::after_millis(1000).await;
    }
}

/// Drains the outbox onto USART2 TX — sole writer, no collisions.
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
    info!("Node started");

    // PC13 is the active-low built-in LED; start with it off (High).
    let led = Output::new(p.PC13, Level::High, Speed::Low);

    let rx = UartRx::new(
        p.USART1,
        p.PA10,
        p.DMA1_CH5,
        Irqs,
        Config::default(),
    )
    .unwrap();

    let tx = UartTx::new(
        p.USART2,
        p.PA2,
        p.DMA1_CH7,
        Irqs,
        Config::default(),
    )
    .unwrap();

    spawner.spawn(rx_forward(rx, led).unwrap());
    spawner.spawn(originate().unwrap());
    spawner.spawn(tx_drain(tx).unwrap());
}
