#![feature(const_mut_refs)]

use crate::cpu::Cpu;

mod cpu;
mod inst;
mod bus;
mod dma;
mod gpu;
mod ppi;
mod pit;

fn main() {
    let mut cpu = Cpu::new();

    loop {
        cpu.clock();
    }
}
