use std::process::exit;
use crate::bus::Bus;
use crate::inst::INSTS;

#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub struct Register {
	pub value: u16
}

impl Register {
	pub const fn new(value: u16) -> Self {
		Self { value }
	}

	pub const fn get_low(&self) -> u8 {
		self.value as u8
	}

	pub const fn get_high(&self) -> u8 {
		(self.value >> 8) as u8
	}

	pub fn set_low(&mut self, value: u8) {
		self.value &= 0xFF00;
		self.value |= value as u16;
	}

	pub fn set_high(&mut self, value: u8) {
		self.value &= 0x00FF;
		self.value |= (value as u16) << 8;
	}
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub struct Flags {
	pub value: u16
}

impl Flags {
	pub const fn new(value: u16) -> Self {
		Self { value }
	}

	pub const fn get_carry(&self) -> bool {
		(self.value & 1) != 0
	}

	pub fn set_carry(&mut self, value: bool) {
		self.value &= !0b1;
		self.value |= value as u16;
	}

	pub const fn get_parity(&self) -> bool {
		(self.value & 1 << 2) != 0
	}

	pub fn set_parity(&mut self, value: bool) {
		self.value &= !(1 << 2);
		self.value |= (value as u16) << 2;
	}

	pub const fn get_aux_carry(&self) -> bool {
		(self.value & 1 << 4) != 0
	}

	pub fn set_aux_carry(&mut self, value: bool) {
		self.value &= !(1 << 4);
		self.value |= (value as u16) << 4;
	}

	pub const fn get_zero(&self) -> bool {
		(self.value & 1 << 6) != 0
	}

	pub fn set_zero(&mut self, value: bool) {
		self.value &= !(1 << 6);
		self.value |= (value as u16) << 6;
	}

	pub const fn get_sign(&self) -> bool {
		(self.value & 1 << 7) != 0
	}

	pub fn set_sign(&mut self, value: bool) {
		self.value &= !(1 << 7);
		self.value |= (value as u16) << 7;
	}

	pub const fn get_trap(&self) -> bool {
		(self.value & 1 << 8) != 0
	}

	pub fn set_trap(&mut self, value: bool) {
		self.value &= !(1 << 8);
		self.value |= (value as u16) << 8;
	}

	pub const fn get_int_enable(&self) -> bool {
		(self.value & 1 << 9) != 0
	}

	pub fn set_int_enable(&mut self, value: bool) {
		self.value &= !(1 << 9);
		self.value |= (value as u16) << 9;
	}

	pub const fn get_direction(&self) -> bool {
		(self.value & 1 << 10) != 0
	}

	pub fn set_direction(&mut self, value: bool) {
		self.value &= !(1 << 10);
		self.value |= (value as u16) << 10;
	}

	pub const fn get_overflow(&self) -> bool {
		(self.value & 1 << 11) != 0
	}

	pub fn set_overflow(&mut self, value: bool) {
		self.value &= !(1 << 11);
		self.value |= (value as u16) << 11;
	}
}

pub const AX: usize = 0;
pub const CX: usize = 1;
pub const DX: usize = 2;
pub const BX: usize = 3;
pub const SP: usize = 4;
pub const BP: usize = 5;
pub const SI: usize = 6;
pub const DI: usize = 7;

#[derive(Debug)]
pub struct Cpu {
	pub regs: [Register; 8],
	pub ip: u16,

	pub cs: u16,
	pub ds: u16,
	pub ss: u16,
	pub es: u16,
	pub flags: Flags,

	pub bus: Bus,
	pub op: u8,
	pub fetched_bytes: [u8; 5],
	pub nmi_enabled: bool,
	pub halted: bool
}

pub const fn get_addr(seg: u16, off: u16) -> u32 {
	(seg as u32) << 4 | off as u32
}

impl Cpu {
	pub fn new() -> Self {
		Self {
			regs: [Register::new(0); 8],
			ip: 0,
			cs: 0xFFFF,
			ds: 0,
			ss: 0,
			es: 0,
			flags: Flags::new(0),
			bus: Bus::new(),
			op: 0,
			fetched_bytes: [0; 5],
			nmi_enabled: false,
			halted: false
		}
	}

	pub fn read_at_ip(&mut self) -> u8 {
		let addr = get_addr(self.cs, self.ip);
		self.ip += 1;
		self.bus.read(addr)
	}

	pub fn port_write8(&mut self, port: u16, value: u8) {
		if port < 8 {
			if port % 2 == 0 {
				self.bus.dma.write_offset(port as u8 / 2, value);
			} else {
				self.bus.dma.write_word_count(port as u8 / 2, value);
			}
		} else if port == 8 {
			self.bus.dma.command.value = value;
		} else if port == 0xA {
			self.bus.dma.write_single_mask(value);
		} else if port == 0xB {
			self.bus.dma.set_mode(value);
		} else if port == 0xD {
			self.bus.dma.master_clear();
		} else if port == 0x41 {
			self.bus.pit.write_count(1, value);
		} else if port == 0x43 {
			self.bus.pit.write_control(value);
		} else if port == 0x61 {
			// todo cassette control
		} else if port == 0x63 {
			self.bus.ppi.control = value;
		} else if port >= 0x80 && port <= 0x83 {
			self.bus.dma.channels[(port - 0x80) as usize].page_base = value;
		} else if port == 0xA0 {
			self.nmi_enabled = value == 0x80;
		} else if port == 0x3B8 {
			self.bus.gpu.crt_out1 = value;
		} else if port == 0x3D8 {
			self.bus.gpu.mode = value;
		} else {
			eprintln!("port_write8: unimplemented write to {:#X}", port);
		}
	}

	pub fn port_read8(&mut self, port: u16) -> u8 {
		if port < 8 {
			if port % 2 == 0 {
				self.bus.dma.read_offset(port as u8 / 2)
			} else {
				self.bus.dma.read_word_count(port as u8 / 2)
			}
		} else if port == 8 {
			self.bus.dma.status.value
		} else if port == 0x41 {
			self.bus.pit.read_count(1)
		} else {
			eprintln!("port_read8: unimplemented read from {:#X}", port);
			0
		}
	}

	pub fn clock(&mut self) {
		self.bus.clock();

		if self.halted {
			return;
		}

		self.op = self.read_at_ip();
		let inst = INSTS[self.op as usize];
		if inst.1 == Cpu::nop && self.op != 0x90 {
			eprintln!("unimplemented op: {:#X}", self.op);
			exit(1);
		}
		// fetch
		inst.0(self);
		// execute
		inst.1(self);
	}
}
