#[derive(Debug, PartialEq, Copy, Clone)]
pub enum LoadMode {
	Latch,
	Lsb,
	Msb,
	Both
}

#[derive(Debug, Copy, Clone)]
pub struct Counter {
	pub load_mode: LoadMode,
	pub mode: u8,
	pub bcd: bool,
	pub init_count: u16,
	pub value: u16,
	pub latch: Option<u16>,
	lsb: bool
}

impl Counter {
	const fn new() -> Self {
		Self { load_mode: LoadMode::Latch, mode: 0, bcd: false, init_count: 0, value: 0, latch: None, lsb: true }
	}
}

#[derive(Debug)]
pub struct Pit {
	pub counters: [Counter; 3]
}

impl Pit {
	pub const fn new() -> Self {
		Self { counters: [Counter::new(); 3] }
	}

	pub fn write_control(&mut self, control: u8) {
		let num = control >> 6;
		assert_ne!(num, 0b11);

		let load_mode = match control >> 4 & 0b11 {
			0b00 => LoadMode::Latch,
			0b01 => LoadMode::Lsb,
			0b10 => LoadMode::Msb,
			0b11 => LoadMode::Both,
			_ => unreachable!()
		};

		if load_mode == LoadMode::Latch {
			self.counters[num as usize].latch = Some(self.counters[num as usize].value);
			return;
		}

		let mode = match control >> 1 & 0b111 {
			0b000 => 0,
			0b001 => 1,
			0b010 | 0b110 => 2,
			0b011 | 0b111 => 3,
			0b100 => 4,
			0b101 => 5,
			_ => unreachable!()
		};

		let bcd = control & 1 > 0;

		self.counters[num as usize].load_mode = load_mode;
		self.counters[num as usize].lsb = load_mode == LoadMode::Lsb || load_mode == LoadMode::Both;
		self.counters[num as usize].mode = mode;
		self.counters[num as usize].bcd = bcd;
	}

	pub fn write_count(&mut self, counter: u8, value: u8) {
		let counter = &mut self.counters[counter as usize];
		if counter.lsb {
			counter.init_count = value as u16;
			if counter.load_mode == LoadMode::Both {
				counter.lsb = false;
			}
		} else {
			counter.init_count |= (value as u16) << 8;
			if counter.load_mode == LoadMode::Both {
				counter.lsb = true;
			}
		}
	}

	pub fn read_count(&mut self, counter: u8) -> u8 {
		let counter = &mut self.counters[counter as usize];
		if counter.lsb {
			let value = match counter.latch {
				Some(value) => value,
				None => counter.value
			};
			if counter.load_mode == LoadMode::Both {
				counter.lsb = false;
			}
			value as u8
		} else {
			let value = match counter.latch {
				Some(value) => value >> 8,
				None => counter.value >> 8
			};
			if counter.load_mode == LoadMode::Both {
				counter.lsb = true;
			}
			value as u8
		}
	}

	pub fn clock(&mut self) {
		for i in 0..3 {
			let counter = &mut self.counters[i];
			counter.value = counter.value.wrapping_sub(1);
			if counter.value == 0 {
				counter.value = counter.init_count;
			}
		}
	}
}
