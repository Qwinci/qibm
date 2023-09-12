use crate::bus::Bus;

#[repr(transparent)]
#[derive(Debug)]
pub struct Status {
	pub value: u8
}

impl Status {
	pub fn reached_tc(&self, channel: u8) -> bool {
		self.value & 1 << channel > 0
	}

	pub fn set_reached_tc(&mut self, channel: u8, reached: bool) {
		if reached {
			self.value |= 1 << channel;
		} else {
			self.value &= !(1 << channel);
		}
	}

	pub fn channel_requested(&self, channel: u8) -> bool {
		self.value & 1 << (4 + channel) > 0
	}

	pub fn set_channel_requested(&mut self, channel: u8, request: bool) {
		if request {
			self.value |= 1 << (4 + channel);
		} else {
			self.value &= !(1 << (4 + channel));
		}
	}
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum TransferMode {
	Verify,
	Write,
	Read
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Mode {
	Demand,
	Single,
	Block,
	Cascade
}

#[derive(Debug, Copy, Clone)]
pub struct Channel {
	pub transfer_mode: TransferMode,
	pub mode: Mode,
	pub base_offset: u16,
	pub base_word_count: u16,
	pub offset: u16,
	pub word_count: u16,
	// page base (in 64k pages for 8bit or 128k for 16bit transactions)
	pub page_base: u8,
	pub mask: bool,
	pub auto_init: bool,
	pub addr_inc: bool,

	offset_latch: bool,
	word_latch: bool
}

impl Channel {
	const fn new() -> Self {
		Self {
			transfer_mode: TransferMode::Read,
			mode: Mode::Single,
			base_offset: 0,
			base_word_count: 0,
			offset: 0,
			word_count: 0,
			page_base: 0,
			mask: true,
			auto_init: false,
			addr_inc: true,
			offset_latch: true,
			word_latch: true
		}
	}
}

#[repr(transparent)]
#[derive(Debug)]
pub struct Command {
	pub value: u8
}

impl Command {
	fn mem_to_mem_enable(&self) -> bool {
		self.value & 1 > 0
	}

	fn ch0_addr_hold_enable(&self) -> bool {
		self.value & 1 << 1 > 0
	}

	fn controller_enable(&self) -> bool {
		self.value & 1 << 2 == 0
	}

	fn compressed_timing(&self) -> bool {
		self.value & 1 << 3 > 0
	}

	fn rotate_priority(&self) -> bool {
		self.value & 1 << 4 > 0
	}

	fn extended_write_selection(&self) -> bool {
		self.value & 1 << 5 > 0
	}

	fn dreq_active_low(&self) -> bool {
		self.value & 1 << 6 > 0
	}

	fn dack_active_high(&self) -> bool {
		self.value & 1 << 7 > 0
	}
}

#[derive(Debug)]
pub struct Dma {
	pub channels: [Channel; 4],
	pub status: Status,
	pub command: Command
}

impl Dma {
	pub const fn new() -> Self {
		Self {
			channels: [Channel::new(); 4],
			status: Status { value: 0 },
			command: Command { value: 0 }
		}
	}

	pub fn master_clear(&mut self) {
		self.status.value = 0;
		for channel in self.channels.iter_mut() {
			channel.offset_latch = true;
			channel.word_latch = true;
			channel.mask = true;
		}
	}

	pub fn write_offset(&mut self, channel: u8, value: u8) {
		let channel = &mut self.channels[channel as usize];
		if channel.offset_latch {
			channel.base_offset = value as u16;
			channel.offset = value as u16;
			channel.offset_latch = false;
		} else {
			channel.base_offset |= (value as u16) << 8;
			channel.offset |= (value as u16) << 8;
			channel.offset_latch = true;
		}
	}

	pub fn read_offset(&mut self, channel: u8) -> u8 {
		let channel = &mut self.channels[channel as usize];
		if channel.offset_latch {
			channel.offset_latch = false;
			channel.offset as u8
		} else {
			channel.offset_latch = true;
			(channel.offset >> 8) as u8
		}
	}

	pub fn write_word_count(&mut self, channel: u8, value: u8) {
		let channel = &mut self.channels[channel as usize];
		if channel.word_latch {
			channel.base_word_count = value as u16;
			channel.word_count = value as u16;
			channel.word_latch = false;
		} else {
			channel.base_word_count |= (value as u16) << 8;
			channel.word_count |= (value as u16) << 8;
			channel.word_latch = true;
		}
	}

	pub fn read_word_count(&mut self, channel: u8) -> u8 {
		let channel = &mut self.channels[channel as usize];
		if channel.word_latch {
			channel.word_latch = false;
			channel.word_count as u8
		} else {
			channel.word_latch = true;
			(channel.word_count >> 8) as u8
		}
	}

	pub fn set_mode(&mut self, value: u8) {
		let channel = value & 0b11;
		let channel = &mut self.channels[channel as usize];

		channel.transfer_mode = match value >> 2 & 0b11 {
			0b00 => TransferMode::Verify,
			0b01 => TransferMode::Write,
			0b10 => TransferMode::Read,
			_ => unreachable!()
		};
		channel.auto_init = value & 1 << 4 > 0;
		channel.addr_inc = value & 1 << 5 == 0;
		channel.mode = match value >> 6 {
			0b00 => Mode::Demand,
			0b01 => Mode::Single,
			0b10 => Mode::Block,
			0b11 => Mode::Cascade,
			_ => unreachable!()
		};
	}

	pub fn write_single_mask(&mut self, value: u8) {
		let channel = value & 0b11;
		let channel = &mut self.channels[channel as usize];
		channel.mask = value & 1 << 2 > 0;
	}

	pub fn clock(&mut self) {
		// todo
		return;

		if !self.command.controller_enable() {
			return;
		}

		// transfer ~1byte per 3-4 clocks, so this is called for cpu_hz / 4 clocks

		for (id, channel) in self.channels.iter_mut().enumerate() {
			if channel.mask {
				continue;
			}

			channel.word_count = channel.word_count.wrapping_sub(1);
			if channel.addr_inc {
				channel.offset = channel.offset.wrapping_add(1);
			} else {
				channel.offset = channel.offset.wrapping_sub(1);
			}
			if channel.word_count == 0xFFFF {
				self.status.set_channel_requested(id as u8, false);
				if channel.auto_init {
					channel.offset = channel.base_offset;
					channel.word_count = channel.base_word_count;
				} else {
					self.status.set_reached_tc(id as u8, true);
					channel.mask = true;
				}
			}
		}
	}
}
