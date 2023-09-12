use crate::cpu::{AX, BP, Cpu, CX, DX, get_addr, SP};

#[repr(u8)]
pub enum Mode {
	Mem = 0b00,
	Mem8Disp = 0b01,
	Mem16Disp = 0b10,
	Reg = 0b11
}

#[derive(Clone, Copy)]
pub struct ModRm(u8);

const RM_DISP16: u8 = 0b110;

impl ModRm {
	pub const fn get_mode(&self) -> Mode {
		match self.0 >> 6 {
			0b00 => Mode::Mem,
			0b01 => Mode::Mem8Disp,
			0b10 => Mode::Mem16Disp,
			0b11 => Mode::Reg,
			_ => unreachable!()
		}
	}

	pub const fn get_reg(&self) -> usize {
		(self.0 >> 3 & 0b111) as usize
	}

	pub const fn get_rm(&self) -> u8 {
		self.0 & 0b111
	}
}

enum Ea {
	Reg(usize),
	Mem(u32)
}

impl Cpu {
	fn fetch_0(&mut self) {

	}
	fn fetch_1(&mut self) {
		self.fetched_bytes[0] = self.read_at_ip();
	}
	fn fetch_2(&mut self) {
		self.fetched_bytes[0] = self.read_at_ip();
		self.fetched_bytes[1] = self.read_at_ip();
	}
	fn fetch_3(&mut self) {
		self.fetched_bytes[0] = self.read_at_ip();
		self.fetched_bytes[1] = self.read_at_ip();
		self.fetched_bytes[2] = self.read_at_ip();
	}
	fn fetch_4(&mut self) {
		self.fetched_bytes[0] = self.read_at_ip();
		self.fetched_bytes[1] = self.read_at_ip();
		self.fetched_bytes[2] = self.read_at_ip();
		self.fetched_bytes[3] = self.read_at_ip();
	}
	fn fetch_5(&mut self) {
		self.fetched_bytes[0] = self.read_at_ip();
		self.fetched_bytes[1] = self.read_at_ip();
		self.fetched_bytes[2] = self.read_at_ip();
		self.fetched_bytes[3] = self.read_at_ip();
		self.fetched_bytes[4] = self.read_at_ip();
	}

	pub fn nop(&mut self) -> u8 {
		0
	}

	fn jmp_far(&mut self) -> u8 {
		let ip_low = self.fetched_bytes[0];
		let ip_high = self.fetched_bytes[1];
		let cs_low = self.fetched_bytes[2];
		let cs_high = self.fetched_bytes[3];
		self.ip = ip_low as u16 | (ip_high as u16) << 8;
		self.cs = cs_low as u16 | (cs_high as u16) << 8;
		0
	}

	fn cli(&mut self) -> u8 {
		self.flags.set_int_enable(false);
		0
	}

	fn mov_reg_low_imm8(&mut self) -> u8 {
		let reg = self.op & 0x3;
		self.regs[reg as usize].set_low(self.fetched_bytes[0]);
		0
	}

	fn mov_reg_high_imm8(&mut self) -> u8 {
		let reg = self.op & 0x3;
		self.regs[reg as usize].set_high(self.fetched_bytes[0]);
		0
	}

	fn mov_reg_imm16(&mut self) -> u8 {
		let reg = self.op & 7;
		let value = self.fetched_bytes[0] as u16 | (self.fetched_bytes[1] as u16) << 8;
		self.regs[reg as usize].value = value;
		0
	}

	fn sahf(&mut self) -> u8 {
		let old = self.flags.value & 0b1111111100101010;
		self.flags.value = self.regs[AX].get_high() as u16;
		self.flags.value |= old;
		0
	}

	fn lahf(&mut self) -> u8 {
		self.regs[AX].set_high(self.flags.value as u8 & !(1 << 1 | 1 << 3 | 1 << 5));
		0
	}

	fn jo(&mut self) -> u8 {
		if self.flags.get_overflow() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jno(&mut self) -> u8 {
		if !self.flags.get_overflow() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jc(&mut self) -> u8 {
		if self.flags.get_carry() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jnc(&mut self) -> u8 {
		if !self.flags.get_carry() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jz(&mut self) -> u8 {
		if self.flags.get_zero() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jnz(&mut self) -> u8 {
		if !self.flags.get_zero() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jna(&mut self) -> u8 {
		if self.flags.get_carry() || self.flags.get_zero() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn js(&mut self) -> u8 {
		if self.flags.get_sign() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jns(&mut self) -> u8 {
		if !self.flags.get_sign() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jp(&mut self) -> u8 {
		if self.flags.get_parity() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn jnp(&mut self) -> u8 {
		if !self.flags.get_parity() {
			let inc = self.fetched_bytes[0] as i8;
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn calc_ea(&mut self, modrm: ModRm) -> Ea {
		match modrm.get_mode() {
			Mode::Mem => {
				let rm = modrm.get_rm();
				let addr = if rm == RM_DISP16 {
					let low = self.read_at_ip();
					let high = self.read_at_ip();
					low as u32 | (high as u32) << 8
				} else {
					if rm == SP as u8 || rm == BP as u8 {
						get_addr(self.ss, self.regs[rm as usize].value)
					}
					else {
						get_addr(self.ds, self.regs[rm as usize].value)
					}
				};
				Ea::Mem(addr)
			}
			Mode::Mem8Disp => {
				let rm = modrm.get_rm();
				let disp = self.read_at_ip();
				let addr = if rm == SP as u8 || rm == BP as u8 {
					get_addr(self.ss, self.regs[rm as usize].value)
				} else {
					get_addr(self.ds, self.regs[rm as usize].value)
				} + disp as u32;
				Ea::Mem(addr)
			}
			Mode::Mem16Disp => {
				let rm = modrm.get_rm();
				let disp_low = self.read_at_ip();
				let disp_high = self.read_at_ip();
				let disp = disp_low as u32 | (disp_high as u32) << 8;
				let addr = if rm == SP as u8 || rm == BP as u8 {
					get_addr(self.ss, self.regs[rm as usize].value)
				} else {
					get_addr(self.ds, self.regs[rm as usize].value)
				} + disp;
				Ea::Mem(addr)
			}
			Mode::Reg => {
				Ea::Reg(modrm.get_rm() as usize)
			}
		}
	}

	fn fetch8_from_ea(&mut self, ea: &Ea) -> u8 {
		match ea {
			Ea::Reg(num) => {
				let whole_reg = num & 3;
				let high = num >> 2 > 0;

				let reg = &mut self.regs[whole_reg];
				if high {
					reg.get_high()
				} else {
					reg.get_low()
				}
			}
			Ea::Mem(addr) => {
				self.bus.read(*addr)
			}
		}
	}

	fn fetch16_from_ea(&mut self, ea: &Ea) -> u16 {
		match ea {
			Ea::Reg(num) => {
				self.regs[*num].value
			}
			Ea::Mem(addr) => {
				let low = self.bus.read(*addr);
				let high = self.bus.read(*addr + 1);
				low as u16 | (high as u16) << 8
			}
		}
	}

	fn write8_to_ea(&mut self, ea: &Ea, value: u8) {
		match ea {
			Ea::Reg(num) => {
				let whole_reg = num & 3;
				let high = num >> 2 > 0;

				let reg = &mut self.regs[whole_reg];
				if high {
					reg.set_high(value)
				} else {
					reg.set_low(value)
				}
			}
			Ea::Mem(addr) => {
				self.bus.write(*addr, value);
			}
		}
	}

	fn write16_to_ea(&mut self, ea: &Ea, value: u16) {
		match ea {
			Ea::Reg(num) => {
				self.regs[*num].value = value;
			}
			Ea::Mem(addr) => {
				self.bus.write(*addr, value as u8);
				self.bus.write(*addr + 1, (value >> 8) as u8);
			}
		}
	}

	fn dx(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let op = modrm.get_reg();
		let word = self.op & 1 > 0;

		let amount = if self.op & 0xF < 2 { 1 } else { self.regs[CX].get_low() as u32 };

		let ea = self.calc_ea(modrm);
		if !word {
			let orig = self.fetch8_from_ea(&ea);
			let value;
			match op {
				// rol
				0b000 => {
					value = orig.rotate_left(amount);
					// todo this might not be correct for amount 0
					self.flags.set_carry(value & 1 > 0);
					self.flags.set_overflow(value & 1 << 7 != orig & 1 << 7);
				}
				// ror
				0b001 => {
					value = orig.rotate_right(amount);
					// todo this might not be correct for amount 0
					self.flags.set_carry(value & 1 << 7 > 0);
					self.flags.set_overflow(value & 1 << 7 != orig & 1 << 7);
				}
				// rcl
				0b010 => {
					let mut tmp = orig;
					let mut carry = self.flags.get_carry() as u8;
					for _ in 0..amount {
						let new_carry = tmp >> 7;
						tmp = tmp << 1 | carry;
						carry = new_carry;
					}
					value = tmp;
					self.flags.set_carry(carry > 0);
					self.flags.set_overflow(value ^ orig & 0x80 > 0);
				}
				// rcr
				0b011 => {
					let mut tmp = orig;
					let mut carry = self.flags.get_carry() as u8;
					for _ in 0..amount {
						let new_carry = tmp & 1;
						tmp = tmp >> 1 | carry << 7;
						carry = new_carry;
					}
					value = tmp;
					self.flags.set_carry(carry > 0);
					self.flags.set_overflow(value ^ orig & 0x80 > 0);
				}
				// shl
				0b100 => {
					value = orig << amount;
					self.flags.set_carry(orig << (amount - 1) & 1 > 0);
					self.flags.set_sign(false);
					self.flags.set_zero(value == 0);
					self.flags.set_parity(value.count_ones() % 2 == 0);
					self.flags.set_overflow(value & 1 << 7 != orig & 1 << 7);
				}
				// shr
				0b101 => {
					value = orig >> amount;
					self.flags.set_carry(orig >> (amount - 1) & 1 > 0);
					self.flags.set_sign(false);
					self.flags.set_zero(value == 0);
					self.flags.set_parity(value.count_ones() % 2 == 0);
					self.flags.set_overflow(value & 1 << 7 != orig & 1 << 7);
				}
				// sar
				0b111 => {
					value = orig >> amount | (orig & 1 << 7);
					self.flags.set_carry(orig >> (amount - 1) & 1 > 0);
					self.flags.set_zero(value == 0);
					self.flags.set_parity(value.count_ones() % 2 == 0);
					self.flags.set_overflow(false);
				}
				num => todo!("D2 subop {:#b}", num)
			}

			self.write8_to_ea(&ea, value);
		} else {
			todo!();
		}

		0
	}

	fn fetch8_from_reg(&self, modrm: ModRm) -> u8 {
		let num = modrm.get_reg();
		let whole_reg = num & 3;
		let high = num >> 2 > 0;

		let reg = &self.regs[whole_reg];
		if high {
			reg.get_high()
		} else {
			reg.get_low()
		}
	}

	fn fetch16_from_reg(&self, modrm: ModRm) -> u16 {
		self.regs[modrm.get_reg()].value
	}

	fn write8_to_reg(&mut self, modrm: ModRm, value: u8) {
		let num = modrm.get_reg();
		let whole_reg = num & 3;
		let high = num >> 2 > 0;

		let reg = &mut self.regs[whole_reg];
		if high {
			reg.set_high(value);
		} else {
			reg.set_low(value);
		}
	}

	fn write16_to_reg(&mut self, modrm: ModRm, value: u16) {
		self.regs[modrm.get_reg()].value = value;
	}

	fn xor(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let word = self.op & 1 > 0;
		let to_reg = self.op & 1 << 1 > 0;

		let ea = self.calc_ea(modrm);

		if !word {
			let result = if !to_reg {
				let value = self.fetch8_from_ea(&ea);
				let value2 = self.fetch8_from_reg(modrm);
				let result = value ^ value2;
				self.write8_to_ea(&ea, result);
				result
			} else {
				let value = self.fetch8_from_reg(modrm);
				let value2 = self.fetch8_from_ea(&ea);
				let result = value ^ value2;
				self.write8_to_reg(modrm, result);
				result
			};

			self.flags.set_overflow(false);
			self.flags.set_carry(false);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
		} else {
			let result = if !to_reg {
				let value = self.fetch16_from_ea(&ea);
				let value2 = self.fetch16_from_reg(modrm);
				let result = value ^ value2;
				self.write16_to_ea(&ea, result);
				result
			} else {
				let value = self.fetch16_from_reg(modrm);
				let value2 = self.fetch16_from_ea(&ea);
				let result = value ^ value2;
				self.write16_to_reg(modrm, result);
				result
			};

			self.flags.set_overflow(false);
			self.flags.set_carry(false);
			self.flags.set_sign(result & 1 << 15 > 0);
			self.flags.set_zero(result == 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
		}

		0
	}

	fn stc(&mut self) -> u8 {
		self.flags.set_carry(true);
		0
	}

	fn fetch_from_sr(&self, modrm: ModRm) -> u16 {
		let reg = modrm.get_reg();
		match reg & 0b11 {
			0b00 => self.es,
			0b01 => self.cs,
			0b10 => self.ss,
			0b11 => self.ds,
			_ => unreachable!()
		}
	}

	fn write_to_sr(&mut self, modrm: ModRm, value: u16) {
		let reg = modrm.get_reg();
		match reg & 0b11 {
			0b00 => self.es = value,
			0b01 => self.cs = value,
			0b10 => self.ss = value,
			0b11 => self.ds = value,
			_ => unreachable!()
		}
	}

	fn mov_rm_sr(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let ea = self.calc_ea(modrm);
		self.write16_to_ea(&ea, self.fetch_from_sr(modrm));
		0
	}

	fn mov_sr(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let ea = self.calc_ea(modrm);
		let value = self.fetch16_from_ea(&ea);
		self.write_to_sr(modrm, value);

		0
	}

	fn mov_rm_r16(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let ea = self.calc_ea(modrm);
		self.write16_to_ea(&ea, self.fetch16_from_reg(modrm));
		0
	}

	fn mov_r16_rm(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let ea = self.calc_ea(modrm);
		let value = self.fetch16_from_ea(&ea);
		self.write16_to_reg(modrm, value);
		0
	}

	fn mov_rm_r8(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let ea = self.calc_ea(modrm);
		self.write8_to_ea(&ea, self.fetch8_from_reg(modrm));
		0
	}

	fn mov_r8_rm(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let ea = self.calc_ea(modrm);
		let value = self.fetch8_from_ea(&ea);
		self.write8_to_reg(modrm, value);
		0
	}

	fn clc(&mut self) -> u8 {
		self.flags.set_carry(false);
		0
	}

	fn jmp(&mut self) -> u8 {
		let inc = self.fetched_bytes[0] as i8;
		self.ip = self.ip.wrapping_add_signed(inc as i16);
		0
	}

	fn or(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let word = self.op & 1 > 0;
		let to_reg = self.op & 1 << 1 > 0;

		let ea = self.calc_ea(modrm);

		if !word {
			let result = if !to_reg {
				let value = self.fetch8_from_ea(&ea);
				let value2 = self.fetch8_from_reg(modrm);
				let result = value | value2;
				self.write8_to_ea(&ea, result);
				result
			} else {
				let value = self.fetch8_from_reg(modrm);
				let value2 = self.fetch8_from_ea(&ea);
				let result = value | value2;
				self.write8_to_reg(modrm, result);
				result
			};

			self.flags.set_overflow(false);
			self.flags.set_carry(false);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
		} else {
			let result = if !to_reg {
				let value = self.fetch16_from_ea(&ea);
				let value2 = self.fetch16_from_reg(modrm);
				let result = value | value2;
				self.write16_to_ea(&ea, result);
				result
			} else {
				let value = self.fetch16_from_reg(modrm);
				let value2 = self.fetch16_from_ea(&ea);
				let result = value | value2;
				self.write16_to_reg(modrm, result);
				result
			};

			self.flags.set_overflow(false);
			self.flags.set_carry(false);
			self.flags.set_sign(result & 1 << 15 > 0);
			self.flags.set_zero(result == 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
		}

		0
	}

	fn out_imm8_al(&mut self) -> u8 {
		self.port_write8(self.fetched_bytes[0] as u16, self.regs[AX].get_low());
		0
	}

	fn out_dx_al(&mut self) -> u8 {
		self.port_write8(self.regs[DX].value, self.regs[AX].get_low());
		0
	}

	fn in_al_imm8(&mut self) -> u8 {
		let value = self.port_read8(self.fetched_bytes[0] as u16);
		self.regs[AX].set_low(value);
		0
	}

	fn in_al_dx(&mut self) -> u8 {
		let value = self.port_read8(self.regs[DX].value);
		self.regs[AX].set_low(value);
		0
	}

	fn inc_dec(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let wide = self.op & 1 > 0;
		let op = modrm.get_reg();
		let ea = self.calc_ea(modrm);

		if !wide {
			let orig = self.fetch8_from_ea(&ea);
			let value;
			match op {
				// inc
				0b000 => {
					value = orig.wrapping_add(1);
					self.flags.set_aux_carry((orig & 0xF).wrapping_add(1) > 0xF);
					let same_bits = !(orig ^ 1);
					self.flags.set_overflow(same_bits & (value ^ orig) & 0x80 > 0);
					self.flags.set_parity(value.count_ones() % 2 == 0);
					self.flags.set_sign(value & 1 << 7 > 0);
					self.flags.set_zero(value == 0);
				}
				// dec
				0b001 => {
					value = orig.wrapping_sub(1);
					self.flags.set_aux_carry((orig & 0xF).wrapping_sub(1) > 0xF);
					let same_bits = !(orig ^ (-1i8 as u8));
					self.flags.set_overflow(same_bits & (value ^ orig) & 0x80 > 0);
					self.flags.set_parity(value.count_ones() % 2 == 0);
					self.flags.set_sign(value & 1 << 7 > 0);
					self.flags.set_zero(value == 0);
				}
				_ => unreachable!()
			}
			self.write8_to_ea(&ea, value);
		} else {
			unreachable!()
		}
		0
	}

	fn inc_reg16(&mut self) -> u8 {
		let reg = self.op & 0xF;
		let orig = self.regs[reg as usize].value;
		let value = orig.wrapping_add(1);
		self.regs[reg as usize].value = value;

		self.flags.set_aux_carry((orig & 0xF).wrapping_add(1) > 0xF);
		let same_bits = !(orig ^ 1);
		self.flags.set_overflow(same_bits & (value ^ orig) & 0x80 > 0);
		self.flags.set_parity(value.count_ones() % 2 == 0);
		self.flags.set_sign(value & 1 << 7 > 0);
		self.flags.set_zero(value == 0);

		0
	}

	fn jmp_near(&mut self) -> u8 {
		let addr = self.fetched_bytes[0] as u16 | (self.fetched_bytes[1] as u16) << 8;
		self.ip = self.ip.wrapping_add_signed(addr as i16);
		0
	}

	fn add(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let word = self.op & 1 > 0;
		let to_reg = self.op & 1 << 1 > 0;

		let ea = self.calc_ea(modrm);

		if !word {
			let orig;
			let addend;
			let (result, carry) = if !to_reg {
				orig = self.fetch8_from_ea(&ea);
				addend = self.fetch8_from_reg(modrm);
				let result = orig.overflowing_add(addend);
				self.write8_to_ea(&ea, result.0);
				result
			} else {
				orig = self.fetch8_from_reg(modrm);
				addend = self.fetch8_from_ea(&ea);
				let result = orig.overflowing_add(addend);
				self.write8_to_reg(modrm, result.0);
				result
			};

			self.flags.set_carry(carry);
			self.flags.set_aux_carry((orig & 0xF).wrapping_add(addend & 0xF) > 0xF);
			let same_bits = !(orig ^ addend);
			self.flags.set_overflow(same_bits & (result ^ orig) & 0x80 > 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
		} else {
			todo!();
		}

		0
	}

	fn loop_short(&mut self) -> u8 {
		let inc = self.fetched_bytes[0] as i8;
		self.regs[CX].value = self.regs[CX].value.wrapping_sub(1);
		if self.regs[CX].value != 0 {
			self.ip = self.ip.wrapping_add_signed(inc as i16);
		}
		0
	}

	fn ret(&mut self) -> u8 {
		let addr = get_addr(self.ss, self.regs[SP].value);
		let low = self.bus.read(addr);
		let high = self.bus.read(addr + 1);
		self.regs[SP].value += 2;
		self.ip = low as u16 | (high as u16) << 8;
		0
	}

	fn x80(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let op = modrm.get_reg();
		let word = self.op & 1 > 0;

		let ea = self.calc_ea(modrm);
		if !word {
			let orig = self.fetch8_from_ea(&ea);
			let value2 = self.read_at_ip();
			match op {
				// cmp
				0b111 => {
					let (result, carry) = orig.overflowing_sub(value2);
					self.flags.set_aux_carry((orig & 0xF).wrapping_sub(value2 & 0xF) > 0xF);
					self.flags.set_carry(carry);
					let same_bits = !(orig ^ value2);
					self.flags.set_overflow(same_bits & (result ^ orig) & 0x80 > 0);
					self.flags.set_parity(result.count_ones() % 2 == 0);
					self.flags.set_sign(result & 1 << 7 > 0);
					self.flags.set_zero(result == 0);
				}
				num => todo!("0x80 subop {:#b}", num)
			}
		} else {
			todo!();
		}

		0
	}

	fn hlt(&mut self) -> u8 {
		self.halted = true;
		eprintln!("cpu halted");
		0
	}

	fn sub(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let word = self.op & 1 > 0;
		let to_reg = self.op & 1 << 1 > 0;

		let ea = self.calc_ea(modrm);

		if !word {
			let orig;
			let second;
			let (result, carry) = if !to_reg {
				orig = self.fetch8_from_ea(&ea);
				second = self.fetch8_from_reg(modrm);
				let result = orig.overflowing_sub(second);
				self.write8_to_ea(&ea, result.0);
				result
			} else {
				orig = self.fetch8_from_reg(modrm);
				second = self.fetch8_from_ea(&ea);
				let result = orig.overflowing_sub(second);
				self.write8_to_reg(modrm, result.0);
				result
			};

			self.flags.set_carry(carry);
			self.flags.set_aux_carry((orig & 0xF).wrapping_sub(second & 0xF) > 0xF);
			let same_bits = !(orig ^ second);
			self.flags.set_overflow(same_bits & (result ^ orig) & 0x80 > 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
		} else {
			let orig;
			let second;
			let (result, carry) = if !to_reg {
				orig = self.fetch16_from_ea(&ea);
				second = self.fetch16_from_reg(modrm);
				let result = orig.overflowing_sub(second);
				self.write16_to_ea(&ea, result.0);
				result
			} else {
				orig = self.fetch16_from_reg(modrm);
				second = self.fetch16_from_ea(&ea);
				let result = orig.overflowing_sub(second);
				self.write16_to_reg(modrm, result.0);
				result
			};

			self.flags.set_carry(carry);
			self.flags.set_aux_carry((orig & 0xF).wrapping_sub(second & 0xF) > 0xF);
			let same_bits = !(orig ^ second);
			self.flags.set_overflow(same_bits & (result ^ orig) & 0x80 > 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
		}

		0
	}

	fn and(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let word = self.op & 1 > 0;
		let to_reg = self.op & 1 << 1 > 0;

		let ea = self.calc_ea(modrm);

		if !word {
			let result = if !to_reg {
				let value = self.fetch8_from_ea(&ea);
				let value2 = self.fetch8_from_reg(modrm);
				let result = value & value2;
				self.write8_to_ea(&ea, result);
				result
			} else {
				let value = self.fetch8_from_reg(modrm);
				let value2 = self.fetch8_from_ea(&ea);
				let result = value & value2;
				self.write8_to_reg(modrm, result);
				result
			};

			self.flags.set_overflow(false);
			self.flags.set_carry(false);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
		} else {
			let result = if !to_reg {
				let value = self.fetch16_from_ea(&ea);
				let value2 = self.fetch16_from_reg(modrm);
				let result = value & value2;
				self.write16_to_ea(&ea, result);
				result
			} else {
				let value = self.fetch16_from_reg(modrm);
				let value2 = self.fetch16_from_ea(&ea);
				let result = value & value2;
				self.write16_to_reg(modrm, result);
				result
			};

			self.flags.set_overflow(false);
			self.flags.set_carry(false);
			self.flags.set_sign(result & 1 << 15 > 0);
			self.flags.set_zero(result == 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
		}

		0
	}

	fn push_r16(&mut self) -> u8 {
		let reg = self.op & 7;
		let value = self.regs[reg as usize].value;
		let sp = &mut self.regs[SP];
		sp.value = sp.value.wrapping_sub(2);
		self.bus.write(get_addr(self.ss, sp.value), value as u8);
		self.bus.write(get_addr(self.ss, sp.value.wrapping_add(1)), (value >> 8) as u8);
		0
	}

	fn pop_r16(&mut self) -> u8 {
		let reg = self.op & 7;
		let sp = &mut self.regs[SP];
		let low = self.bus.read(get_addr(self.ss, sp.value));
		let high = self.bus.read(get_addr(self.ss, sp.value.wrapping_add(1)));
		sp.value = sp.value.wrapping_add(2);
		let dest = &mut self.regs[reg as usize];
		dest.value = low as u16 | (high as u16) << 8;
		0
	}

	fn cmp(&mut self) -> u8 {
		let modrm = ModRm(self.fetched_bytes[0]);
		let word = self.op & 1 > 0;
		let to_reg = self.op & 1 << 1 > 0;

		let ea = self.calc_ea(modrm);

		if !word {
			let orig;
			let second;
			let (result, carry) = if !to_reg {
				orig = self.fetch8_from_ea(&ea);
				second = self.fetch8_from_reg(modrm);
				let result = orig.overflowing_sub(second);
				result
			} else {
				orig = self.fetch8_from_reg(modrm);
				second = self.fetch8_from_ea(&ea);
				let result = orig.overflowing_sub(second);
				result
			};

			self.flags.set_carry(carry);
			self.flags.set_aux_carry((orig & 0xF).wrapping_sub(second & 0xF) > 0xF);
			let same_bits = !(orig ^ second);
			self.flags.set_overflow(same_bits & (result ^ orig) & 0x80 > 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
		} else {
			let orig;
			let second;
			let (result, carry) = if !to_reg {
				orig = self.fetch16_from_ea(&ea);
				second = self.fetch16_from_reg(modrm);
				let result = orig.overflowing_sub(second);
				result
			} else {
				orig = self.fetch16_from_reg(modrm);
				second = self.fetch16_from_ea(&ea);
				let result = orig.overflowing_sub(second);
				result
			};

			self.flags.set_carry(carry);
			self.flags.set_aux_carry((orig & 0xF).wrapping_sub(second & 0xF) > 0xF);
			let same_bits = !(orig ^ second);
			self.flags.set_overflow(same_bits & (result ^ orig) & 0x80 > 0);
			self.flags.set_parity(result.count_ones() % 2 == 0);
			self.flags.set_sign(result & 1 << 7 > 0);
			self.flags.set_zero(result == 0);
		}

		0
	}
}

const fn generate_insts() -> [(fn(&mut Cpu), fn(&mut Cpu) -> u8); 0x100] {
	let mut arr = [(Cpu::fetch_0 as fn(&mut Cpu), Cpu::nop as fn(&mut Cpu) -> u8); 0x100];
	arr[0x00] = (Cpu::fetch_1, Cpu::add);
	arr[0x01] = (Cpu::fetch_1, Cpu::add);
	arr[0x02] = (Cpu::fetch_1, Cpu::add);
	arr[0x03] = (Cpu::fetch_1, Cpu::add);
	arr[0x08] = (Cpu::fetch_1, Cpu::or);
	arr[0x09] = (Cpu::fetch_1, Cpu::or);
	arr[0x0A] = (Cpu::fetch_1, Cpu::or);
	arr[0x0B] = (Cpu::fetch_1, Cpu::or);
	arr[0x20] = (Cpu::fetch_1, Cpu::and);
	arr[0x21] = (Cpu::fetch_1, Cpu::and);
	arr[0x22] = (Cpu::fetch_1, Cpu::and);
	arr[0x23] = (Cpu::fetch_1, Cpu::and);
	arr[0x28] = (Cpu::fetch_1, Cpu::sub);
	arr[0x29] = (Cpu::fetch_1, Cpu::sub);
	arr[0x2A] = (Cpu::fetch_1, Cpu::sub);
	arr[0x2B] = (Cpu::fetch_1, Cpu::sub);
	arr[0x30] = (Cpu::fetch_1, Cpu::xor);
	arr[0x31] = (Cpu::fetch_1, Cpu::xor);
	arr[0x32] = (Cpu::fetch_1, Cpu::xor);
	arr[0x33] = (Cpu::fetch_1, Cpu::xor);
	arr[0x38] = (Cpu::fetch_1, Cpu::cmp);
	arr[0x39] = (Cpu::fetch_1, Cpu::cmp);
	arr[0x3A] = (Cpu::fetch_1, Cpu::cmp);
	arr[0x3B] = (Cpu::fetch_1, Cpu::cmp);
	arr[0x40] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x41] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x42] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x43] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x44] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x45] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x46] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x47] = (Cpu::fetch_0, Cpu::inc_reg16);
	arr[0x50] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x51] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x52] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x53] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x54] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x55] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x56] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x57] = (Cpu::fetch_0, Cpu::push_r16);
	arr[0x58] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x59] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x5A] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x5B] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x5C] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x5D] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x5E] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x5F] = (Cpu::fetch_0, Cpu::pop_r16);
	arr[0x70] = (Cpu::fetch_1, Cpu::jo);
	arr[0x71] = (Cpu::fetch_1, Cpu::jno);
	arr[0x72] = (Cpu::fetch_1, Cpu::jc);
	arr[0x73] = (Cpu::fetch_1, Cpu::jnc);
	arr[0x74] = (Cpu::fetch_1, Cpu::jz);
	arr[0x75] = (Cpu::fetch_1, Cpu::jnz);
	arr[0x76] = (Cpu::fetch_1, Cpu::jna);
	arr[0x78] = (Cpu::fetch_1, Cpu::js);
	arr[0x79] = (Cpu::fetch_1, Cpu::jns);
	arr[0x7A] = (Cpu::fetch_1, Cpu::jp);
	arr[0x7B] = (Cpu::fetch_1, Cpu::jnp);
	arr[0x80] = (Cpu::fetch_1, Cpu::x80);
	arr[0x88] = (Cpu::fetch_1, Cpu::mov_rm_r8);
	arr[0x89] = (Cpu::fetch_1, Cpu::mov_rm_r16);
	arr[0x8A] = (Cpu::fetch_1, Cpu::mov_r8_rm);
	arr[0x8B] = (Cpu::fetch_1, Cpu::mov_r16_rm);
	arr[0x8C] = (Cpu::fetch_1, Cpu::mov_rm_sr);
	arr[0x8E] = (Cpu::fetch_1, Cpu::mov_sr);
	arr[0x9E] = (Cpu::fetch_0, Cpu::sahf);
	arr[0x9F] = (Cpu::fetch_0, Cpu::lahf);
	arr[0xB0] = (Cpu::fetch_1, Cpu::mov_reg_low_imm8);
	arr[0xB1] = (Cpu::fetch_1, Cpu::mov_reg_low_imm8);
	arr[0xB2] = (Cpu::fetch_1, Cpu::mov_reg_low_imm8);
	arr[0xB3] = (Cpu::fetch_1, Cpu::mov_reg_low_imm8);
	arr[0xB4] = (Cpu::fetch_1, Cpu::mov_reg_high_imm8);
	arr[0xB5] = (Cpu::fetch_1, Cpu::mov_reg_high_imm8);
	arr[0xB6] = (Cpu::fetch_1, Cpu::mov_reg_high_imm8);
	arr[0xB7] = (Cpu::fetch_1, Cpu::mov_reg_high_imm8);
	arr[0xB8] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xB9] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xBA] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xBB] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xBC] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xBD] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xBE] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xBF] = (Cpu::fetch_2, Cpu::mov_reg_imm16);
	arr[0xC3] = (Cpu::fetch_0, Cpu::ret);
	arr[0xD0] = (Cpu::fetch_1, Cpu::dx);
	arr[0xD1] = (Cpu::fetch_1, Cpu::dx);
	arr[0xD2] = (Cpu::fetch_1, Cpu::dx);
	arr[0xD3] = (Cpu::fetch_1, Cpu::dx);
	arr[0xE2] = (Cpu::fetch_1, Cpu::loop_short);
	arr[0xE4] = (Cpu::fetch_1, Cpu::in_al_imm8);
	arr[0xE6] = (Cpu::fetch_1, Cpu::out_imm8_al);
	arr[0xE9] = (Cpu::fetch_2, Cpu::jmp_near);
	arr[0xEA] = (Cpu::fetch_4, Cpu::jmp_far);
	arr[0xEB] = (Cpu::fetch_1, Cpu::jmp);
	arr[0xEC] = (Cpu::fetch_0, Cpu::in_al_dx);
	arr[0xEE] = (Cpu::fetch_0, Cpu::out_dx_al);
	arr[0xF4] = (Cpu::fetch_0, Cpu::hlt);
	arr[0xF8] = (Cpu::fetch_0, Cpu::clc);
	arr[0xF9] = (Cpu::fetch_0, Cpu::stc);
	arr[0xFA] = (Cpu::fetch_0, Cpu::cli);
	arr[0xFE] = (Cpu::fetch_1, Cpu::inc_dec);

	arr
}

pub static INSTS: [(fn(&mut Cpu), fn(&mut Cpu) -> u8); 0x100] = generate_insts();
