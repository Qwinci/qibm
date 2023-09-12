use std::fs::read;
use crate::dma::Dma;
use crate::gpu::Gpu;
use crate::pit::Pit;
use crate::ppi::Ppi;

#[derive(Debug)]
pub struct FastMem {
	mem: Vec<u8>,
	ptrs: Vec<usize>,
	page_size: usize
}

impl FastMem {
	pub fn new(total_size: usize, page_size: usize) -> Self {
		Self {
			mem: vec![0; total_size],
			ptrs: vec![0; total_size / page_size],
			page_size
		}
	}

	pub fn add_range(&mut self, start: usize, size: usize) {
		assert_eq!(start % self.page_size, 0);
		assert_eq!(size % self.page_size, 0);

		let start_page = start / self.page_size;
		let end_page = (start + size) / self.page_size;

		for i in start_page..end_page {
			self.ptrs[i] = &mut self.mem[i * self.page_size] as *mut _ as usize;
		}
	}

	pub fn remove_range(&mut self, start: usize, size: usize) {
		assert_eq!(start % self.page_size, 0);
		assert_eq!(size % self.page_size, 0);

		let start_page = start / self.page_size;
		let end_page = (start + size) / self.page_size;

		for i in start_page..end_page {
			self.ptrs[i] = 0;
		}
	}

	pub fn get(&self, addr: usize) -> Option<&[u8]> {
		let ptr = self.ptrs[addr / self.page_size];
		let off = addr % self.page_size;
		if ptr == 0 {
			None
		} else {
			unsafe {
				Some(std::slice::from_raw_parts((ptr + off) as _, self.mem.len() - addr))
			}
		}
	}

	pub fn get_mut(&mut self, addr: usize) -> Option<&mut [u8]> {
		let ptr = self.ptrs[addr / self.page_size];
		let off = addr % self.page_size;
		if ptr == 0 {
			None
		} else {
			unsafe {
				Some(std::slice::from_raw_parts_mut((ptr + off) as _, self.mem.len() - addr))
			}
		}
	}
}

#[derive(Debug)]
pub struct Bus {
	mem: FastMem,
	pub dma: Dma,
	pub gpu: Gpu,
	pub ppi: Ppi,
	pub pit: Pit,
	pub cycles: usize
}

impl Bus {
	pub fn new() -> Self {
		let mut mem = FastMem::new(0x100000, 0x4000);
		// system ram
		mem.add_range(0, 1024 * 64);
		// io channel ram
		mem.add_range(0x10000, 192 * 1024);
		// io expansion ram
		mem.add_range(0x40000, 384 * 1024);
		// graphics mem
		mem.add_range(0xA4000, 112 * 1024);
		// expansion memory
		mem.add_range(0xC0000, 192 * 1024);
		// unknown bios memory???
		mem.add_range(0xF0000, 16 * 1024);
		// base system rom
		mem.add_range(0xF4000, 48 * 1024);

		let bios = read("roms/BIOS.bin").unwrap();
		let ptr = &mut mem.get_mut(0xFE000).unwrap()[0..0x2000];
		ptr.copy_from_slice(&bios);

		Self {
			mem,
			dma: Dma::new(),
			gpu: Gpu::new(),
			ppi: Ppi::new(),
			pit: Pit::new(),
			cycles: 0
		}
	}

	pub fn read(&mut self, addr: u32) -> u8 {
		if let Some(mem) = self.mem.get(addr as usize) {
			mem[0]
		} else {
			eprintln!("bus: unimplemented read from {:#X}", addr);
			0
		}
	}

	pub fn write(&mut self, addr: u32, value: u8) {
		if let Some(mem) = self.mem.get_mut(addr as usize) {
			mem[0] = value;
		}
	}

	pub fn clock(&mut self) {
		if self.cycles % 4 == 0 {
			// clock the pit every 4 cpu cycle for approx 1.19mhz
			self.pit.clock();
			self.dma.clock();
		}
		self.cycles += 1;
	}
}