#[derive(Debug)]
pub struct Gpu {
	pub mode: u8,
	pub crt_out1: u8
}

impl Gpu {
	pub const fn new() -> Self {
		Self {
			mode: 0,
			crt_out1: 0
		}
	}
}
