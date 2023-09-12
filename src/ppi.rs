#[derive(Debug)]
pub struct Ppi {
	pub control: u8
}

impl Ppi {
	pub const fn new() -> Self {
		Self { control: 0 }
	}
}