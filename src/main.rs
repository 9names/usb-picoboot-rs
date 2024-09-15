mod picoboot_cmds;
mod picousb;
mod piconusb;

pub const PICO_PAGE_SIZE: usize = 256;
pub const PICO_SECTOR_SIZE: u32 = 4096;
pub const PICO_FLASH_START: u32 = 0x10000000;
pub const PICO_STACK_POINTER: u32 = 0x20042000;

#[derive(Debug, Clone, Copy)]
pub enum TargetID {
    Rp2040,
    Rp2350,
}

use uf2_decode::convert_from_uf2;

fn uf2_pages(bytes: Vec<u8>) -> Result<Vec<Vec<u8>>, ()> {
    let fw = convert_from_uf2(&bytes).map_err(|_| ())?.0;
    let mut fw_pages: Vec<Vec<u8>> = vec![];
    let len = fw.len();
    for i in (0..len).step_by(PICO_PAGE_SIZE) {
        let size = std::cmp::min(len - i, PICO_PAGE_SIZE);
        let mut page = fw[i..i + size].to_vec();
        page.resize(PICO_PAGE_SIZE, 0);
        fw_pages.push(page);
    }
    Ok(fw_pages)
}

fn main() {
    // create rusb connection object
    let ctx = rusb::Context::new().expect("Could not initialize libusb");
    let mut conn = picousb::PicobootConnection::new(ctx);

    // create nusb connection object
    // let mut conn = piconusb::PicobootConnection::new().unwrap();

    println!("Connected to PicoBoot!");

    // firmware in a big vector of u8's
    let fw_name = match conn.get_device_type() {
        Some(TargetID::Rp2040) => "fw_blink.uf2",
        Some(TargetID::Rp2350) => "fw_blink_rp2350.uf2",
        None => panic!("No known RP device connected"),
    };
    let fw = std::fs::read(fw_name).unwrap();
    let fw_pages = uf2_pages(fw).unwrap();

    println!("resetting interface");
    conn.reset_interface();
    println!("reset interface");
    println!("claiming access");
    conn.access_exclusive_eject()
        .expect("failed to claim access");
    println!("claimed access");
    conn.exit_xip().expect("failed to exit from xip mode");

    let mut erased_sectors = vec![];

    for (i, page) in fw_pages.iter().enumerate() {
        let addr = (i * PICO_PAGE_SIZE) as u32 + PICO_FLASH_START;
        let size = PICO_PAGE_SIZE as u32;
        println!("performing ops on addr={:#X}", addr);

        // Erase is by sector. Addresses must be on sector boundary
        let sector_addr = addr - (addr % PICO_SECTOR_SIZE);
        if !erased_sectors.contains(&sector_addr) {
            // Sector containing this page hasn't been erased yet, erase it now
            println!("\terasing flash");
            conn.flash_erase(addr, PICO_SECTOR_SIZE)
                .expect("failed to erase flash");
            println!("\terase flash success");
            erased_sectors.push(sector_addr);
        }

        println!("\twriting flash");
        conn.flash_write(addr, page.to_vec())
            .expect("failed to write flash");
        println!("\twrite flash success");

        println!("\treading flash");
        let read = conn.flash_read(addr, size).expect("failed to read flash");
        println!("\tread flash success");

        println!("\tcomparing flash and expected");
        let matching = page.iter().zip(&read).filter(|&(a, b)| a == b).count();
        if matching != PICO_PAGE_SIZE {
            panic!(
                "page failed to match (expected {}, got {})",
                PICO_PAGE_SIZE, matching
            )
        }
        println!("\ttotal success");
    }

    println!("sector success!!!");

    match conn.get_device_type().expect("No known RP chip found") {
        TargetID::Rp2040 => {
            conn.reboot(0x0, PICO_STACK_POINTER, 500)
                .expect("failed to reboot device"); // sp is SRAM_END_RP2040
        }
        TargetID::Rp2350 => {
            conn.reboot2_normal(500).expect("failed to reboot device")
        }
    }

    println!("reboot success");
}
