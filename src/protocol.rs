pub const DATA_FRAME_SIZE: usize = 7;

const DATA_FRAME_START_BYTE: u8 = 104u8;
const DATA_FRAME_END_BYTE: u8 = 22u8;

const DESK_TO_PANEL_HEIGHT_BYTE: u8 = 0u8;

const PANEL_TO_DESK_UP_BYTE: u8 = 1u8;
const PANEL_TO_DESK_DOWN_BYTE: u8 = 2u8;
const PANEL_TO_DESK_NO_KEY_BYTE: u8 = 3u8;
const PANEL_TO_DESK_DESK_RESET_BYTE: u8 = 4u8;
const PANEL_TO_DESK_ONE_BYTE: u8 = 6u8;
const PANEL_TO_DESK_TWO_BYTE: u8 = 7u8;
const PANEL_TO_DESK_THREE_BYTE: u8 = 8u8;
const PANEL_TO_DESK_RESET_ONE_BYTE: u8 = 10u8;
const PANEL_TO_DESK_RESET_TWO_BYTE: u8 = 11u8;
const PANEL_TO_DESK_RESET_THREE_BYTE: u8 = 12u8;

pub type DataFrame = [u8; DATA_FRAME_SIZE];

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PanelToDeskMessage {
    Up,
    Down,
    NoKey,
    DeskReset,
    One(f32),
    Two(f32),
    Three(f32),
    ResetOne,
    ResetTwo,
    ResetThree,
    Unknown(u8, u8, u8, u8, u8),
}

impl PanelToDeskMessage {
    pub fn as_frame(&self) -> DataFrame {
        match *self {
            PanelToDeskMessage::Up => build_frame(PANEL_TO_DESK_UP_BYTE, 0u8, 0u8),
            PanelToDeskMessage::Down => build_frame(PANEL_TO_DESK_DOWN_BYTE, 0u8, 0u8),
            PanelToDeskMessage::NoKey => build_frame(PANEL_TO_DESK_NO_KEY_BYTE, 0u8, 0u8),
            PanelToDeskMessage::DeskReset => build_frame(PANEL_TO_DESK_DESK_RESET_BYTE, 0u8, 0u8),
            PanelToDeskMessage::One(target_height) => {
                let (height_msb, height_lsb) = height_to_bytes(target_height, 0.0);
                build_frame(PANEL_TO_DESK_ONE_BYTE, height_lsb, height_msb)
            }
            PanelToDeskMessage::Two(target_height) => {
                let (height_msb, height_lsb) = height_to_bytes(target_height, 0.0);
                build_frame(PANEL_TO_DESK_TWO_BYTE, height_lsb, height_msb)
            }
            PanelToDeskMessage::Three(target_height) => {
                let (height_msb, height_lsb) = height_to_bytes(target_height, 0.0);
                build_frame(PANEL_TO_DESK_THREE_BYTE, height_lsb, height_msb)
            }
            PanelToDeskMessage::ResetOne => build_frame(PANEL_TO_DESK_RESET_ONE_BYTE, 0u8, 0u8),
            PanelToDeskMessage::ResetTwo => build_frame(PANEL_TO_DESK_RESET_TWO_BYTE, 0u8, 0u8),
            PanelToDeskMessage::ResetThree => build_frame(PANEL_TO_DESK_RESET_THREE_BYTE, 0u8, 0u8),
            PanelToDeskMessage::Unknown(a, b, c, d, e) => {
                [DATA_FRAME_START_BYTE, a, b, c, d, e, DATA_FRAME_END_BYTE]
            }
        }
    }

    pub fn from_frame(buf: &DataFrame) -> PanelToDeskMessage {
        // TODO: validate checksum somewhere. Or don't; just pass it on to desk?
        match buf[2] {
            PANEL_TO_DESK_UP_BYTE => PanelToDeskMessage::Up,
            PANEL_TO_DESK_DOWN_BYTE => PanelToDeskMessage::Down,
            PANEL_TO_DESK_NO_KEY_BYTE => PanelToDeskMessage::NoKey,
            PANEL_TO_DESK_DESK_RESET_BYTE => PanelToDeskMessage::DeskReset,
            PANEL_TO_DESK_ONE_BYTE => {
                PanelToDeskMessage::One(bytes_to_height_cm(buf[4], buf[3], 0.0))
            }
            PANEL_TO_DESK_TWO_BYTE => {
                PanelToDeskMessage::Two(bytes_to_height_cm(buf[4], buf[3], 0.0))
            }
            PANEL_TO_DESK_THREE_BYTE => {
                PanelToDeskMessage::Three(bytes_to_height_cm(buf[4], buf[3], 0.0))
            }
            PANEL_TO_DESK_RESET_ONE_BYTE => PanelToDeskMessage::ResetOne,
            PANEL_TO_DESK_RESET_TWO_BYTE => PanelToDeskMessage::ResetTwo,
            PANEL_TO_DESK_RESET_THREE_BYTE => PanelToDeskMessage::ResetThree,
            _ => PanelToDeskMessage::Unknown(buf[1], buf[2], buf[3], buf[4], buf[5]),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DeskToPanelMessage {
    Height(f32),
    Unknown(u8, u8, u8, u8, u8),
}

impl DeskToPanelMessage {
    pub fn as_frame(&self) -> DataFrame {
        match *self {
            DeskToPanelMessage::Height(h) => {
                // TODO: handle height outside of range
                let (height_msb, height_lsb) = height_to_bytes(h, 65.0);
                build_frame(DESK_TO_PANEL_HEIGHT_BYTE, height_msb, height_lsb)
            }
            DeskToPanelMessage::Unknown(a, b, c, d, e) => {
                [DATA_FRAME_START_BYTE, a, b, c, d, e, DATA_FRAME_END_BYTE]
            }
        }
    }

    pub fn from_frame(frame: &DataFrame) -> DeskToPanelMessage {
        // TODO: validate checksum somewhere. Or don't; just pass it on to panel?
        match frame[2] {
            DESK_TO_PANEL_HEIGHT_BYTE => {
                DeskToPanelMessage::Height(bytes_to_height_cm(frame[3], frame[4], 65.0))
            }
            _ => DeskToPanelMessage::Unknown(frame[1], frame[2], frame[3], frame[4], frame[5]),
        }
    }
}

pub fn is_start_byte(b: u8) -> bool {
    b == DATA_FRAME_START_BYTE
}

fn build_frame(b2: u8, b3: u8, b4: u8) -> DataFrame {
    [
        DATA_FRAME_START_BYTE,
        1u8,
        b2,
        b3,
        b4,
        checksum(&[1u8, b2, b3, b4]),
        DATA_FRAME_END_BYTE,
    ]
}

pub fn validate_frame(frame: &DataFrame) -> bool {
    if frame.len() != DATA_FRAME_SIZE {
        return false;
    }

    if frame[0] != DATA_FRAME_START_BYTE {
        return false;
    }

    if frame[DATA_FRAME_SIZE - 1] != DATA_FRAME_END_BYTE {
        return false;
    }

    return true;
}

fn bytes_to_height_cm(msb: u8, lsb: u8, offset_cm: f32) -> f32 {
    (256.0 * msb as f32 + lsb as f32) / 10.0 + offset_cm
}

fn height_to_bytes(height_cm: f32, offset_cm: f32) -> (u8, u8) {
    let net_height_mm = (height_cm - offset_cm) * 10.0;
    let msb = (net_height_mm / 256.0) as u8;

    let lsb = (net_height_mm - (msb as f32 * 256.0)) as u8;
    (msb, lsb)
}

fn checksum(b: &[u8]) -> u8 {
    // TODO: can we do the modulo inline to avoid up-casting to usize? Is it worth it?
    (b.iter().map(|x| *x as usize).sum::<usize>() % 256) as u8
}
