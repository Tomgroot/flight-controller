use crossterm::event::{
    self, Event, KeyCode, KeyEventKind, KeyboardEnhancementFlags, PopKeyboardEnhancementFlags,
    PushKeyboardEnhancementFlags,
};
use crossterm::{execute, terminal};
use fc_core::{Channel, RcChannels, RC_MAX, RC_MID};
use std::collections::HashSet;
use std::io::{self, stdout};
use std::sync::{Arc, Mutex};

struct State {
    keys_pressed: HashSet<KeyCode>,
}

impl State {
    fn is_pressed(&self, key: KeyCode) -> bool {
        self.keys_pressed.contains(&key)
    }
    fn direction(&self, positive: KeyCode, negative: KeyCode) -> f32 {
        f32::from(self.is_pressed(positive)) - f32::from(self.is_pressed(negative))
    }
    fn yaw(&self) -> f32 {
        return self.direction(KeyCode::Char('a'), KeyCode::Char('d'));
    }
    fn throttle(&self) -> f32 {
        return self.direction(KeyCode::Char('w'), KeyCode::Char('s'));
    }
}

pub struct KeyboardRc {
    state: Arc<Mutex<State>>,
}

impl KeyboardRc {
    pub fn spawn() -> std::io::Result<Self> {
        if !terminal::supports_keyboard_enhancement()? {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "terminal does not support keyboard",
            ));
        }

        terminal::enable_raw_mode()?;

        execute!(
            stdout(),
            PushKeyboardEnhancementFlags(
                KeyboardEnhancementFlags::DISAMBIGUATE_ESCAPE_CODES
                    | KeyboardEnhancementFlags::REPORT_EVENT_TYPES
                    | KeyboardEnhancementFlags::REPORT_ALL_KEYS_AS_ESCAPE_CODES
            )
        )?;

        let state = Arc::new(Mutex::new(State {
            keys_pressed: HashSet::new(),
        }));

        let shared = Arc::clone(&state);

        std::thread::spawn(move || loop {
            let Ok(Event::Key(key)) = event::read() else {
                continue;
            };

            let mut state = shared.lock().unwrap();

            match key.kind {
                KeyEventKind::Press => {
                    state.keys_pressed.insert(key.code);
                }
                KeyEventKind::Release => {
                    state.keys_pressed.remove(&key.code);
                }
                _ => {}
            }
        });

        Ok(Self { state })
    }

    pub fn sample(&self) -> RcChannels {
        let state = self.state.lock().unwrap();
        let mut rc = RcChannels::default();

        rc[Channel::Roll] = deflection_to_rc(0.0);
        rc[Channel::Pitch] = deflection_to_rc(0.0);
        rc[Channel::Throttle] = deflection_to_rc(state.throttle());
        rc[Channel::Yaw] = deflection_to_rc(state.yaw());
        rc[Channel::Arm] = RC_MAX;
        return rc;
    }
}

fn deflection_to_rc(deflection: f32) -> u16 {
    (RC_MID as f32 + deflection * (RC_MAX - RC_MID) as f32) as u16
}

impl Drop for KeyboardRc {
    fn drop(&mut self) {
        let _ = execute!(stdout(), PopKeyboardEnhancementFlags);
        let _ = terminal::disable_raw_mode();
    }
}
