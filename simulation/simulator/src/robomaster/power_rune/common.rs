#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum RuneMode {
    Small,
    Large,
}

impl RuneMode {
    pub fn from_config(value: &str) -> Option<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "small" => Some(Self::Small),
            "large" | "big" => Some(Self::Large),
            _ => None,
        }
    }
}

pub const RUNE_TARGET_COUNT: usize = 5;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RuneHitOutcome {
    Ignored,
    WrongTarget,
    PrimaryHit,
    SecondaryHit,
    Activated,
}

impl RuneHitOutcome {
    pub const fn is_accurate(self) -> bool {
        matches!(
            self,
            Self::PrimaryHit | Self::SecondaryHit | Self::Activated
        )
    }

    pub const fn activates_rune(self) -> bool {
        matches!(self, Self::Activated)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RuneTransition {
    None,
    Started,
    Restarted,
    Advanced,
    Failed,
    Activated,
    ResetToInactive,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rune_mode_accepts_small_large_and_big() {
        assert_eq!(RuneMode::from_config("small"), Some(RuneMode::Small));
        assert_eq!(RuneMode::from_config(" LARGE "), Some(RuneMode::Large));
        assert_eq!(RuneMode::from_config("big"), Some(RuneMode::Large));
        assert_eq!(RuneMode::from_config("invalid"), None);
    }
}
