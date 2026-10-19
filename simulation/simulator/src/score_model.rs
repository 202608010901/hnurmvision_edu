// Pure scoring rules, also tested with `rustc --test` without the renderer.
#[derive(Clone, Copy, Debug, Default)]
pub struct Score {
    pub shots: u64,
    pub hits: u64,
    pub off_target: u64,
    pub damage: f64,
    pub seconds: f64,
}
impl Score {
    pub fn accuracy(&self) -> f64 { if self.shots == 0 { 0.0 } else { 100.0*self.hits as f64/self.shots as f64 } }
    pub fn dps(&self) -> f64 { if self.seconds <= 0.0 { 0.0 } else { self.damage/self.seconds } }
}
#[derive(Clone, Copy, Debug)]
pub struct Shot {
    pub epoch: u64,
    pub target: usize,
    pub damage: f64,
    pub resolved: bool,
}
#[derive(Debug)]
pub struct ScoreBook {
    pub scores: [Score; 3],
    pub selected: usize,
    pub running: bool,
    pub epoch: u64,
    pub damage_per_hit: f64,
}
impl ScoreBook {
    pub fn new(damage: f64) -> Self {
        Self { scores: [Score::default(); 3], selected: 0, running: false, epoch: 0,
            damage_per_hit: if damage.is_finite() && damage > 0.0 { damage } else { 10.0 } }
    }
    pub fn tick(&mut self, dt: f64) {
        if self.running && dt.is_finite() && dt > 0.0 { self.scores[self.selected].seconds += dt; }
    }
    pub fn select(&mut self, target: usize) {
        if !self.running && target < 3 { self.selected = target; }
    }
    pub fn reset(&mut self) {
        self.running = false;
        self.epoch += 1;
        self.scores = [Score::default(); 3];
    }
    pub fn launch(&mut self) -> Option<Shot> {
        if !self.running { return None; }
        self.scores[self.selected].shots += 1;
        Some(Shot { epoch: self.epoch, target: self.selected, damage: self.damage_per_hit, resolved: false })
    }
    // Only the first physical contact can score. Body/ground hits are misses.
    pub fn contact(&mut self, shot: &mut Shot, armor_target: Option<usize>) {
        if shot.resolved { return; }
        shot.resolved = true;
        if shot.epoch != self.epoch { return; }
        let score = &mut self.scores[shot.target];
        if armor_target == Some(shot.target) {
            score.hits += 1;
            score.damage += shot.damage;
        } else if armor_target.is_some() { score.off_target += 1; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn misses_duplicates_damage_and_time() {
        let mut b=ScoreBook::new(10.0); b.running=true; b.tick(2.0);
        let mut hit=b.launch().unwrap(); let _miss=b.launch().unwrap();
        b.contact(&mut hit,Some(0)); b.contact(&mut hit,Some(0));
        assert_eq!(b.scores[0].hits,1); assert_eq!(b.scores[0].accuracy(),50.0);
        assert_eq!(b.scores[0].damage,10.0); assert_eq!(b.scores[0].dps(),5.0);
        b.running=false; b.tick(100.0); assert!(b.launch().is_none());
        assert_eq!(b.scores[0].dps(),5.0);
    }
    #[test]
    fn reset_rejects_inflight_and_wrong_target_is_not_credited() {
        let mut b=ScoreBook::new(10.0); b.running=true;
        let mut wrong=b.launch().unwrap(); b.contact(&mut wrong,Some(1));
        assert_eq!(b.scores[0].off_target,1); assert_eq!(b.scores[1].hits,0);
        let mut old=b.launch().unwrap(); b.reset(); b.contact(&mut old,Some(0));
        assert_eq!(b.scores[0].hits,0); assert_eq!(b.scores[0].shots,0);
    }
    #[test]
    fn switched_round_keeps_shots_with_original_target_and_body_hit_consumes() {
        let mut b=ScoreBook::new(10.0); b.running=true;
        let mut old=b.launch().unwrap(); b.select(1); assert_eq!(b.selected,0);
        b.running=false; b.select(1); b.running=true; b.tick(3.0);
        b.contact(&mut old,Some(0)); assert_eq!(b.scores[0].hits,1);
        let mut body=b.launch().unwrap(); b.contact(&mut body,None); b.contact(&mut body,Some(1));
        assert_eq!(b.scores[1].hits,0); assert_eq!(b.scores[0].seconds,0.0);
        assert_eq!(b.scores[1].seconds,3.0);
    }
}
