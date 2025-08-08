use std::time::{Duration, Instant};

pub struct Profiler {
    n: u64, //recorded intervals
    agg: f64, //aggregate time
    aggvar: f64, //aggregate variance
    t0: Option<Instant>, //time 0
}

impl Profiler {
    pub fn new() -> Self{
        Profiler{
            n: 0,
            agg: 0.0,
            aggvar: 0.0,
            t0: None,
        }
    }

    pub fn tic(&mut self){
        self.t0 = Some(Instant::now())
    }

    pub fn toc(&mut self) -> Duration{
        if let Some(inst) = self.t0 {
            let t = inst.elapsed();
            self.n += 1;
            self.agg += 1.0;
            self.aggvar += t.as_millis().pow(2) as f64;
            return t
        }
        return Duration::from_secs(0)
    }

    pub fn profile<T, F: FnOnce() -> T>(&mut self, func: F) -> T{
        self.tic();
        let res: T = func();
        self.toc();
        return res;
    }
}