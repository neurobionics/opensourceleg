use crate::logger::Logger;

mod logger;
mod record;
fn main() {
    Logger::init(None, None, None, false);
    Logger::debug(String::from("yaya"));
    Logger::info(String::from("yaya"));
    Logger::trace(String::from("yaya"));
    Logger::warn(String::from("yaya"));
    Logger::error(String::from("yaya"));
}