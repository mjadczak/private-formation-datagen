#[macro_use]
extern crate clap;
extern crate pid_control;
extern crate num;

pub mod simulation;
pub mod trajectory;
pub mod base;

fn main() {
    let matches = clap_app!(datagen =>
        (version: "0.1")
        (author: "Matt Jadczak <mnj24@cam.ac.uk>")
        (about: "Generates trajectory data for simulated robots")
    ).get_matches();
}
