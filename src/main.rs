#[macro_use]
extern crate clap;

fn main() {
    let matches = clap_app!(datagen =>
        (version: "0.1")
        (author: "Matt Jadczak <mnj24@cam.ac.uk>")
        (about: "Generates trajectory data for simulated robots")
    ).get_matches();
}
