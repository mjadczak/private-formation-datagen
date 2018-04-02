use base::*;

mod example;
mod feature;

pub use self::example::*;
pub use self::feature::*;

use std::path::{Path, PathBuf};
use std::io;
use std::fs;
use std::io::prelude::*;
use flate2::write::ZlibEncoder;
use time;
use std::marker::PhantomData;
use simulation;
use std::collections::HashMap;
use protobuf::{self, Message};
use std::error::Error;
use std::mem::size_of;
use crc::crc32;
use byteorder::{WriteBytesExt, LE};
use std::fmt;

#[derive(PartialEq, Debug, Clone, Copy)]
struct DataInfo {
    time_step: Seconds,
    num_steps: usize,
    num_robots: usize,
    // todo params, noise?
}

#[derive(Debug)]
pub enum TfRecordError {
    IoError(io::Error),
    ProtobufError(protobuf::ProtobufError),
    ArgumentError {
        message: &'static str
    }
}

use self::TfRecordError::*;

pub type TfRecordResult<T> = Result<T, TfRecordError>;

impl TfRecordError {
    pub fn from_str(message: &'static str) -> TfRecordError {
        ArgumentError {
            message
        }
    }
}

impl Error for TfRecordError {
    fn description(&self) -> &str {
        match self {
            &ArgumentError { message: m } => m,
            &IoError(_) => "IO error",
            &ProtobufError(_) => "ProtoBuf error: {}"
        }
    }

    fn cause(&self) -> Option<&Error> {
        match self {
            &ArgumentError { .. } => None,
            &ProtobufError(ref e) => Some(e),
            &IoError(ref e) => Some(e)
        }
    }
}

impl fmt::Display for TfRecordError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(self.description())
    }
}

impl From<io::Error> for TfRecordError {
    fn from(err: io::Error) -> Self {
        IoError(err)
    }
}

impl From<protobuf::ProtobufError> for TfRecordError {
    fn from(err: protobuf::ProtobufError) -> Self {
        ProtobufError(err)
    }
}

#[derive(Debug)]
pub struct ResultsWriter<S: Vector> {
    info: Option<DataInfo>,
    out_stream: ZlibEncoder<fs::File>,
    file_path: PathBuf,
    _marker: PhantomData<S>
}

impl ResultsWriter<Metres> {
    /// Creates a new writer from a new output path.
    /// This path must be a folder, but does not have to exist.
    pub fn from_path<P: AsRef<Path>>(path: P) -> TfRecordResult<ResultsWriter<Metres>> {
        let path = path.as_ref();
        if path.exists() && !path.is_dir() {
            return Err(TfRecordError::from_str("the input path must be a directory"));
        }
        fs::create_dir_all(path)?;
        let mut file_path = path.join(Self::get_prefix());
        assert!(file_path.set_extension("tfrecord"));
        let writer = fs::File::create(&file_path)?;

        Ok(ResultsWriter::<Metres> {
            info: None,
            out_stream: ZlibEncoder::new(writer, Default::default()),
            file_path,
            _marker: Default::default(),
        })
    }

    pub fn write_record<R: simulation::SimulationResult<Metres>>(&mut self, result: R, leader_number: usize) -> TfRecordResult<()> {
        // check that all the static info is still the same
        self.verify_info(&result)?;

        // construct an `Example` message
        let data = result.into_data();

        // first, construct the features
        let mut features = HashMap::<String, Feature>::new();

        // add the leader feature
        let mut leader_num_list = Int64List::new();
        leader_num_list.set_value(vec![leader_number as i64]);
        let mut leader_num_feature = Feature::new();
        leader_num_feature.set_int64_list(leader_num_list);
        features.insert("leader_number".to_string(), leader_num_feature);

        // add the trajectory features per robot
        for (i, x) in data.into_iter().enumerate() {
            let mut feature_list = FloatList::new();
            feature_list.set_value(x.into_iter().map(|v| v as f32).collect());

            let mut feature = Feature::new();
            feature.set_float_list(feature_list);

            let key = format!("x{}", i);
            features.insert(key, feature);
        }

        // make the example
        let mut features_msg = Features::new();
        features_msg.set_feature(features);
        let mut example = Example::new();
        example.set_features(features_msg);

        let data_length = example.compute_size() as u64;



        // Format of a single record:
        // all fields little-endian
        //  uint64    length
        //  uint32    masked crc of length
        //  byte      data[length]
        //  uint32    masked crc of data

        self.out_stream.write_u64::<LE>(data_length)?;
        self.out_stream.write_u32::<LE>(masked_crc32c_u64(data_length))?;
        let data_crc = {
            let mut writer = Crc32CWriter::new(&mut self.out_stream);
            example.write_to_writer(&mut writer)?;
            writer.finish()
        };
        self.out_stream.write_u32::<LE>(data_crc)?;

        Ok(())
    }

    fn get_prefix() -> String {
        time::strftime("dg_%Y-%m-%d_%H-%M-%S", &time::now()).unwrap()
    }

    fn verify_info<R: simulation::SimulationResult<Metres>>(&mut self, result: &R) -> TfRecordResult<()> {
        let cmp = DataInfo {
            time_step: result.time_step(),
            num_robots: result.num_robots(),
            num_steps: result.num_steps()
        };

        // if empty, insert and comparison will succeed
        // else, get the current value
        if *self.info.get_or_insert(cmp) == cmp {
            Ok(())
        } else {
            Err(TfRecordError::from_str("all records written to a single file must have the same static parameters"))
        }
    }
}

impl<S: Vector> Drop for ResultsWriter<S> {
    fn drop(&mut self) {
        if let None = self.info {
            return;
        }
        let info = self.info.unwrap();
        // writer automatically gets dropped and saves off, but we need to save off the info file
        let data_file_name = self.file_path.file_name().unwrap().to_str().unwrap().to_string();
        assert!(self.file_path.set_extension("dginfo"));
        let mut file = fs::File::create(&self.file_path).unwrap();
        // todo YAML
        writeln!(&mut file, "# datagen-info v0.1").unwrap();
        writeln!(&mut file, "time_step: {}", info.time_step).unwrap();
        writeln!(&mut file, "num_robots: {}", info.num_robots).unwrap();
        writeln!(&mut file, "num_steps: {}", info.num_steps).unwrap();
        writeln!(&mut file, "data_file: {}", data_file_name).unwrap();
    }
}

// https://github.com/tensorflow/tensorflow/blob/49c20c5814dd80f81ced493d362d374be9ab0b3e/tensorflow/core/lib/hash/crc32c.h#L40
#[inline]
fn mask_crc(crc: u32) -> u32 {
    const K_MASK_DELTA: u32 = 0xa282ead8;
    ((crc >> 15) | (crc << 17)).wrapping_add(K_MASK_DELTA)
}

fn masked_crc32c_u64(val: u64) -> u32 {
    let mut bytes: Vec<u8> = Vec::with_capacity(size_of::<u64>());
    bytes.write_u64::<LE>(val).unwrap();
    mask_crc(crc32::checksum_castagnoli(&bytes))
}

/// A masked CRC32C writer
struct Crc32CWriter<'w, W: Write + 'w> {
    writer: &'w mut W,
    value: u32,
}

impl<'w, W: Write + 'w> Crc32CWriter<'w, W> {
    pub fn new(writer: &'w mut W) -> Crc32CWriter<'w, W> {
        Crc32CWriter {
            writer,
            value: 0
        }
    }

    pub fn reset(&mut self) {
        self.value = 0;
    }

    pub fn finish(self) -> u32 {
        mask_crc(self.value)
    }
}

impl<'w, W: Write + 'w> Write for Crc32CWriter<'w, W> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, io::Error> {
        let num_bytes = self.writer.write(buf)?;
        self.value = crc32::update(self.value, &crc32::CASTAGNOLI_TABLE, &buf[..num_bytes]);
        Ok(num_bytes)
    }

    fn flush(&mut self) -> Result<(), io::Error> {
        self.writer.flush()
    }
}