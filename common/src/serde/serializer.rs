use std::{
    io::{self, Seek, Write},
    iter::repeat_n,
};

#[rustfmt::skip]
pub trait Serializer {
    fn pos(&mut self) -> usize;
    fn write_bytes(&mut self, data: &[u8]);
    fn reserve(&mut self, length: usize) -> usize;
    fn execute_at(&mut self, offset: usize, f: impl FnOnce(&mut SizedSerializer));
    fn view_mut(&mut self, offset: usize, size: usize) -> &mut [u8];

    fn write_bool(&mut self, data: bool) { self.write_u8(data as u8); }
    fn write_u8(&mut self, data: u8) { self.write_bytes(&[data]); }
    fn write_u16_be(&mut self, data: u16) { self.write_bytes(&data.to_be_bytes()); }
    fn write_u16_le(&mut self, data: u16) { self.write_bytes(&data.to_le_bytes()); }
    fn write_u32_be(&mut self, data: u32) { self.write_bytes(&data.to_be_bytes()); }
    fn write_u32_le(&mut self, data: u32) { self.write_bytes(&data.to_le_bytes()); }
    fn write_u64_be(&mut self, data: u64) { self.write_bytes(&data.to_be_bytes()); }
    fn write_u64_le(&mut self, data: u64) { self.write_bytes(&data.to_le_bytes()); }
    fn write_f32_be(&mut self, data: f32) { self.write_bytes(&data.to_be_bytes()); }
    fn write_f32_le(&mut self, data: f32) { self.write_bytes(&data.to_le_bytes()); }
    fn write_f64_be(&mut self, data: f64) { self.write_bytes(&data.to_be_bytes()); }
    fn write_f64_le(&mut self, data: f64) { self.write_bytes(&data.to_le_bytes()); }
}

pub struct SizedSerializer<'a> {
    buffer: &'a mut [u8],
    offset: usize,
}

pub struct DynamicSerializer {
    buffer: Vec<u8>,
}

/// Writes to a stream. The first write error is kept and the writes after
/// it are skipped: [`WriterSerializer::finish`] returns it.
pub struct WriterSerializer<T: Write + Seek> {
    stream: T,
    error: Option<io::Error>,
    /// Handed out by `view_mut`, which a stream can't do.
    scratch: Vec<u8>,
}

impl<'a> SizedSerializer<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer, offset: 0 }
    }
}

impl DynamicSerializer {
    pub fn new() -> Self {
        Self { buffer: Vec::new() }
    }

    pub fn inner_mut(&mut self) -> &mut Vec<u8> {
        &mut self.buffer
    }

    pub fn into_inner(self) -> Vec<u8> {
        self.buffer
    }
}

impl<T: Write + Seek> WriterSerializer<T> {
    pub fn new(stream: T) -> Self {
        Self {
            stream,
            error: None,
            scratch: Vec::new(),
        }
    }

    fn fail(&mut self, error: io::Error) {
        self.error.get_or_insert(error);
    }

    /// Flushes the stream and gives it back, or the first error.
    pub fn finish(mut self) -> io::Result<T> {
        if let Some(error) = self.error.take() {
            return Err(error);
        }
        self.stream.flush()?;
        Ok(self.stream)
    }
}

impl Serializer for SizedSerializer<'_> {
    fn pos(&mut self) -> usize {
        self.offset
    }

    /// Writes what fits: the buffer is sized by the caller beforehand.
    fn write_bytes(&mut self, data: &[u8]) {
        let start = self.offset.min(self.buffer.len());
        let room = &mut self.buffer[start..];
        let n = data.len().min(room.len());
        room[..n].copy_from_slice(&data[..n]);
        self.offset = self.offset.saturating_add(data.len());
    }

    fn reserve(&mut self, length: usize) -> usize {
        let out = self.offset;
        self.offset += length;
        out
    }

    fn execute_at(&mut self, offset: usize, f: impl FnOnce(&mut SizedSerializer)) {
        let mut ser = SizedSerializer::new(&mut self.buffer[offset..]);
        f(&mut ser);
    }

    fn view_mut(&mut self, offset: usize, size: usize) -> &mut [u8] {
        &mut self.buffer[offset..(offset + size)]
    }
}

impl Serializer for DynamicSerializer {
    fn pos(&mut self) -> usize {
        self.buffer.len()
    }

    fn write_bytes(&mut self, data: &[u8]) {
        self.buffer.extend_from_slice(data);
    }

    fn reserve(&mut self, length: usize) -> usize {
        let start = self.buffer.len();
        self.buffer.extend(repeat_n(0, length));
        start
    }

    fn execute_at(&mut self, offset: usize, f: impl FnOnce(&mut SizedSerializer)) {
        let mut ser = SizedSerializer::new(&mut self.buffer[offset..]);
        f(&mut ser);
    }

    fn view_mut(&mut self, offset: usize, size: usize) -> &mut [u8] {
        &mut self.buffer[offset..(offset + size)]
    }
}

impl<T: Write + Seek> Serializer for WriterSerializer<T> {
    fn pos(&mut self) -> usize {
        match self.stream.stream_position() {
            Ok(pos) => pos as usize,
            Err(e) => {
                self.fail(e);
                0
            }
        }
    }

    fn write_bytes(&mut self, data: &[u8]) {
        if self.error.is_none()
            && let Err(e) = self.stream.write_all(data)
        {
            self.fail(e);
        }
    }

    fn reserve(&mut self, length: usize) -> usize {
        let start = self.pos();
        let zeros = [0_u8; 256];
        let mut left = length;
        while left > 0 && self.error.is_none() {
            let n = left.min(zeros.len());
            self.write_bytes(&zeros[..n]);
            left -= n;
        }
        start
    }

    fn execute_at(&mut self, _offset: usize, _f: impl FnOnce(&mut SizedSerializer)) {
        self.fail(unsupported());
    }

    fn view_mut(&mut self, _offset: usize, size: usize) -> &mut [u8] {
        self.fail(unsupported());
        self.scratch.clear();
        self.scratch.resize(size, 0);
        &mut self.scratch
    }
}

fn unsupported() -> io::Error {
    io::Error::new(
        io::ErrorKind::Unsupported,
        "not supported yet: writing back into a stream",
    )
}

impl Default for DynamicSerializer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use std::io::{Cursor, SeekFrom};

    use super::*;

    /// Takes `room` bytes, then fails like a full disk.
    struct Full {
        data: Cursor<Vec<u8>>,
        room: usize,
    }

    impl Write for Full {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            if self.data.get_ref().len() + buf.len() > self.room {
                return Err(io::Error::other("disk full"));
            }
            self.data.write(buf)
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Seek for Full {
        fn seek(&mut self, pos: SeekFrom) -> io::Result<u64> {
            self.data.seek(pos)
        }
    }

    #[test]
    fn a_write_error_is_reported() {
        let mut ser = WriterSerializer::new(Full {
            data: Cursor::new(Vec::new()),
            room: 6,
        });
        ser.write_u32_be(1);
        ser.write_u32_be(2);
        ser.write_u8(3);
        assert!(ser.finish().is_err());

        let mut ser = WriterSerializer::new(Cursor::new(Vec::new()));
        ser.write_u16_be(0x0102);
        let at = ser.reserve(2);
        assert_eq!(at, 2);
        let out = ser.finish().unwrap().into_inner();
        assert_eq!(out, [1, 2, 0, 0]);
    }

    #[test]
    fn writing_past_a_sized_buffer_does_not_crash() {
        let mut buffer = [0; 3];
        let mut ser = SizedSerializer::new(&mut buffer);
        ser.write_u32_be(0x01020304);
        assert_eq!(buffer, [1, 2, 3]);
    }
}
