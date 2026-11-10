use std::{
    borrow::Cow,
    io::{self, ErrorKind, Read, Seek, SeekFrom},
};

#[rustfmt::skip]
pub trait Deserializer {
    fn pos(&mut self) -> usize;
    fn size(&mut self) -> usize;
    fn advance_by(&mut self, amount: usize);
    fn jump_to(&mut self, pos: usize);
    fn execute_at<T>(&mut self, pos: usize, func: impl FnOnce(&mut Self) -> T) -> T;
    fn read_bytes(&mut self, length: usize) -> Cow<'_, [u8]>;
    fn is_eof(&mut self) -> bool;
    /// Whether a read ran past the end of the input (or failed): what was
    /// read since is zeros, and the result shouldn't be trusted.
    fn truncated(&self) -> bool;

    fn read_array<const LENGTH: usize>(&mut self) -> [u8; LENGTH] {
        <[u8; LENGTH]>::try_from(&*self.read_bytes(LENGTH)).unwrap_or([0; LENGTH])
    }

    /// A list stored as a big endian u32 count and the items. Stops at the
    /// end of the input, so a damaged count can't make it loop or allocate
    /// beyond what the input holds.
    fn read_list<T>(&mut self, mut item: impl FnMut(&mut Self) -> T) -> Vec<T> where Self: Sized {
        let count = self.read_u32_be();
        let mut out = Vec::new();
        for _ in 0..count {
            if self.truncated() { break; }
            out.push(item(self));
        }
        out
    }

    fn read_bool(&mut self) -> bool { self.read_u8() != 0 }
    fn read_u8(&mut self) -> u8 { self.read_array::<1>()[0] }
    fn read_u16_be(&mut self) -> u16 { u16::from_be_bytes(self.read_array()) }
    fn read_u16_le(&mut self) -> u16 { u16::from_le_bytes(self.read_array()) }
    fn read_i16_be(&mut self) -> i16 { i16::from_be_bytes(self.read_array()) }
    fn read_i16_le(&mut self) -> i16 { i16::from_le_bytes(self.read_array()) }
    fn read_u32_be(&mut self) -> u32 { u32::from_be_bytes(self.read_array()) }
    fn read_u32_le(&mut self) -> u32 { u32::from_le_bytes(self.read_array()) }
    fn read_i32_be(&mut self) -> i32 { i32::from_be_bytes(self.read_array()) }
    fn read_i32_le(&mut self) -> i32 { i32::from_le_bytes(self.read_array()) }
    fn read_u64_be(&mut self) -> u64 { u64::from_be_bytes(self.read_array()) }
    fn read_u64_le(&mut self) -> u64 { u64::from_le_bytes(self.read_array()) }
    fn read_i64_be(&mut self) -> i64 { i64::from_be_bytes(self.read_array()) }
    fn read_i64_le(&mut self) -> i64 { i64::from_le_bytes(self.read_array()) }
    fn read_f32_be(&mut self) -> f32 { f32::from_be_bytes(self.read_array()) }
    fn read_f32_le(&mut self) -> f32 { f32::from_le_bytes(self.read_array()) }
    fn read_f64_be(&mut self) -> f64 { f64::from_be_bytes(self.read_array()) }
    fn read_f64_le(&mut self) -> f64 { f64::from_le_bytes(self.read_array()) }
}

/// Reads from a byte slice. Reading past the end gives fewer bytes (the
/// fixed size reads give zeros) and marks the input as truncated.
pub struct SliceDeserializer<'a> {
    buffer: &'a [u8],
    offset: usize,
    truncated: bool,
}

/// Reads from a stream. A read or seek error, or reading past the end,
/// gives fewer bytes (the fixed size reads give zeros) and is kept: see
/// [`ReaderDeserializer::error`].
pub struct ReaderDeserializer<T: Read> {
    reader: T,
    error: Option<io::Error>,
    truncated: bool,
}

impl<'a> SliceDeserializer<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            buffer: data,
            offset: 0,
            truncated: false,
        }
    }

    /// Up to `length` bytes from the current position.
    pub fn read_slice(&mut self, length: usize) -> &'a [u8] {
        let start = self.offset.min(self.buffer.len());
        let end = self.offset.saturating_add(length).min(self.buffer.len());
        if end - start < length {
            self.truncated = true;
        }
        self.offset = self.offset.saturating_add(length);
        &self.buffer[start..end]
    }
}

impl<T: Read> ReaderDeserializer<T> {
    pub fn new(reader: T) -> Self {
        Self {
            reader,
            error: None,
            truncated: false,
        }
    }

    /// The first read or seek error, if any.
    pub fn error(&self) -> Option<&io::Error> {
        self.error.as_ref()
    }

    fn fail(&mut self, error: io::Error) {
        self.error.get_or_insert(error);
    }

    /// Up to `length` bytes. Reads in steps, so a bogus length from a
    /// damaged file can't allocate more than the file holds.
    fn read_vec(&mut self, length: usize) -> Vec<u8> {
        const STEP: usize = 1 << 20;
        let mut buf = Vec::with_capacity(length.min(STEP));
        let mut chunk = [0_u8; 8192];
        while buf.len() < length {
            let want = (length - buf.len()).min(chunk.len());
            match self.reader.read(&mut chunk[..want]) {
                Ok(0) => break,
                Ok(n) => buf.extend_from_slice(&chunk[..n]),
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => {
                    self.fail(e);
                    break;
                }
            }
        }
        if buf.len() < length {
            self.truncated = true;
        }
        buf
    }
}

impl<'a> Deserializer for SliceDeserializer<'a> {
    fn pos(&mut self) -> usize {
        self.offset
    }

    fn size(&mut self) -> usize {
        self.buffer.len()
    }

    fn advance_by(&mut self, amount: usize) {
        self.offset = self.offset.saturating_add(amount);
    }

    fn jump_to(&mut self, pos: usize) {
        self.offset = pos;
    }

    fn execute_at<T>(&mut self, pos: usize, func: impl FnOnce(&mut Self) -> T) -> T {
        let offset = self.offset;
        self.jump_to(pos);
        let result = func(self);
        self.jump_to(offset);
        result
    }

    fn read_bytes(&mut self, length: usize) -> Cow<'_, [u8]> {
        Cow::Borrowed(self.read_slice(length))
    }

    fn is_eof(&mut self) -> bool {
        self.offset >= self.buffer.len()
    }

    fn truncated(&self) -> bool {
        self.truncated
    }
}

impl<T: Read + Seek> ReaderDeserializer<T> {
    fn seek(&mut self, to: SeekFrom) -> u64 {
        match self.reader.seek(to) {
            Ok(pos) => pos,
            Err(e) => {
                self.fail(e);
                0
            }
        }
    }
}

impl<T: Read + Seek> Deserializer for ReaderDeserializer<T> {
    fn pos(&mut self) -> usize {
        self.seek(SeekFrom::Current(0)) as usize
    }

    fn size(&mut self) -> usize {
        let pos = self.seek(SeekFrom::Current(0));
        let len = self.seek(SeekFrom::End(0));
        if pos != len {
            self.seek(SeekFrom::Start(pos));
        }
        len as usize
    }

    fn advance_by(&mut self, amount: usize) {
        if let Err(e) = self.reader.seek_relative(amount as i64) {
            self.fail(e);
        }
    }

    fn jump_to(&mut self, pos: usize) {
        self.seek(SeekFrom::Start(pos as u64));
    }

    fn execute_at<K>(&mut self, pos: usize, func: impl FnOnce(&mut Self) -> K) -> K {
        let offset = self.seek(SeekFrom::Current(0));
        self.jump_to(pos);
        let result = func(self);
        self.jump_to(offset as usize);
        result
    }

    fn read_bytes(&mut self, length: usize) -> Cow<'_, [u8]> {
        Cow::Owned(self.read_vec(length))
    }

    fn is_eof(&mut self) -> bool {
        // An unreadable stream has nothing more to give.
        self.error.is_some() || self.pos() >= self.size()
    }

    fn truncated(&self) -> bool {
        self.truncated || self.error.is_some()
    }
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use super::*;

    #[test]
    fn reading_past_the_end_gives_zeros() {
        let mut des = SliceDeserializer::new(&[1, 2, 3]);
        assert_eq!(des.read_u16_be(), 0x0102);
        assert!(!des.truncated());
        assert_eq!(des.read_u32_be(), 0);
        assert!(des.truncated());
        assert_eq!(&*des.read_bytes(usize::MAX), &[] as &[u8]);

        let mut des = ReaderDeserializer::new(Cursor::new(vec![1, 2, 3]));
        assert_eq!(des.read_u16_be(), 0x0102);
        assert_eq!(des.read_u32_be(), 0);
        assert!(des.truncated());
        assert!(des.error().is_none(), "the end isn't an error");
    }

    #[test]
    fn a_huge_count_stops_at_the_end() {
        let mut data = u32::MAX.to_be_bytes().to_vec();
        data.extend([0; 8]);
        let mut des = SliceDeserializer::new(&data);
        let list = des.read_list(|des| des.read_u32_be());
        assert_eq!(list.len(), 3, "two items and the one that ran out");
        assert!(des.truncated());

        // A reader doesn't allocate the length it's asked for up front.
        let mut des = ReaderDeserializer::new(Cursor::new(vec![7; 4]));
        assert_eq!(des.read_bytes(usize::MAX).len(), 4);
    }
}
