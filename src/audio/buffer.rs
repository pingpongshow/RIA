//! Lock-free ring buffer for audio samples

/// A simple ring buffer for audio samples
pub struct RingBuffer<T> {
    buffer: Vec<T>,
    capacity: usize,
    read_pos: usize,
    write_pos: usize,
    count: usize,
}

impl<T: Clone + Default> RingBuffer<T> {
    /// Create a new ring buffer with the given capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: vec![T::default(); capacity],
            capacity,
            read_pos: 0,
            write_pos: 0,
            count: 0,
        }
    }

    /// Write samples to the buffer
    pub fn write(&mut self, data: &[T]) -> usize {
        let mut written = 0;
        for sample in data {
            if self.count < self.capacity {
                self.buffer[self.write_pos] = sample.clone();
                self.write_pos = (self.write_pos + 1) % self.capacity;
                self.count += 1;
                written += 1;
            } else {
                // Buffer full - overwrite oldest
                self.buffer[self.write_pos] = sample.clone();
                self.write_pos = (self.write_pos + 1) % self.capacity;
                self.read_pos = (self.read_pos + 1) % self.capacity;
                written += 1;
            }
        }
        written
    }

    /// Read samples from the buffer
    pub fn read(&mut self, data: &mut [T]) -> usize {
        let mut read = 0;
        for sample in data.iter_mut() {
            if self.count > 0 {
                *sample = self.buffer[self.read_pos].clone();
                self.read_pos = (self.read_pos + 1) % self.capacity;
                self.count -= 1;
                read += 1;
            } else {
                break;
            }
        }
        read
    }

    /// Peek at samples without removing them
    pub fn peek(&self, data: &mut [T]) -> usize {
        let mut pos = self.read_pos;
        let mut peeked = 0;
        for sample in data.iter_mut() {
            if peeked < self.count {
                *sample = self.buffer[pos].clone();
                pos = (pos + 1) % self.capacity;
                peeked += 1;
            } else {
                break;
            }
        }
        peeked
    }

    /// Get number of samples available
    pub fn available(&self) -> usize {
        self.count
    }

    /// Get free space in buffer
    pub fn free(&self) -> usize {
        self.capacity - self.count
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.read_pos = 0;
        self.write_pos = 0;
        self.count = 0;
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Check if buffer is full
    pub fn is_full(&self) -> bool {
        self.count >= self.capacity
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ring_buffer_basic() {
        let mut buf: RingBuffer<f32> = RingBuffer::new(10);

        assert!(buf.is_empty());
        assert_eq!(buf.available(), 0);

        buf.write(&[1.0, 2.0, 3.0]);
        assert_eq!(buf.available(), 3);

        let mut out = [0.0f32; 2];
        let read = buf.read(&mut out);
        assert_eq!(read, 2);
        assert_eq!(out, [1.0, 2.0]);
        assert_eq!(buf.available(), 1);
    }

    #[test]
    fn test_ring_buffer_wrap() {
        let mut buf: RingBuffer<i32> = RingBuffer::new(4);

        buf.write(&[1, 2, 3]);
        let mut out = [0i32; 2];
        buf.read(&mut out);

        buf.write(&[4, 5, 6]);
        assert_eq!(buf.available(), 4);

        let mut out = [0i32; 4];
        buf.read(&mut out);
        assert_eq!(out, [3, 4, 5, 6]);
    }
}
