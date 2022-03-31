extern crate parquet;
use crate::msg::image::{Image, ImageRef};
use crate::msg::point_cloud2::{PointCloud2, PointCloud2Ref};
use crate::msg::Msg;
use parquet::file::reader::SerializedFileReader;
use parquet::record::reader::RowIter;
use parquet::record::Row;
use std::fs::File;
use std::path::Path;
use std::sync::Arc;

pub struct ParquetFileReaderRowIter<'a> {
    path: &'a str,
    row_iter: RowIter<'a>,
}

pub trait HasRowIter<'a> {
    fn new(path: &'a str) -> Self;
    fn next_msg(&mut self) -> Option<Arc<dyn Msg>>;
}

impl<'a> ParquetFileReaderRowIter<'a> {
    fn new(path: &'a str) -> Self {
        let file: File = File::open(&Path::new(path)).unwrap();
        let reader = SerializedFileReader::new(file).unwrap();
        let row_iter = reader.into_iter();
        Self { path, row_iter }
    }
    pub fn path(&self) -> &str {
        self.path
    }
}

impl<'a> Iterator for ParquetFileReaderRowIter<'a> {
    type Item = Row;

    fn next(&mut self) -> Option<Self::Item> {
        self.row_iter.next()
    }
}

pub struct PointCloud2ParquetRowIter<'a> {
    parquet_row_iter: ParquetFileReaderRowIter<'a>,
}

impl<'a> HasRowIter<'a> for PointCloud2ParquetRowIter<'a> {
    fn new(path: &'a str) -> Self {
        let parquet_row_iter = ParquetFileReaderRowIter::new(path);
        Self { parquet_row_iter }
    }
    fn next_msg(&mut self) -> Option<Arc<dyn Msg>> {
        match self.next() {
            Some(msg) => Some(msg),
            None => None,
        }
    }
}

impl<'a> Iterator for PointCloud2ParquetRowIter<'a> {
    type Item = PointCloud2Ref;

    fn next(&mut self) -> Option<Self::Item> {
        match self.parquet_row_iter.next() {
            Some(row) => Some(Arc::new(PointCloud2::from(row))),
            None => None,
        }
    }
}

pub struct ImageParquetRowIter<'a> {
    parquet_row_iter: ParquetFileReaderRowIter<'a>,
}

impl<'a> HasRowIter<'a> for ImageParquetRowIter<'a> {
    fn new(path: &'a str) -> Self {
        let parquet_row_iter = ParquetFileReaderRowIter::new(path);
        Self { parquet_row_iter }
    }
    fn next_msg(&mut self) -> Option<Arc<dyn Msg>> {
        match self.next() {
            Some(msg) => Some(msg),
            None => None,
        }
    }
}

impl<'a> Iterator for ImageParquetRowIter<'a> {
    type Item = ImageRef;

    fn next(&mut self) -> Option<Self::Item> {
        match self.parquet_row_iter.next() {
            Some(row) => Some(Arc::new(Image::from(row))),
            None => None,
        }
    }
}
