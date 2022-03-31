extern crate parquet;

use crate::msg::Msg;
use crate::reader::parquet::HasRowIter;
use chrono::{DateTime, Utc};

use std::sync::Arc;

pub struct Reader2<T, U>
where
    T: HasRowIter<'static> + 'static,
    U: HasRowIter<'static> + 'static,
{
    // a_iter: &'a mut T,
    a_iter: T,
    a_last: Option<Arc<dyn Msg>>,
    b_iter: U,
    b_last: Option<Arc<dyn Msg>>,
}

pub trait HasReader2Iter<T, U>
where
    T: HasRowIter<'static> + 'static,
    U: HasRowIter<'static> + 'static,
{
    fn new(a_iter: T, b_iter: U) -> Self;
    fn a_last(&self) -> Option<Arc<dyn Msg>>;
    fn b_last(&self) -> Option<Arc<dyn Msg>>;
    fn a_next(&mut self) -> Option<Arc<dyn Msg>>;
    fn b_next(&mut self) -> Option<Arc<dyn Msg>>;
    fn timestamp(&self, msg: Option<Arc<dyn Msg>>) -> DateTime<Utc>;
}

impl<T, U> HasReader2Iter<T, U> for Reader2<T, U>
where
    T: HasRowIter<'static> + 'static,
    U: HasRowIter<'static> + 'static,
{
    fn new(a_iter: T, b_iter: U) -> Self {
        // initialise with first rows of each
        let mut a_iter = a_iter;
        let mut b_iter = b_iter;
        let a_last = a_iter.next_msg();
        let b_last = b_iter.next_msg();
        Self {
            a_iter,
            a_last,
            b_iter,
            b_last,
        }
    }

    // pub fn from(a_file_name:&'a str, b_file_name:&'a str) -> Self {
    //   let mut a_iter = T::new(a_file_name);
    //   let a_last = a_iter.next_msg();
    //   let mut b_iter = U::new(b_file_name);
    //   let b_last= b_iter.next_msg();
    //   Self { a_iter,  a_last, b_iter, b_last }
    // }

    fn a_last(&self) -> Option<Arc<dyn Msg>> {
        self.a_last.clone()
    }

    fn b_last(&self) -> Option<Arc<dyn Msg>> {
        self.b_last.clone()
    }

    fn a_next(&mut self) -> Option<Arc<dyn Msg>> {
        match self.a_last.clone() {
            Some(a_last) => {
                self.a_last = self.a_iter.next_msg();
                self.a_last.clone()
            },
            None => None,
        }
    }
    fn b_next(&mut self) -> Option<Arc<dyn Msg>> {
        match self.b_last.clone() {
            Some(b_last) => {
                self.b_last = self.b_iter.next_msg();
                self.b_last.clone()
            },
            None => None,
        }
    }

    fn timestamp(&self, msg: Option<Arc<dyn Msg>>) -> DateTime<Utc> {
        match msg {
            Some(msg) => msg.timestamp(),
            None => chrono::MAX_DATE.and_hms(23, 59, 59),
        }
    }
}

impl<T, U> Iterator for Reader2<T, U>
where
    T: HasRowIter<'static> + Iterator + 'static,
    U: HasRowIter<'static> + Iterator + 'static,
{
    type Item = Arc<dyn Msg>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.a_last.is_none() && self.b_last.is_none() {
            return None;
        }

        if self.timestamp(self.a_last.clone()) < self.timestamp(self.b_last.clone()) {
            self.a_next();
        } else {
            self.b_next();
        }

        let a_timestamp = self.timestamp(self.a_last.clone());
        let b_timestamp = self.timestamp(self.b_last.clone());


        if a_timestamp < b_timestamp
        {
            self.a_last.clone()
        } else {
            self.b_last.clone()
        }
    }
}
