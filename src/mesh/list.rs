use std::{marker::PhantomData, ops::{Deref, DerefMut}, usize};

#[derive(Debug, Hash)]
pub struct Index<T> {
    index: usize,
    _data: PhantomData<T>
}
impl<T> std::fmt::Display for Index<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.index)
    }
}
impl<T> Deref for Index<T> {
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.index
    }
}
impl<T> DerefMut for Index<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.index
    }
}
impl<T> From<usize> for Index<T> {
    fn from(value: usize) -> Self {
        Self { index: value, _data: PhantomData::default() }
    }
}

impl<T> From<Index<T>> for usize {
    fn from(value: Index<T>) -> Self {
        value.index
    }
}

impl<T> Copy for Index<T> {}
impl<T> Clone for Index<T> {
    fn clone(&self) -> Self {
        Self { index: self.index, _data: PhantomData }
    }
}

impl<T> PartialEq for Index<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}
impl<T> Eq for Index<T> {}

impl<T> Index<T> {
    pub fn new(index: usize) -> Self {
        Self { index: index, _data: PhantomData::default() }
    }
}

struct ListIndexTransformations<T>(Vec<Index<T>>);

impl<T> Deref for ListIndexTransformations<T> {
    type Target = Vec<Index<T>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct List<T, I = T>(Vec<Option<T>>, PhantomData<I>);

impl<T, I> core::ops::Index<Index<I>> for List<T, I> {
    type Output = Option<T>;
    fn index(&self, index: Index<I>) -> &Self::Output {
        &self.0[index.index]
    }
}

impl<T, I> core::ops::IndexMut<Index<I>> for List<T, I> {
    fn index_mut(&mut self, index: Index<I>) -> &mut Self::Output {
        &mut self.0[index.index]
    }
}

impl<T, I> Default for List<T, I> {
    fn default() -> Self {
        Self(Vec::default(), PhantomData)
    }
}

impl<T, I> Deref for List<T, I> {
    type Target = Vec<Option<T>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, I> DerefMut for List<T, I> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

// impl<T> Iterator for List<T>
// where 
//     T: Clone
// {
//     type Item = T;

//     fn next(&mut self) -> Option<Self::Item> {
//         for item in &mut self.0 {
//             if item.is_some() {
//                 return std::mem::replace(item, None);
//             }
//         }
//         return None;
//     }
// }

impl<T, I> List<T, I> {
    pub fn new(values: Vec<Option<T>>) -> Self {
        Self(values, PhantomData)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self(Vec::with_capacity(capacity), PhantomData)
    }

    pub fn with_defaults(len: usize) -> Self {
        Self((0..len).map(|_| None).collect(), PhantomData)
    }

    pub fn take(self) -> Vec<Option<T>> {
        self.0
    }

    pub fn push(&mut self, value: Option<T>) -> Index<T> {
        self.0.push(value);
        Index::new(self.0.len()-1)
    }

    pub fn compact(&mut self) -> ListIndexTransformations<T> {
        let mut transformations: Vec<Index<T>> = (0..self.0.len()).map(|i| i.into()).collect();
        let (mut i, mut j) = (0, self.0.len() - 1);
        while i <= j {
            match &self.0[i] {
                Some(_) => {
                    transformations[i] = i.into();
                },
                None => {
                    while self.0[j].is_none() {
                        j -= 1
                    } 
                    let item = std::mem::replace(&mut self.0[j], None);
                    std::mem::replace(&mut self.0[i], item);
                    transformations[j] = j.into();
                    j -= 1
                }
            }
            i += 1;
        }
        ListIndexTransformations(transformations)
    }
}