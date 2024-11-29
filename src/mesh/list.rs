use std::{marker::PhantomData, ops::{Deref, DerefMut}, usize};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Index<T>
where
    T: Clone
{
    index: usize,
    _data: PhantomData<T>
}
impl<T> std::fmt::Display for Index<T>
where
    T: Clone
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.index)
    }
}
impl<T> Deref for Index<T> 
where
    T: Clone
{
    type Target = usize;
    fn deref(&self) -> &Self::Target {
        &self.index
    }
}
impl<T> DerefMut for Index<T> 
where
    T: Clone
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.index
    }
}
impl<T> From<usize> for Index<T>
where
    T: Clone
{
    fn from(value: usize) -> Self {
        Self { index: value, _data: PhantomData::default() }
    }
}

impl<T> Copy for Index<T> where T: Clone {}

impl<T> Index<T>
where
    T: Clone
{
    pub fn new(index: usize) -> Self {
        Self { index: index, _data: PhantomData::default() }
    }
}

struct ListIndexTransformations<T>(Vec<Index<T>>)
where
    T: Clone;

impl<T> Deref for ListIndexTransformations<T>
where
    T: Clone
{
    type Target = Vec<Index<T>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Debug)]
pub struct List<T>(Vec<Option<T>>)
where
    T: Clone;

impl<T> core::ops::Index<Index<T>> for List<T>
where
    T: Clone
{
    type Output = Option<T>;
    fn index(&self, index: Index<T>) -> &Self::Output {
        &self.0[index.index]
    }
}

impl<T> core::ops::IndexMut<Index<T>> for List<T>
where
    T: Clone
{
    fn index_mut(&mut self, index: Index<T>) -> &mut Self::Output {
        &mut self.0[index.index]
    }
}

impl<T> Default for List<T>
where
    T: Clone
{
    fn default() -> Self {
        Self(Vec::default())
    }
}

impl<T> Deref for List<T>
where
    T: Clone
{
    type Target = Vec<Option<T>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for List<T>
where
    T: Clone
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T> List<T>
where
    T: Clone
{

    pub fn push(&mut self, value: Option<T>) -> Index<T> {
        self.0.push(value);
        Index::new(self.0.len())
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