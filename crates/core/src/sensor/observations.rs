use anyhow::{Result, ensure};
use std::{any::Any, collections::BTreeMap};

pub struct Observation<T> {
    pub sampled_at_s: f64,
    pub delivered_at_s: f64,
    pub value: T,
}

#[derive(Default)]
pub struct Observations {
    values: BTreeMap<String, Box<dyn Any + Send + Sync>>,
}
impl Observations {
    /// Missing data is normal before the first sensor delivery; a wrong type is an error.
    pub fn get<T: Send + Sync + 'static>(&self, topic: &str) -> Result<Option<&Observation<T>>> {
        let Some(value) = self.values.get(topic) else {
            return Ok(None);
        };
        Ok(Some(value.downcast_ref::<Observation<T>>().ok_or_else(
            || anyhow::anyhow!("observation '{topic}' has a different message type"),
        )?))
    }
    pub(crate) fn publish<T: Send + Sync + 'static>(
        &mut self,
        topic: &str,
        time_s: f64,
        value: T,
    ) -> Result<()> {
        ensure!(!topic.is_empty(), "observation topic must not be empty");
        if let Some(previous) = self.values.get(topic) {
            ensure!(
                previous.is::<Observation<T>>(),
                "observation '{topic}' changed message type"
            );
        }
        self.values.insert(
            topic.to_owned(),
            Box::new(Observation {
                sampled_at_s: time_s,
                delivered_at_s: time_s,
                value,
            }),
        );
        Ok(())
    }
    pub(crate) fn deliver(&mut self, pending: &mut Self) -> Result<()> {
        for (topic, value) in &pending.values {
            if let Some(previous) = self.values.get(topic) {
                ensure!(
                    (**previous).type_id() == (**value).type_id(),
                    "observation '{topic}' changed message type"
                );
            }
        }
        self.values.append(&mut pending.values);
        Ok(())
    }
}
