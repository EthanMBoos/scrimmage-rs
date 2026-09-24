//! Typed accessors for legacy XML parameters.

use anyhow::{Context, Result, bail, ensure};
use std::collections::BTreeMap;

pub type Params = BTreeMap<String, String>;
pub(crate) fn number(params: &Params, key: &str, default: f64) -> Result<f64> {
    let value = match params.get(key) {
        Some(value) => value
            .parse()
            .with_context(|| format!("{key}: expected number, got {value:?}"))?,
        None => default,
    };
    ensure!(value.is_finite(), "{key}: expected finite number");
    Ok(value)
}
pub(crate) fn integer(params: &Params, key: &str, default: usize) -> Result<usize> {
    params.get(key).map_or(Ok(default), |value| {
        value
            .parse()
            .with_context(|| format!("{key}: expected nonnegative integer, got {value:?}"))
    })
}
pub(crate) fn boolean(params: &Params, key: &str, default: bool) -> Result<bool> {
    match params.get(key).map(String::as_str) {
        None => Ok(default),
        Some("true" | "1") => Ok(true),
        Some("false" | "0") => Ok(false),
        Some(value) => bail!("{key}: expected boolean, got {value:?}"),
    }
}
pub(crate) fn vector<const N: usize>(
    params: &Params,
    key: &str,
    default: [f64; N],
) -> Result<[f64; N]> {
    match params.get(key) {
        None => Ok(default),
        Some(value) => {
            let values: Vec<f64> = value
                .split(|c: char| c == ',' || c.is_whitespace())
                .filter(|s| !s.is_empty())
                .map(str::parse)
                .collect::<std::result::Result<_, _>>()
                .with_context(|| format!("invalid vector {key}"))?;
            ensure!(
                values.len() == N && values.iter().all(|x| x.is_finite()),
                "{key}: expected {N} finite values"
            );
            Ok(std::array::from_fn(|i| values[i]))
        }
    }
}
