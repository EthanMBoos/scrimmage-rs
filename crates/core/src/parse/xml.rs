//! XML loading, variable substitution, and XInclude expansion.

use super::Params;
use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use std::{
    fs,
    path::{Path, PathBuf},
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Node {
    pub name: String,
    pub text: String,
    pub attrs: Params,
    pub children: Vec<Node>,
}
impl Node {
    fn from_xml(node: roxmltree::Node) -> Self {
        Self {
            name: node.tag_name().name().into(),
            text: node.text().unwrap_or("").trim().into(),
            attrs: node
                .attributes()
                .map(|a| (a.name().into(), a.value().into()))
                .collect(),
            children: node
                .children()
                .filter(|node| node.is_element())
                .map(Self::from_xml)
                .collect(),
        }
    }
    pub(super) fn params(&self) -> Params {
        self.children
            .iter()
            .map(|node| (node.name.clone(), node.text.clone()))
            .collect()
    }
}
pub fn substitute(input: &str, overrides: &Params) -> Result<String> {
    let mut expanded = String::new();
    let mut remaining = input;
    while let Some(offset) = remaining.find("${") {
        expanded.push_str(&remaining[..offset]);
        remaining = &remaining[offset + 2..];
        let end = remaining
            .find('}')
            .context("unclosed mission substitution")?;
        let expression = &remaining[..end];
        let (key, default) = expression
            .split_once('=')
            .map_or((expression, None), |(key, value)| (key, Some(value)));
        let value = overrides
            .get(key)
            .map(String::as_str)
            .or(default)
            .with_context(|| format!("missing mission variable {key}"))?;
        expanded.push_str(value);
        remaining = &remaining[end + 1..];
    }
    expanded.push_str(remaining);
    Ok(expanded)
}
pub(super) fn read_xml(path: &Path, overrides: &Params, stack: &mut Vec<PathBuf>) -> Result<Node> {
    let path = path
        .canonicalize()
        .with_context(|| format!("mission/include {}", path.display()))?;
    ensure!(
        !stack.contains(&path),
        "cyclic XInclude: {}",
        path.display()
    );
    ensure!(stack.len() < 64, "XInclude depth exceeded");
    stack.push(path.clone());
    let xml = substitute(&fs::read_to_string(&path)?, overrides)?;
    let doc =
        roxmltree::Document::parse(&xml).with_context(|| format!("parse {}", path.display()))?;
    let mut root = Node::from_xml(doc.root_element());
    expand_includes(&mut root, path.parent().unwrap(), overrides, stack)?;
    stack.pop();
    Ok(root)
}
fn expand_includes(
    node: &mut Node,
    base: &Path,
    overrides: &Params,
    stack: &mut Vec<PathBuf>,
) -> Result<()> {
    let base = node
        .attrs
        .get("base")
        .map_or_else(|| base.to_path_buf(), |path| base.join(path));
    let mut children = Vec::new();
    for mut node in std::mem::take(&mut node.children) {
        if node.name == "include" {
            let include_base = node
                .attrs
                .get("base")
                .map_or_else(|| base.clone(), |path| base.join(path));
            let href = node.attrs.get("href").context("XInclude missing href")?;
            let root = read_xml(&include_base.join(href), overrides, stack)?;
            if let Some(selector) = node.attrs.get("xpointer") {
                let path = selector
                    .strip_prefix("xpointer(/")
                    .and_then(|input| input.strip_suffix(')'))
                    .context("unsupported XInclude xpointer")?;
                let mut parts = path.split('/');
                let parent = parts.next().unwrap_or("");
                let child = parts
                    .next()
                    .context("xpointer must select direct children")?;
                ensure!(
                    parts.next().is_none() && (parent == "*" || parent == root.name),
                    "unsupported xpointer {selector}"
                );
                children.extend(
                    root.children
                        .into_iter()
                        .filter(|node| child == "*" || node.name == child),
                );
            } else {
                children.push(root);
            }
        } else {
            expand_includes(&mut node, &base, overrides, stack)?;
            children.push(node);
        }
    }
    node.children = children;
    Ok(())
}
