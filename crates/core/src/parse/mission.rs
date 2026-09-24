use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};

use super::xml::{Node, read_xml};
use super::{Params, boolean, integer, number};
use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use crate::{
    entity::EntityDefinition, simcontrol::generation::Generator,
    simcontrol::world_plugins::CompiledWorld,
};

/// A scenario that has passed parameter and plugin-connection validation.
/// Its fields cannot be constructed or modified by a caller.
pub struct ResolvedScenario {
    pub(crate) config: ScenarioConfig,
    pub(crate) definitions: Vec<EntityDefinition>,
    pub(crate) generators: Vec<Generator>,
    pub(crate) world: CompiledWorld,
    pub(crate) end_conditions: EndConditions,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct EndConditions {
    pub time: bool,
    pub all_dead: bool,
    pub one_team: bool,
}

impl EndConditions {
    fn parse(params: &Params) -> Result<Self> {
        let specification = params
            .get("end_condition")
            .map(String::as_str)
            .unwrap_or("time");
        let mut conditions = Self::default();
        for condition in specification.split(',').map(str::trim) {
            match condition {
                "time" => conditions.time = true,
                "all_dead" => conditions.all_dead = true,
                "one_team" => conditions.one_team = true,
                "none" => {}
                invalid => bail!("unknown end condition: {invalid}"),
            }
        }
        Ok(conditions)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct PluginConfig {
    pub name: String,
    pub params: Params,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(crate) struct EntityConfig {
    pub params: Params,
    pub autonomy: Vec<PluginConfig>,
    pub controllers: Vec<PluginConfig>,
    pub motion: PluginConfig,
    pub sensors: Vec<PluginConfig>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenarioConfig {
    pub source: PathBuf,
    pub start_s: f64,
    pub end_s: f64,
    pub dt_s: f64,
    pub motion_multiplier: usize,
    pub seed: u32,
    pub worker_count: usize,
    pub enable_gui: bool,
    pub time_warp: f64,
    pub start_paused: bool,
    pub params: Params,
    pub(crate) entities: Vec<EntityConfig>,
    pub(crate) interactions: Vec<PluginConfig>,
    pub(crate) networks: Vec<PluginConfig>,
    pub(crate) metrics: Vec<PluginConfig>,
    pub(crate) expanded_xml: Node,
}
fn collect_xml(directory: &Path, paths_by_name: &mut BTreeMap<String, PathBuf>) -> Result<()> {
    if !directory.is_dir() {
        return Ok(());
    }
    let mut entries = fs::read_dir(directory)?.collect::<std::io::Result<Vec<_>>>()?;
    entries.sort_by_key(|entry| entry.file_name());

    for entry in entries {
        let path = entry.path();
        if entry.file_type()?.is_dir() {
            collect_xml(&path, paths_by_name)?;
        } else if path.extension().is_some_and(|extension| extension == "xml") {
            let name = path
                .file_stem()
                .context("XML filename has no stem")?
                .to_string_lossy()
                .into_owned();
            paths_by_name.entry(name).or_insert(path);
        }
    }
    Ok(())
}

impl ScenarioConfig {
    pub fn resolve(self) -> Result<ResolvedScenario> {
        self.resolve_with_registry(&crate::plugin::PluginRegistry::with_builtins())
    }

    pub fn resolve_with_registry(
        self,
        registry: &crate::plugin::PluginRegistry,
    ) -> Result<ResolvedScenario> {
        self.validate_supported()?;
        ensure!(
            self.dt_s.is_finite() && self.dt_s > 0.0,
            "dt must be positive and finite"
        );
        ensure!(
            self.start_s.is_finite() && self.end_s.is_finite() && self.end_s >= self.start_s,
            "invalid simulation interval"
        );
        ensure!(
            self.motion_multiplier > 0,
            "motion_multiplier must be positive"
        );
        ensure!(self.worker_count > 0, "worker count must be positive");

        let mut definitions = Vec::new();
        let mut generators = Vec::new();
        for (index, entity) in self.entities.iter().enumerate() {
            definitions.push(
                EntityDefinition::parse(entity, registry)
                    .with_context(|| format!("entity block {index}"))?,
            );
            generators.push(Generator::new(entity, index, self.start_s)?);
        }

        let world = CompiledWorld::compile(&self, registry)?;
        let end_conditions = EndConditions::parse(&self.params)?;

        Ok(ResolvedScenario {
            config: self,
            definitions,
            generators,
            world,
            end_conditions,
        })
    }

    pub fn load(path: &Path, root: &Path, overrides: &Params) -> Result<Self> {
        Self::load_with_registry(
            path,
            root,
            overrides,
            &crate::plugin::PluginRegistry::with_builtins(),
        )
    }

    pub fn load_with_registry(
        path: &Path,
        root: &Path,
        overrides: &Params,
        registry: &crate::plugin::PluginRegistry,
    ) -> Result<Self> {
        let mut candidates = vec![path.to_path_buf()];
        if let Some(paths) = std::env::var_os("SCRIMMAGE_MISSION_PATH") {
            candidates.extend(std::env::split_paths(&paths).map(|plugin| plugin.join(path)));
        }
        candidates.push(root.join("missions").join(path));
        let path = candidates
            .iter()
            .find(|plugin| plugin.is_file())
            .with_context(|| format!("mission not found: {}", path.display()))?;
        let expanded_xml = read_xml(path, overrides, &mut Vec::new())?;
        ensure!(expanded_xml.name == "runscript", "expected runscript root");
        let run = &expanded_xml
            .children
            .iter()
            .find(|node| node.name == "run")
            .context("missing run element")?
            .attrs;
        let mut plugins = BTreeMap::new();
        if let Some(paths) = std::env::var_os("SCRIMMAGE_PLUGIN_PATH") {
            for plugin_path in std::env::split_paths(&paths) {
                collect_xml(&plugin_path, &mut plugins)?;
            }
        }
        // Built-in XML defaults are required: the Rust fallback defaults are not kept
        // in sync with them, so a missing folder would silently change the mission.
        let builtin_defaults = root.join("crates/core/src/plugin");
        ensure!(
            builtin_defaults.is_dir(),
            "built-in plugin defaults not found at {}; pass --root <scrimmage-rs checkout>",
            builtin_defaults.display()
        );
        collect_xml(&builtin_defaults, &mut plugins)?;
        let mut parameter_groups = BTreeMap::new();
        let mut entity_groups = BTreeMap::new();
        for node in &expanded_xml.children {
            let Some(name) = node.attrs.get("name") else {
                continue;
            };
            match node.name.as_str() {
                "param_common" => {
                    parameter_groups.insert(name, node.params());
                }
                "entity_common" => {
                    entity_groups.insert(name, node);
                }
                _ => {}
            }
        }

        let parse_plugin = |node: &Node| -> Result<PluginConfig> {
            let name = &node.text;
            ensure!(
                registry.contains_in_category(&node.name, name),
                "Mission requests {} '{name}', but it isn't registered in this executable.",
                node.name
            );
            let mut params = if let Some(path) = plugins.get(name) {
                read_xml(path, overrides, &mut Vec::new())?.params()
            } else {
                Params::new()
            };

            if let Some(group_name) = node.attrs.get("param_common") {
                let group = parameter_groups
                    .get(group_name)
                    .with_context(|| format!("unknown param_common {group_name}"))?;
                params.extend(
                    group
                        .iter()
                        .map(|(key, value)| (key.clone(), value.clone())),
                );
            }
            params.extend(
                node.attrs
                    .iter()
                    .map(|(key, value)| (key.clone(), value.clone())),
            );
            Ok(PluginConfig {
                name: name.clone(),
                params,
            })
        };

        let mut entities = Vec::new();
        for entity_node in expanded_xml
            .children
            .iter()
            .filter(|node| node.name == "entity")
        {
            let mut children = Vec::new();
            if let Some(group_name) = entity_node.attrs.get("entity_common") {
                let group = entity_groups
                    .get(group_name)
                    .with_context(|| format!("unknown entity_common {group_name}"))?;
                children.extend(group.children.iter());
            }
            children.extend(entity_node.children.iter());

            let mut entity_params = Params::new();
            let mut autonomy = Vec::new();
            let mut controllers = Vec::new();
            let mut sensors = Vec::new();
            let mut motion = None;
            for node in children {
                entity_params.insert(node.name.clone(), node.text.clone());
                match node.name.as_str() {
                    "autonomy" => autonomy.push(parse_plugin(node)?),
                    "controller" => controllers.push(parse_plugin(node)?),
                    "sensor" => sensors.push(parse_plugin(node)?),
                    "motion_model" => {
                        ensure!(motion.is_none(), "entity requires one motion_model");
                        motion = Some(parse_plugin(node)?);
                    }
                    _ => {}
                }
            }
            entities.push(EntityConfig {
                params: entity_params,
                autonomy,
                controllers,
                sensors,
                motion: motion.context("entity requires one motion_model")?,
            });
        }

        let mut interactions = Vec::new();
        let mut networks = Vec::new();
        let mut metrics = Vec::new();
        for node in &expanded_xml.children {
            match node.name.as_str() {
                "entity_interaction" => interactions.push(parse_plugin(node)?),
                "network" => networks.push(parse_plugin(node)?),
                "metrics" => metrics.push(parse_plugin(node)?),
                _ => {}
            }
        }
        let params = expanded_xml.params();
        let start_s = number(run, "start", 0.0)?;
        let end_s = number(run, "end", 100.0)?;
        let dt_s = number(run, "dt", 0.1)?;
        ensure!(
            dt_s > 0.0 && end_s >= start_s,
            "run requires dt > 0 and end >= start"
        );
        let motion_multiplier = integer(run, "motion_multiplier", 1)?;
        ensure!(motion_multiplier > 0, "motion_multiplier must be positive");
        let multithread = expanded_xml
            .children
            .iter()
            .find(|node| node.name == "multi_threaded");
        let worker_count = if boolean(&params, "multi_threaded", false)? {
            integer(
                &multithread
                    .context("missing threading configuration")?
                    .attrs,
                "num_threads",
                1,
            )?
        } else {
            1
        };
        ensure!(worker_count > 0, "num_threads must be positive");
        let seed = integer(&params, "seed", 2_147_483_648)?;
        ensure!(seed <= u32::MAX as usize, "seed exceeds uint32");
        Ok(Self {
            source: path.canonicalize()?,
            start_s,
            end_s,
            dt_s,
            motion_multiplier,
            seed: seed as u32,
            worker_count,
            enable_gui: boolean(run, "enable_gui", false)?,
            time_warp: number(run, "time_warp", 0.0)?,
            start_paused: boolean(run, "start_paused", false)?,
            params,
            entities,
            interactions,
            networks,
            metrics,
            expanded_xml,
        })
    }
    fn validate_supported(&self) -> Result<()> {
        let mut unsupported: Vec<String> = Vec::new();
        for entity in &self.entities {
            if entity.params.contains_key("gpu_motion_model") {
                unsupported.push("GPU motion execution".into());
            }
            if entity.params.contains_key("latitude") || entity.params.contains_key("longitude") {
                unsupported.push("geodetic entity initialization".into());
            }
        }
        ensure!(
            unsupported.is_empty(),
            "not yet implemented: {}",
            unsupported.join(", ")
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugin::PluginRegistry;

    fn root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    #[test]
    fn bundled_defaults_belong_to_implemented_plugins() -> Result<()> {
        let mut plugins = BTreeMap::new();
        collect_xml(&root().join("crates/core/src/plugin"), &mut plugins)?;
        assert!(!plugins.is_empty(), "no bundled plugin defaults found");
        let registry = PluginRegistry::with_builtins();
        for name in plugins.keys() {
            assert!(
                registry.contains(name),
                "defaults without a Rust plugin: {name}"
            );
        }
        Ok(())
    }

    #[test]
    fn colocated_defaults_and_mission_overrides_are_preserved() -> Result<()> {
        let root = root();
        let mission = ScenarioConfig::load(
            &root.join("crates/core/tests/fixtures/plugin_defaults.xml"),
            &root,
            &Params::new(),
        )?;
        let defaults = &mission.entities[0];
        assert_eq!(defaults.autonomy[0].params["speed"], "21");
        assert_eq!(
            defaults.controllers[0].params["heading_pid"],
            "1, 0.01, 2, 9"
        );
        assert_eq!(defaults.motion.params["turning_radius"], "13");
        assert_eq!(defaults.motion.params["max_velocity"], "40");
        assert_eq!(defaults.sensors[0].params["stddev_m"], "0.1");
        assert_eq!(defaults.sensors[0].params["bias_world_m"], "0 0 0");
        assert_eq!(defaults.sensors[0].params["topic"], "position");
        assert_eq!(mission.interactions[0].params["collision_range"], "2");
        assert_eq!(mission.metrics[0].params["team_collisions_w"], "-1.0");
        assert_eq!(mission.networks[0].params["library"], "LocalNetwork_plugin");
        assert_eq!(
            mission.networks[1].params["library"],
            "GlobalNetwork_plugin"
        );

        let overrides = &mission.entities[1];
        assert_eq!(overrides.autonomy[0].params["speed"], "26");
        assert_eq!(overrides.motion.params["turning_radius"], "23");
        assert_eq!(overrides.motion.params["min_velocity"], "16");
        assert_eq!(overrides.motion.params["max_velocity"], "40");
        assert_eq!(overrides.sensors[0].params["stddev_m"], "0");
        mission.resolve()?;
        Ok(())
    }

    #[test]
    fn missing_bundled_defaults_are_an_error() {
        let root = root();
        let error = ScenarioConfig::load(
            &root.join("missions/straight-no-gui.xml"),
            &root.join("missions"),
            &Params::new(),
        )
        .unwrap_err();
        assert!(
            error
                .to_string()
                .contains("built-in plugin defaults not found")
        );
    }

    #[test]
    fn earlier_plugin_search_paths_override_bundled_defaults() -> Result<()> {
        let root = root();
        let overlay = root.join("crates/core/tests/fixtures/plugin_overlay");
        let mut plugins = BTreeMap::new();
        collect_xml(&overlay, &mut plugins)?;
        collect_xml(&root.join("crates/core/src/plugin"), &mut plugins)?;
        assert_eq!(
            plugins["SimpleAircraft"],
            overlay.join("SimpleAircraft.xml")
        );
        let defaults = read_xml(&plugins["SimpleAircraft"], &Params::new(), &mut Vec::new())?;
        assert_eq!(defaults.params()["turning_radius"], "37");
        assert!(plugins.contains_key("Straight"));
        Ok(())
    }
}
