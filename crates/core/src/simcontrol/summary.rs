//! Formatting is independent of metric implementations.
use super::Simulation;
use std::collections::{BTreeMap, BTreeSet};

impl Simulation {
    pub fn summary_csv(&self) -> String {
        let reports = self.metric_reports();
        if reports.is_empty() {
            return String::new();
        }
        let mut occurrences = BTreeMap::new();
        for (_, report) in &reports {
            for header in &report.headers {
                *occurrences.entry(header).or_insert(0) += 1;
            }
        }
        let mut columns = Vec::new();
        let mut teams = BTreeSet::new();
        for (instance, report) in &reports {
            for header in &report.headers {
                let name = if occurrences[header] > 1 {
                    format!("{instance}/{header}")
                } else {
                    header.clone()
                };
                columns.push(name);
            }
            teams.extend(report.teams.keys().copied());
        }
        let mut csv = format!("team_id,score,{}\n", columns.join(","));
        for team_id in teams {
            let score: f64 = reports
                .iter()
                .filter_map(|(_, report)| report.teams.get(&team_id))
                .map(|team| team.score)
                .sum();
            csv.push_str(&format!("{team_id},{score:.6}"));
            for (_, report) in &reports {
                for header in &report.headers {
                    let value = report
                        .teams
                        .get(&team_id)
                        .and_then(|team| team.values.get(header))
                        .copied()
                        .unwrap_or(0.0);
                    csv.push_str(&format!(",{value:.6}"));
                }
            }
            csv.push('\n');
        }
        csv
    }
}
