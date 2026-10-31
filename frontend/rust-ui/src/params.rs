//! Parameter forms generated from the engine's parameter declarations, so the UI shows exactly
//! the parameters the selected algorithm or problem accepts — and new algorithms need no UI code.

use std::collections::BTreeMap;

/// Operator names valid for the current problem, per parameter name (mutation, crossover, neighbourhood).
pub type Operators = BTreeMap<String, Vec<String>>;

use eframe::egui;
use serde_json::{json, Value};

use crate::model::ParamSpec;

pub type Values = BTreeMap<String, Value>;

/// Declared defaults for every parameter.
pub fn defaults(specs: &[ParamSpec]) -> Values {
    specs.iter().map(|s| (s.name.clone(), s.default_value.clone())).collect()
}

/// Choices for an operator parameter: "default" plus the operators offered by the problem.
fn operator_options(spec: &ParamSpec, operators: Option<&Operators>) -> Option<Vec<String>> {
    let ops = operators?.get(&spec.name)?;
    let mut options = vec!["default".to_string()];
    options.extend(ops.iter().cloned());
    Some(options)
}

/// Like [`to_json`], but operator names that the problem does not offer fall back to "default"
/// (e.g. a "gaussian" mutation remembered from a real-vector problem when scheduling permutations).
pub fn to_json_for(specs: &[ParamSpec], values: &Values, operators: Option<&Operators>) -> Value {
    let mut out = to_json(specs, values);
    if let Value::Object(map) = &mut out {
        for spec in specs {
            if let (Some(options), Some(v)) = (operator_options(spec, operators), map.get_mut(&spec.name)) {
                if !options.iter().any(|o| Some(o.as_str()) == v.as_str()) {
                    *v = json!("default");
                }
            }
        }
    }
    out
}

/// Values restricted to declared parameters (the engine rejects unknown names).
pub fn to_json(specs: &[ParamSpec], values: &Values) -> Value {
    let mut out = serde_json::Map::new();
    for spec in specs {
        if let Some(v) = values.get(&spec.name) {
            out.insert(spec.name.clone(), v.clone());
        }
    }
    Value::Object(out)
}

/// Renders an editable grid. Names listed in `skip` are rendered by the caller.
pub fn form(ui: &mut egui::Ui, id: &str, specs: &[ParamSpec], values: &mut Values, skip: &[&str]) -> bool {
    form_for(ui, id, specs, values, skip, None)
}

/// Renders an editable grid whose operator choices are limited to what the problem offers.
pub fn form_for(ui: &mut egui::Ui, id: &str, specs: &[ParamSpec], values: &mut Values, skip: &[&str],
                operators: Option<&Operators>) -> bool {
    let mut changed = false;
    egui::Grid::new(id).num_columns(2).spacing([10.0, 6.0]).striped(false).show(ui, |ui| {
        for spec in specs.iter().filter(|s| !skip.contains(&s.name.as_str())) {
            ui.label(pretty(&spec.name)).on_hover_text(&spec.description);
            let entry = values.entry(spec.name.clone()).or_insert_with(|| spec.default_value.clone());
            match operator_options(spec, operators) {
                Some(options) => {
                    if !options.iter().any(|o| Some(o.as_str()) == entry.as_str()) {
                        *entry = json!("default");
                    }
                    let restricted = ParamSpec { options, ..spec.clone() };
                    changed |= widget(ui, id, &restricted, entry);
                }
                None => changed |= widget(ui, id, spec, entry),
            }
            ui.end_row();
        }
    });
    changed
}

fn widget(ui: &mut egui::Ui, id: &str, spec: &ParamSpec, value: &mut Value) -> bool {
    let response = match spec.kind.as_str() {
        "INT" => {
            let mut v = value.as_i64().unwrap_or(0);
            let lo = spec.min.unwrap_or(i64::MIN as f64) as i64;
            let hi = spec.max.unwrap_or(i64::MAX as f64) as i64;
            let r = ui.add(egui::DragValue::new(&mut v).range(lo..=hi).speed(speed(spec)));
            *value = json!(v);
            r
        }
        "DOUBLE" => {
            let mut v = value.as_f64().unwrap_or(0.0);
            let lo = spec.min.unwrap_or(f64::MIN);
            let hi = spec.max.unwrap_or(f64::MAX);
            let r = ui.add(egui::DragValue::new(&mut v).range(lo..=hi).speed(speed(spec)).max_decimals(6));
            *value = json!(v);
            r
        }
        "BOOL" => {
            let mut v = value.as_bool().unwrap_or(false);
            let r = ui.checkbox(&mut v, "");
            *value = json!(v);
            r
        }
        "CHOICE" => {
            let mut current = value.as_str().unwrap_or_default().to_string();
            let before = current.clone();
            egui::ComboBox::from_id_salt((id, &spec.name)).selected_text(&current).width(160.0).show_ui(ui, |ui| {
                for option in &spec.options {
                    ui.selectable_value(&mut current, option.clone(), option);
                }
            });
            *value = json!(current);
            return before != value.as_str().unwrap_or_default();
        }
        _ => {
            let mut s = value.as_str().unwrap_or_default().to_string();
            let r = ui.text_edit_singleline(&mut s);
            *value = json!(s);
            r
        }
    };
    response.on_hover_text(&spec.description).changed()
}

fn speed(spec: &ParamSpec) -> f64 {
    match (spec.min, spec.max) {
        (Some(lo), Some(hi)) if hi.is_finite() && lo.is_finite() && hi > lo => ((hi - lo) / 400.0).clamp(0.0005, 50.0),
        _ => 0.05,
    }
}

/// `populationSize` -> `Population size`.
pub fn pretty(name: &str) -> String {
    let mut out = String::new();
    for (i, c) in name.chars().enumerate() {
        if i == 0 {
            out.extend(c.to_uppercase());
        } else if c.is_uppercase() {
            out.push(' ');
            out.extend(c.to_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(name: &str, kind: &str, default: Value) -> ParamSpec {
        ParamSpec {
            name: name.into(),
            kind: kind.into(),
            default_value: default,
            min: None,
            max: None,
            options: vec![],
            description: String::new(),
        }
    }

    #[test]
    fn only_declared_parameters_are_sent() {
        let specs = vec![spec("populationSize", "INT", json!(50)), spec("selection", "CHOICE", json!("tournament"))];
        let mut values = defaults(&specs);
        values.insert("stale".into(), json!(1));
        let out = to_json(&specs, &values);
        assert_eq!(out, json!({"populationSize": 50, "selection": "tournament"}));
    }

    #[test]
    fn operators_unknown_to_the_problem_fall_back_to_default() {
        let mut spec_m = spec("mutation", "CHOICE", json!("default"));
        spec_m.options = vec!["default".into(), "gaussian".into(), "swap".into()];
        let specs = vec![spec_m];
        let mut values = Values::new();
        values.insert("mutation".into(), json!("gaussian"));
        let mut ops = Operators::new();
        ops.insert("mutation".into(), vec!["swap".into(), "insert".into()]);
        assert_eq!(to_json_for(&specs, &values, Some(&ops)), json!({"mutation": "default"}));
        values.insert("mutation".into(), json!("insert"));
        assert_eq!(to_json_for(&specs, &values, Some(&ops)), json!({"mutation": "insert"}));
        assert_eq!(to_json_for(&specs, &values, None), json!({"mutation": "insert"}));
    }

    #[test]
    fn names_are_prettified() {
        assert_eq!(pretty("populationSize"), "Population size");
        assert_eq!(pretty("stepSize"), "Step size");
    }
}
